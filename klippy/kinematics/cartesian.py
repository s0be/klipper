# Code for handling the kinematics of cartesian robots
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper, homing
from toolhead import Axis


class CartKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()

        # Setup boundary checks
        max_velocity = toolhead.get_max_velocity()
        max_z_velocity = config.getfloat('max_z_velocity', max_velocity, above=0., maxval=max_velocity)

        max_accel = toolhead.get_max_accel()
        max_z_accel = config.getfloat('max_z_accel', max_accel, above=0., maxval=max_accel)

        # Setup axis rails
        self.rail_x = stepper.LookupMultiRail(config.getsection('stepper_x'))
        self.rail_x.setup_itersolve('cartesian_stepper_alloc', 'x')
        self.axis_x = Axis('x', self,
                           self.rail_x.set_position,
                           self.rail_x.get_commanded_position,
                           lambda: not self.rail_x.is_motor_enabled(),
                           toolhead.get_max_velocity,
                           toolhead.get_max_accel,
                           True)

        self.rail_y = stepper.LookupMultiRail(config.getsection('stepper_y'))
        self.rail_y.setup_itersolve('cartesian_stepper_alloc', 'y')
        self.axis_y = Axis('y', self,
                           self.rail_y.set_position,
                           self.rail_y.get_commanded_position,
                           lambda: not self.rail_y.is_motor_enabled(),
                           toolhead.get_max_velocity,
                           toolhead.get_max_accel,
                           True)

        self.rail_z = stepper.LookupMultiRail(config.getsection('stepper_z'))
        self.rail_z.setup_itersolve('cartesian_stepper_alloc', 'z')
        self.axis_z = Axis('z', self,
                           self.rail_z.set_position,
                           self.rail_z.get_commanded_position,
                           lambda: not self.rail_z.is_motor_enabled(),
                           lambda: min(max_z_velocity, toolhead.get_max_velocity()),
                           lambda: min(max_z_accel, toolhead.get_max_accel()),
                           True)

        # Setup stepper max halt velocity
        max_halt_velocity = toolhead.get_max_axis_halt()
        self.rail_x.set_max_jerk(max_halt_velocity, max_accel)
        self.rail_y.set_max_jerk(max_halt_velocity, max_accel)
        self.rail_z.set_max_jerk(min(max_halt_velocity, self.axis_z.get_max_velocity()), max_accel)

        # Check for dual carriage support
        self.dual_carriage_axis = None
        self.dual_carriage_rail = None
        self.dual_carriage_active = False
        if config.has_section('dual_carriage'):
            dc_config = config.getsection('dual_carriage')
            self.dual_carriage_axis = dc_config.getchoice('axis', {'x': self.axis_x, 'y': self.axis_y})
            self.dual_carriage_rail = stepper.LookupMultiRail(dc_config)
            self.dual_carriage_rail.setup_itersolve('cartesian_stepper_alloc', self.dual_carriage_axis)
            self.dual_carriage_rail.set_max_jerk(max_halt_velocity, max_accel)
            self.printer.lookup_object('gcode').register_command(
                'SET_DUAL_CARRIAGE', self.cmd_SET_DUAL_CARRIAGE,
                desc=self.cmd_SET_DUAL_CARRIAGE_help)

            axis, rail = None, None
            if self.dual_carriage_axis is self.axis_x:
                axis = self.axis_x
                rail = self.rail_x
            else:
                axis = self.axis_y
                rail = self.rail_y

            def get_active_rail():
                return self.dual_carriage_rail if self.dual_carriage_active else rail

            def set_pos(position):
                active_rail = get_active_rail()
                active_rail.set_position(position)
                if axis.is_homing:
                    axis.limits = active_rail.get_range()

            axis.set_position = set_pos

            def get_pos():
                active_rail = get_active_rail()
                return active_rail.get_commanded_position()

            axis.get_commanded_position = get_pos

            def enable_needed():
                active_rail = get_active_rail()
                return not active_rail.is_motor_enabled()

            axis.get_need_enable = enable_needed

        # Register axi with printer
        self.printer.add_object('axis %s' % self.axis_x.label, self.axis_x)
        self.printer.add_object('axis %s' % self.axis_y.label, self.axis_y)
        self.printer.add_object('axis %s' % self.axis_z.label, self.axis_z)

        self.axes = {
            self.axis_x: self.rail_x,
            self.axis_y: self.rail_y,
            self.axis_z: self.rail_z,
        }

    def get_steppers(self, flags=""):
        return [stepper for rail in self.axes.viewvalues()
                for stepper in rail.get_steppers()
                if rail.name in flags or not flags]

    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            rail = self.axes[axis]
            if axis == self.dual_carriage_axis:
                dual_active = self.dual_carriage_active
                self._activate_carriage(rail)
                self._home_axis(homing_state, axis, rail)
                self._activate_carriage(self.dual_carriage_rail)
                self._home_axis(homing_state, axis, self.dual_carriage_rail)
                if not dual_active:
                    self._activate_carriage(rail)
            else:
                self._home_axis(homing_state, axis, rail)

    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = hi.position_endstop
        forcepos = hi.position_endstop
        if hi.positive_dir:
            forcepos -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos += 1.5 * (position_max - hi.position_endstop)

        homing_state.home_rails([rail], {axis: (forcepos, homepos)})

    def motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
        for rail in self.rails:
            rail.motor_enable(print_time, 0)
        for rail in self.dual_carriage_rails:
            rail.motor_enable(print_time, 0)
        self.need_motor_enable = True

    def _check_motor_enable(self, print_time, move):
        need_motor_enable = False
        for i, rail in enumerate(self.rails):
            if move.axes_d[i]:
                rail.motor_enable(print_time, 1)
            need_motor_enable |= not rail.is_motor_enabled()
        self.need_motor_enable = need_motor_enable

    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                    and (end_pos[i] < self.limits[i][0]
                         or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise homing.EndstopMoveError(
                        end_pos, "Must home axis first")
                raise homing.EndstopMoveError(end_pos)

    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
                or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)

    def move(self, print_time, move):
        if self.need_motor_enable:
            self._check_motor_enable(print_time, move)
        for i, rail in enumerate(self.rails):
            if move.axes_d[i]:
                rail.step_itersolve(move.cmove)

    # Dual carriage support
    def _activate_carriage(self, rail):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.get_last_move_time()
        axis_min, axis_max = self.dual_carriage_axis.limits
        if axis_min <= axis_max:
            self.dual_carriage_axis.limits = rail.get_range()
        self.dual_carriage_active = rail is self.dual_carriage_rail

    cmd_SET_DUAL_CARRIAGE_help = "Set which carriage is active"

    def cmd_SET_DUAL_CARRIAGE(self, params):
        gcode = self.printer.lookup_object('gcode')
        carriage = gcode.get_int('CARRIAGE', params, minval=0, maxval=1)
        rail = self.rail_x if self.dual_carriage_axis is self.axis_x else self.rail_y
        self._activate_carriage(self.dual_carriage_rail if carriage else rail)
        gcode.reset_last_position()


def load_kinematics(toolhead, config):
    return CartKinematics(toolhead, config)
