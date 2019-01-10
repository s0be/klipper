# Filament selector based on Idler/Pulley/Selector style control
#
# Copyright (C) 2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import chelper
import homing
import logging
import stepper


class IdlerPulleyFilamentSelector:
    def __init__(self, filament_toolhead, config):
        self.printer = config.get_printer()
        self.printer.add_object("ips_filament_selector", self)
        self.gcode = self.printer.lookup_object('gcode')
        # Setup axis rails
        self.axis_i = AuxiliaryAxis(config, 'i', config.getfloat('max_i_velocity', above=0.))
        self.axis_s = AuxiliaryAxis(config, 's', config.getfloat('max_s_velocity', above=0.))

        self.gcode.register_command('HOME_FILAMENT_SELECTOR', self.home)
        self.gcode.register_command('HOME_FILAMENT_SELECTOR_DEBUG', self.home_debug)

    def home_debug(self, params):
        try:
            self.home(params)
        except Exception as e:
            logging.warning(e)

    def home(self, params):
        axes = []
        if 'AXIS' in params:
            if 'I' in params['AXIS']:
                axes.append(self.axis_i)
            if 'S' in params['AXIS']:
                axes.append(self.axis_s)

        if not axes:
            axes = [self.axis_i, self.axis_s]

        for axis in axes:
            axis.home()


class AuxiliaryAxis:

    def __init__(self, config, axis, max_velocity):
        self.printer = config.get_printer()
        self.axis = axis

        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.cmove = ffi_main.gc(ffi_lib.move_alloc(), ffi_lib.free)
        self.cmove_fill = ffi_lib.move_fill
        self.rail = stepper.LookupMultiRail(config.getsection('stepper_%s' % self.axis))
        self.rail.setup_itersolve('auxiliary_stepper_alloc')
        self.max_velocity = max_velocity
        self.commanded_pos = 0.

        self.printer.register_event_handler("gcode:m18", self.motor_off)

    def get_toolhead(self):
        return self.printer.lookup_object('toolhead')

    def home(self):
        self.rail.motor_enable(self.get_toolhead().get_last_move_time(), 1)
        self.get_toolhead().dwell(.1)

        home_info = self.rail.get_homing_info()
        position_min, position_max = self.rail.get_range()

        force_pos = home_info.position_endstop
        if home_info.positive_dir:
            force_pos -= 1.5 * (home_info.position_endstop - position_min)
        else:
            force_pos += 1.5 * (position_max - home_info.position_endstop)

        home_pos = home_info.position_endstop

        home_speed = min(home_info.speed, self.max_velocity)
        home_speed_2 = min(home_info.second_homing_speed, self.max_velocity)

        self.set_position(force_pos)
        axis_d = home_pos - force_pos
        est_move_d = abs(axis_d)
        est_steps = sum([est_move_d / s.get_step_dist()
                         for s in self.rail.get_steppers()])
        dwell_t = est_steps * homing.HOMING_STEP_DELAY

        self.homing_move(home_pos, home_speed, dwell_t)

        if home_info.retract_dist:
            retract_r = min(1., home_info.retract_dist / est_move_d)
            retract_pos = home_pos - axis_d * retract_r
            self.move(retract_pos, home_speed)

            force_pos2 = home_pos - axis_d * retract_r
            self.set_position(force_pos2)
            self.homing_move(home_pos, home_speed_2, verify_movement=True)

    def homing_move(self, home_pos, home_speed, dwell_t=0., verify_movement=False):

        for es, es_name in self.rail.get_endstops():
            es.home_prepare()

        if dwell_t:
            self.get_toolhead().dwell(dwell_t, check_stall=False)

        print_time = self.get_toolhead().get_last_move_time()
        start_mcu_pos = [(s, s.get_mcu_position)
                         for s in self.rail.get_steppers()]

        for mcu_endstop, name in self.rail.get_endstops():
            min_step_dist = min([s.get_step_dist()
                                 for s in mcu_endstop.get_steppers()])
            mcu_endstop.home_start(
                print_time, homing.ENDSTOP_SAMPLE_TIME,
                homing.ENDSTOP_SAMPLE_COUNT, min_step_dist / home_speed)
        error = None
        try:
            self.move(home_pos, home_speed)
        except homing.EndstopError as e:
            error = "Error during homing move: %s" % (str(e),)

        move_end_print_time = self.get_toolhead().get_last_move_time()
        self.get_toolhead().reset_print_time(print_time)
        for mcu_endstop, name in self.rail.get_endstops():
            try:
                mcu_endstop.home_wait(move_end_print_time)
            except mcu_endstop.TimeoutError as e:
                if error is None:
                    error = "Failed to home %s: %s" % (name, str(e))

        self.set_position(home_pos)

        for mcu_endstop, name in self.rail.get_endstops():
            mcu_endstop.home_finalize()

        if error is not None:
            raise homing.EndstopError(error)

        if verify_movement:
            for s, pos in start_mcu_pos:
                if s.get_mcu_position() == pos:
                    raise homing.EndstopError("Endstop %s still triggered after retract" % (self.axis,))

    def move(self, end_pos, speed):
        start_pos = self.commanded_pos
        move_d = end_pos - start_pos
        if move_d is 0.:
            return
        speed = min(speed, self.max_velocity)
        move_t = abs(move_d / speed)
        # check move? use rail range as limits
        print_time = self.get_toolhead().get_last_move_time()
        logging.info("aux-axis %s move from %s, by %s" % (self.axis, start_pos, move_d))
        self.move_fill(print_time, move_t, start_pos, move_d, speed)
        self.rail.step_itersolve(self.cmove)
        self.get_toolhead().dwell(move_t)

    def set_position(self, position):
        self.commanded_pos = position
        self.rail.set_position([position, 0., 0.])

    def move_fill(self, print_time, move_t, start_pos, dist, speed):
        self.cmove_fill(self.cmove, print_time,
                        0., move_t, 0.,
                        start_pos, 0., 0.,
                        dist, 0., 0.,
                        0., speed, 0)

    def motor_off(self, print_time):
        self.rail.motor_enable(print_time, 0)

    def motor_on(self, print_time):
        self.rail.motor_enable(print_time, 1)


def load_kinematics(filament_toolhead, config):
    return IdlerPulleyFilamentSelector(filament_toolhead, config)
