# Filament selector based on Idler/Pulley/Selector style control.
# IE: MMU2 kinematics independent of the Prusa PCB.
#
# Copyright (C) 2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import chelper
import homing
import logging
import stepper
import extras.extruder_stepper
from extras.buttons import MCU_buttons


class SelectorSensor:
    def __init__(self, config, pin_params):
        self.printer = config.get_printer()
        self.pin_params = pin_params
        self.status = 0
        mcu = self.pin_params['chip']
        self.mcu_buttons = MCU_buttons(self.printer, mcu)
        self.mcu_buttons.setup_buttons([self.pin_params], self.sensor_change)
        self.endstop = mcu.setup_pin('endstop', self.pin_params)

    def sensor_change(self, event_time, state):
        logging.info("SensorUpdate:%s: %i -> %i" % (self.pin_params['pin'], self.status, state))
        self.status = state
        self.printer.send_event("filament_selector:sensor_change", self, event_time, state)

    def current_status(self):
        return self.status


class FilamentPath:
    def __init__(self, config, number):
        self.printer = config.get_printer()
        self.number = number
        self._status = 0

    def set_status(self, status, broadcast=True):
        previous = self._status
        self._status = status
        if broadcast and (previous != self._status):
            logging.info("FilamentPathStatus:%s: %s -> %s" % (self.number, previous, self._status))
            self.printer.send_event("filament_selector:path_status", self)

    def get_status(self):
        return self._status


class IdlerPulleyFilamentSelector:
    def __init__(self, filament_toolhead, config):
        self.printer = config.get_printer()
        self.pins = self.printer.lookup_object("pins")
        self.toolhead = None
        self.extruder = None

        # Setup axis rails
        self.axis_i = stepper.LookupMultiRail(config.getsection('stepper_i'))
        self.axis_i.setup_itersolve('cartesian_stepper_alloc', 'x')
        self.axis_s = stepper.LookupMultiRail(config.getsection('stepper_s'))
        self.axis_s.setup_itersolve('cartesian_stepper_alloc', 'x')

        self.sensor_pin_config = self.pins.lookup_pin(config.get('sensor_pin'), share_type='ips_sensor')
        self.sensor_s = SelectorSensor(config, self.sensor_pin_config)
        self.axis_p = PulleyStepper(config.getsection('stepper_p'))

        # Calculate move positions
        self.size = config.getint('size', minval=2)
        self.interval_i = config.getfloat('interval_i')
        self.interval_s = config.getfloat('interval_s')
        self.offset_i = [config.getfloat('offset_i_%i' % path, 0.) for path in range(self.size)]
        self.offset_s = [config.getfloat('offset_s_%i' % path, 0.) for path in range(self.size)]
        self.path_i = [path * self.interval_i + self.offset_i[path] for path in range(self.size)]
        self.path_s = [path * self.interval_s + self.offset_s[path] for path in range(self.size)]
        self.check_path_positions(config, self.axis_i, self.path_i)
        self.check_path_positions(config, self.axis_s, self.path_s)
        self.filament_path = [FilamentPath(config, path) for path in range(self.size)]

        # Register Additional Gcode commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('FILAMENT_SELECTOR_HOME', self.gcode_command(self.home))
        self.gcode.register_command('FILAMENT_SELECTOR_HOME_S', self.gcode_command(self.home_s))
        self.gcode.register_command('FILAMENT_SELECTOR_MOVE', self.gcode_command(self.move))
        self.gcode.register_command('FILAMENT_SELECTOR_FEED', self.gcode_command(self.feed))
        self.gcode.register_command('FILAMENT_SELECTOR_CHECK_PATH', self.gcode_command(self.home_filament))

        self.printer.register_event_handler("klippy:ready", self.handle_ready)

    def handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        # self.extruder = self.printer.lookup_object("extruder")

    def get_toolhead(self):
        return self.printer.lookup_object('toolhead')

    def gcode_command(self, impl):
        def catch_and_respond(command):
            try:
                impl(command)
            except Exception as e:
                logging.warning(e)
                self.gcode.respond_info(str(e))

        return catch_and_respond

    def feed(self, command):
        length = command.get_float("LENGTH")
        # dwell_time = self.axis_p.move(self.axis_p.commanded_pos + length, 10)
        # self.get_toolhead().dwell(dwell_time)

    def home(self, command):
        axes = self.get_axes_from_gcode_params(command)

        if self.sensor_s.current_status() and self.axis_s in axes:
            axes.remove(self.axis_s)
            self.gcode.respond_info(
                "Selector blocked by filament. "
                "Set position via 'FILAMENT_SELECTOR_HOME_S PATH=?'")

        for axis in axes:
            axis.home()

        for path in self.filament_path:
            self.home_filament({"PATH": path.number})

    def get_axes_from_gcode_params(self, command):
        axes = []
        axi_arg = command.get("AXIS", default=None)
        if axi_arg is not None:
            if 'I' in axi_arg:
                axes.append(self.axis_i)
            if 'S' in axi_arg:
                axes.append(self.axis_s)
        if not axes:
            axes = [self.axis_i, self.axis_s]
        return axes

    def home_s(self, params):
        path = self.gcode.get_int("PATH", params, minval=0, maxval=self.size - 1)
        self.axis_s.motor_on(self.toolhead.get_last_move_time())
        self.axis_s.set_position(self.path_s[path])

    def _check_s_move(self, aux_axis, end_pos, speed):
        if aux_axis.commanded_pos != end_pos and self.sensor_s.current_status():
            raise homing.EndstopMoveError([end_pos, 0., 0., 0.],
                                          "Selector blocked by filament.")

    def _do_homing_move(self, rail, movepos, speed, accel, triggered, check_trigger):
        # Notify start of homing/probing move
        endstops = rail.get_endstops()
        self.printer.send_event("homing:homing_move_begin",
                                [es for es, name in endstops])
        # Start endstop checking
        self.sync_print_time()
        endstops = self.rail.get_endstops()
        for mcu_endstop, name in endstops:
            min_step_dist = min([s.get_step_dist()
                                 for s in mcu_endstop.get_steppers()])
            mcu_endstop.home_start(
                self.next_cmd_time, ENDSTOP_SAMPLE_TIME, ENDSTOP_SAMPLE_COUNT,
                min_step_dist / speed, triggered=triggered)
        # Issue move
        self.do_move(movepos, speed, accel)
        # Wait for endstops to trigger
        error = None
        for mcu_endstop, name in endstops:
            did_trigger = mcu_endstop.home_wait(self.next_cmd_time)
            if not did_trigger and check_trigger and error is None:
                error = "Failed to home %s: Timeout during homing" % (name,)
        # Signal homing/probing move complete
        try:
            self.printer.send_event("homing:homing_move_end",
                                    [es for es, name in endstops])
        except CommandError as e:
            if error is None:
                error = str(e)
        self.sync_print_time()
        if error is not None:
            raise homing.CommandError(error)

    def _check_p_move(self, aux_axis, end_pos, speed):
        move_d = end_pos - aux_axis.commanded_pos
        # make sure the filament won't get removed from the path
        # if move_d < 0. and not self.sensor_s.current_status():
        # raise homing.EndstopMoveError([end_pos, 0., 0., 0.])

    def check_path_positions(self, config, axis, paths):
        position_min, position_max = axis.get_range()
        for index, position in enumerate(paths):
            if position < position_min or position > position_max:
                raise config.error("%s axis position out of range. path:%i, position:%f, min:%f, max:%f"
                                   % (axis.axis, index, position, position_min, position_max))

    def home_filament(self, params):
        path = self.gcode.get_int("PATH", params, -1, minval=0, maxval=self.size - 1)
        self.move({"PATH": path})
        filament_path = self.filament_path[path]
        filament_path.set_status(0, broadcast=False)

        self.axis_p.home()
        #todo catch the fail
        #todo if it fails, retract the full home length so selector space is for sure clear
        self.get_toolhead().wait_moves()
        if self.sensor_s.current_status():
            filament_path.set_status(1)
            # Retract back out of selector
            self.feed({"LENGTH": -25.0})  # todo homing retract from config
            self.get_toolhead().wait_moves()

        if not filament_path.get_status():
            self.gcode.respond_info("No filament in path %s" % path)

    def move(self, params):
        axes = self.get_axes_from_gcode_params(params)
        path = self.gcode.get_int("PATH", params, minval=0, maxval=self.size - 1)
        dwell_time = None

        if self.axis_i in axes:
            move_i = self.axis_i.move(self.path_i[path], self.axis_i.max_velocity)
            if move_i is not None:
                dwell_time = max(move_i, dwell_time)

        if self.axis_s in axes:
            move_s = self.axis_s.move(self.path_s[path], self.axis_s.max_velocity)
            if move_s is not None:
                dwell_time = max(move_s, dwell_time)

        if dwell_time is not None:
            self.get_toolhead().dwell(dwell_time)


ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4

class PulleyStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        if config.get('endstop_pin', None) is not None:
            self.can_home = True
            self.rail = stepper.PrinterRail(
                config, need_position_minmax=False, default_position_endstop=0.)
            self.steppers = self.rail.get_steppers()
        else:
            self.can_home = False
            self.rail = stepper.PrinterStepper(config)
            self.steppers = [self.rail]
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = config.getfloat('accel', 0., minval=0.)
        self.next_cmd_time = 0.
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_free_moves = ffi_lib.trapq_free_moves
        self.rail.setup_itersolve('cartesian_stepper_alloc', 'x')
        self.rail.set_trapq(self.trapq)
        self.rail.set_max_jerk(9999999.9, 9999999.9)
    def sync_print_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time
    def do_enable(self, enable):
        self.sync_print_time()
        stepper_enable = self.printer.lookup_object('stepper_enable')
        if enable:
            for s in self.steppers:
                se = stepper_enable.lookup_enable(s.get_name())
                se.motor_enable(self.next_cmd_time)
        else:
            for s in self.steppers:
                se = stepper_enable.lookup_enable(s.get_name())
                se.motor_disable(self.next_cmd_time)
        self.sync_print_time()
    def do_set_position(self, setpos):
        self.rail.set_position([setpos, 0., 0.])
    def do_move(self, movepos, speed, accel, sync=True):
        self.sync_print_time()
        cp = self.rail.get_commanded_position()
        dist = movepos - cp
        axis_r, accel_t, cruise_t, cruise_v = force_move.calc_move_time(
            dist, speed, accel)
        self.trapq_append(self.trapq, self.next_cmd_time,
                          accel_t, cruise_t, accel_t,
                          cp, 0., 0., axis_r, 0., 0.,
                          0., cruise_v, accel)
        self.next_cmd_time = self.next_cmd_time + accel_t + cruise_t + accel_t
        self.rail.generate_steps(self.next_cmd_time)
        self.trapq_free_moves(self.trapq, self.next_cmd_time + 99999.9)
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.note_kinematic_activity(self.next_cmd_time)
        if sync:
            self.sync_print_time()
    def do_homing_move(self, movepos, speed, accel, triggered, check_trigger):
        if not self.can_home:
            raise self.printer.command_error(
                "No endstop for this manual stepper")
        # Notify start of homing/probing move
        endstops = self.rail.get_endstops()
        self.printer.send_event("homing:homing_move_begin",
                                [es for es, name in endstops])
        # Start endstop checking
        self.sync_print_time()
        endstops = self.rail.get_endstops()
        for mcu_endstop, name in endstops:
            min_step_dist = min([s.get_step_dist()
                                 for s in mcu_endstop.get_steppers()])
            mcu_endstop.home_start(
                self.next_cmd_time, ENDSTOP_SAMPLE_TIME, ENDSTOP_SAMPLE_COUNT,
                min_step_dist / speed, triggered=triggered)
        # Issue move
        self.do_move(movepos, speed, accel)
        # Wait for endstops to trigger
        error = None
        for mcu_endstop, name in endstops:
            did_trigger = mcu_endstop.home_wait(self.next_cmd_time)
            if not did_trigger and check_trigger and error is None:
                error = "Failed to home %s: Timeout during homing" % (name,)
        # Signal homing/probing move complete
        try:
            self.printer.send_event("homing:homing_move_end",
                                    [es for es, name in endstops])
        except CommandError as e:
            if error is None:
                error = str(e)
        self.sync_print_time()
        if error is not None:
            raise homing.CommandError(error)


def load_kinematics(filament_toolhead, config):
    return IdlerPulleyFilamentSelector(filament_toolhead, config)
