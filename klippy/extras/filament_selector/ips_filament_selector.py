# Filament selector based on Idler/Pulley/Selector style control. MMU2 kinematics independent of the Prusa PCB
#
# Copyright (C) 2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import chelper
import homing
import logging
import stepper
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
        self.axis_i = AuxiliaryAxis(config, 'i', config.getfloat('max_i_velocity', above=0.))

        self.axis_s = AuxiliaryAxis(config, 's', config.getfloat('max_s_velocity', above=0.), self._check_s_move)
        self.sensor_pin_config = self.pins.lookup_pin(config.get('sensor_pin'), share_type='ips_sensor')
        self.sensor_s = SelectorSensor(config, self.sensor_pin_config)

        self.axis_p = FilamentPulley(config, 'p',
                                     config.getfloat('max_s_velocity', 100, above=0.),
                                     self._check_p_move,
                                     self.sensor_s)
        self.ext_id = config.getint('extruder', minval=0)

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
        self.printer.lookup_object("extruder%i" % self.ext_id)

    def get_toolhead(self):
        return self.printer.lookup_object('toolhead')

    def gcode_command(self, impl):
        def catch_and_respond(params):
            try:
                impl(params)
            except Exception as e:
                logging.warning(e)
                self.gcode.respond_info(str(e))

        return catch_and_respond

    def feed(self, params):
        length = self.gcode.get_float("LENGTH", params)
        dwell_time = self.axis_p.move(self.axis_p.commanded_pos + length, 10)
        self.get_toolhead().dwell(dwell_time)

    def home(self, params):
        axes = self.get_axes_from_gcode_params(params)

        if self.sensor_s.current_status() and self.axis_s in axes:
            axes.remove(self.axis_s)
            self.gcode.respond_info(
                "Selector blocked by filament. "
                "Set position via 'FILAMENT_SELECTOR_HOME_S PATH=?'")

        for axis in axes:
            if axis is self.axis_i:
                axis.needs_homing = False
                home_info = axis.rail.get_homing_info()
                home_pos = home_info.position_endstop
                position_min, position_max = axis.get_range()
                force_pos = home_info.position_endstop
                if home_info.positive_dir:
                    force_pos -= (home_info.position_endstop - position_min)
                else:
                    force_pos += (position_max - home_info.position_endstop)

                axis.set_position(force_pos)
                self.get_toolhead().dwell(axis.move(home_pos, 40))
                self.get_toolhead().wait_moves()
                axis.needs_homing = True

            axis.home()

        for path in self.filament_path:
            self.home_filament({"PATH": path.number})

    def get_axes_from_gcode_params(self, params):
        axes = []
        if 'AXIS' in params:
            if 'I' in params['AXIS']:
                axes.append(self.axis_i)
            if 'S' in params['AXIS']:
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


class FilamentPulley:

    def __init__(self, config, axis, max_velocity, move_check, s_sensor):
        self.printer = config.get_printer()
        self.axis = axis
        self.move_check = move_check
        self.sensor = s_sensor

        ffi_main, ffi_lib = chelper.get_ffi()
        self.cmove = ffi_main.gc(ffi_lib.move_alloc(), ffi_lib.free)
        self.cmove_fill = ffi_lib.move_fill
        self.stepper = stepper.PrinterStepper(config.getsection('stepper_%s' % self.axis))
        self.stepper.setup_itersolve('auxiliary_stepper_alloc')
        self.max_velocity = max_velocity
        self.commanded_pos = 0.

        self.sensor.endstop.add_stepper(self.stepper.mcu_stepper)

        self.printer.register_event_handler(
            "toolhead:motor_off", self.motor_off)

    def get_toolhead(self):
        return self.printer.lookup_object('toolhead')

    def motor_off(self, print_time):
        self.stepper.motor_enable(print_time, 0)

    def motor_on(self, print_time):
        self.stepper.motor_enable(print_time, 1)

    def move(self, end_pos, speed, max_speed=None):
        start_pos = self.commanded_pos
        move_d = end_pos - start_pos
        if move_d is 0.:
            return None

        speed = min(speed, self.max_velocity if max_speed is None else max_speed)

        if self.move_check is not None:
            self.move_check(self, end_pos, speed)

        move_t = abs(move_d / speed)
        print_time = self.get_toolhead().get_last_move_time()
        logging.info("pulley %s move from %s, by %s" % (self.axis, start_pos, move_d))

        self.motor_on(print_time)
        self.move_fill(print_time, move_t, start_pos, move_d, speed)
        self.stepper.step_itersolve(self.cmove)
        self.commanded_pos = end_pos
        return move_t

    def home(self):
        self.motor_on(self.get_toolhead().get_last_move_time())
        self.get_toolhead().dwell(.1)

        max_home_move = 60  # TODO config param
        home_speed = 10  # TODO config param
        mcu_endstop = self.sensor.endstop
        mcu_endstop.home_prepare()

        print_time = self.get_toolhead().get_last_move_time()
        step_dist = self.stepper.get_step_dist()
        mcu_endstop.home_start(
            print_time, homing.ENDSTOP_SAMPLE_TIME,
            homing.ENDSTOP_SAMPLE_COUNT, step_dist / home_speed)

        error = None
        try:
            self.get_toolhead().dwell(self.move(max_home_move, home_speed))
        except homing.EndstopError as e:
            error = "Error during homing move: %s" % (str(e),)

        move_end_print_time = self.get_toolhead().get_last_move_time()
        self.get_toolhead().reset_print_time(print_time)
        logging.info("move_end: %f, print_time: %f" % (move_end_print_time, print_time))
        try:
            mcu_endstop.home_wait(move_end_print_time)
        except mcu_endstop.TimeoutError as e:
            if error is None:
                error = "Failed to home filament: %s" % (str(e))

        self.set_position(0)

        mcu_endstop.home_finalize()

        if error is not None:
            self.motor_off(self.get_toolhead().get_last_move_time())
            raise homing.EndstopError(error)

    def set_position(self, position):
        self.commanded_pos = position
        self.stepper.set_position([position, 0., 0.])

    def move_fill(self, print_time, move_t, start_pos, dist, speed):
        self.cmove_fill(self.cmove, print_time,
                        0., move_t, 0.,
                        start_pos, 0., 0.,
                        dist, 0., 0.,
                        0., speed, 0.)


class AuxiliaryAxis:

    def __init__(self, config, axis, max_velocity, move_check=None):
        self.printer = config.get_printer()
        self.axis = axis
        self.move_check = move_check

        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.cmove = ffi_main.gc(ffi_lib.move_alloc(), ffi_lib.free)
        self.cmove_fill = ffi_lib.move_fill
        self.rail = stepper.LookupMultiRail(config.getsection('stepper_%s' % self.axis))
        self.rail.setup_itersolve('auxiliary_stepper_alloc')
        self.max_velocity = max_velocity
        self.commanded_pos = 0.
        self.needs_homing = True

        self.printer.register_event_handler(
            "toolhead:motor_off", self.motor_off)

    def get_range(self):
        return self.rail.get_range()

    def get_toolhead(self):
        return self.printer.lookup_object('toolhead')

    def home(self):
        self.motor_on(self.get_toolhead().get_last_move_time())
        self.get_toolhead().dwell(.1)

        home_info = self.rail.get_homing_info()
        position_min, position_max = self.get_range()

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
            self.get_toolhead().dwell(self.move(retract_pos, home_speed))

            force_pos2 = home_pos - axis_d * retract_r
            self.set_position(force_pos2)
            self.homing_move(home_pos, home_speed_2, verify_movement=True)

        self.needs_homing = False

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
            self.needs_homing = False
            self.get_toolhead().dwell(self.move(home_pos, home_speed))
        except homing.EndstopError as e:
            self.needs_homing = True
            error = "Error during homing move: %s" % (str(e),)

        move_end_print_time = self.get_toolhead().get_last_move_time()
        self.get_toolhead().reset_print_time(print_time)
        logging.info("move_end: %f, print_time: %f" % (move_end_print_time, print_time))
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
            self.motor_off(self.get_toolhead().get_last_move_time())
            raise homing.EndstopError(error)

        if verify_movement:
            for s, pos in start_mcu_pos:
                if s.get_mcu_position() == pos:
                    self.motor_off(self.get_toolhead().get_last_move_time())
                    raise homing.EndstopError("Endstop %s still triggered after retract" % (self.axis,))

    def move(self, end_pos, speed, max_speed=None):
        start_pos = self.commanded_pos
        move_d = end_pos - start_pos
        if move_d == 0.:
            return None

        speed = min(speed, self.max_velocity if max_speed is None else max_speed)
        self.range_check(end_pos)
        if self.move_check is not None:
            self.move_check(self, end_pos, speed)
        move_t = abs(move_d / speed)
        print_time = self.get_toolhead().get_last_move_time()
        logging.info("aux-axis %s move from %s, by %s" % (self.axis, start_pos, move_d))
        self.move_fill(print_time, move_t, start_pos, move_d, speed)
        self.rail.step_itersolve(self.cmove)
        self.commanded_pos = end_pos
        return move_t

    def set_position(self, position):
        self.commanded_pos = position
        self.rail.set_position([position, 0., 0.])

    def move_fill(self, print_time, move_t, start_pos, dist, speed):
        self.cmove_fill(self.cmove, print_time,
                        0., move_t, 0.,
                        start_pos, 0., 0.,
                        dist, 0., 0.,
                        0., speed, 0.)

    def range_check(self, end_pos):
        if self.needs_homing:
            raise homing.EndstopMoveError(end_pos, "Must home axis '%s' first" % self.axis)
        position_min, position_max = self.rail.get_range()
        if end_pos < position_min or end_pos > position_max:
            raise homing.EndstopMoveError([end_pos, 0., 0., 0.],
                                          "Auxiliary axis '%s': Move out of range" % self.axis)

    def motor_off(self, print_time):
        self.needs_homing = True
        self.rail.motor_enable(print_time, 0)

    def motor_on(self, print_time):
        self.rail.motor_enable(print_time, 1)


def load_kinematics(filament_toolhead, config):
    return IdlerPulleyFilamentSelector(filament_toolhead, config)
