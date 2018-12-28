# Code for handling the kinematics of IPS style filament systems (MMU2)
#
# Copyright (C) 2016-2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper, homing, toolhead

STALL_TIME = 0.100


class EndstopError(Exception):
    pass


class IdlerPulleySelector:

    def __init__(self, toolhead, config):
        self.toolhead = toolhead
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        # Setup axis rails
        self.i_rail = stepper.LookupMultiRail(config.getsection('stepper_i'))
        self.s_rail = stepper.LookupMultiRail(config.getsection('stepper_s'))
        self.rails = {'I': self.i_rail, 'S': self.s_rail}
        for axis, rail in self.rails.iteritems():
            rail.setup_itersolve('cartesian_stepper_alloc', axis)

    def home(self):
        self.home_rail(self.s_rail)

    def home_rail(self, rail):
        home_info = rail.get_homing_info()
        # Perform homing
        position_min, position_max = rail.get_range()
        homepos = [home_info.position_endstop, 0, 0, 0]
        forcepos = list(homepos)
        if home_info.positive_dir:
            forcepos[0] -= 1.5 * (home_info.position_endstop - position_min)
        else:
            forcepos[0] += 1.5 * (position_max - home_info.position_endstop)

        axes_d = [mp - fp for mp, fp in zip(homepos, forcepos)]
        est_move_d = abs(axes_d[0]) + abs(axes_d[1]) + abs(axes_d[2])
        est_steps = sum([est_move_d / s.get_step_dist()
                         for es, n in rail.get_endstops() for s in es.get_steppers()])
        dwell_t = est_steps * homing.HOMING_STEP_DELAY

        self.homing_move(forcepos,
                         homepos,
                         rail.get_endstops(),
                         self._get_homing_speed(home_info.speed, rail.endstops),
                         dwell_t,
                         True)

    def homing_move(self, forcepos, movepos, endstops, speed, dwell_t=0.,verify_movement=False):
        # Notify endstops of upcoming home
        for mcu_endstop, name in endstops:
            mcu_endstop.home_prepare()
        if dwell_t:
            self.toolhead.dwell(dwell_t, check_stall=False)
        # Start endstop checking
        print_time = self.toolhead.get_last_move_time()
        start_mcu_pos = [(s, name, s.get_mcu_position())
                         for es, name in endstops for s in es.get_steppers()]

        for mcu_endstop, name in endstops:
            min_step_dist = min([s.get_step_dist()
                                 for s in mcu_endstop.get_steppers()])
            mcu_endstop.home_start(
                print_time, homing.ENDSTOP_SAMPLE_TIME, homing.ENDSTOP_SAMPLE_COUNT,
                min_step_dist / speed)
        # Issue move
        error = None
        try:
            speed = min(speed, self.toolhead.max_velocity)
            move = toolhead.Move(self.toolhead, forcepos, movepos, speed)
            self.toolhead.move_queue.add_move(move)
        except EndstopError as e:
            error = "Error during homing move: %s" % (str(e),)
        # Wait for endstops to trigger
        move_end_print_time = self.toolhead.get_last_move_time()
        self.toolhead.reset_print_time(print_time)
        for mcu_endstop, name in endstops:
            try:
                mcu_endstop.home_wait(move_end_print_time)
            except mcu_endstop.TimeoutError as e:
                if error is None:
                    error = "Failed to home %s: %s" % (name, str(e))

        # self.toolhead.set_position(movepos) TODO set the state here in the ips class
        for mcu_endstop, name in endstops:
            mcu_endstop.home_finalize()
        if error is not None:
            raise EndstopError(error)
        if verify_movement:
            for s, name, pos in start_mcu_pos:
                if s.get_mcu_position() == pos:
                    raise EndstopError("Endstop %s still triggered after retract" % (name,))

    def _get_homing_speed(self, speed, endstops):
        # Round the requested homing speed so that it is an even
        # number of ticks per step.
        mcu_stepper = endstops[0][0].get_steppers()[0]
        adjusted_freq = mcu_stepper.get_mcu().get_adjusted_freq()
        dist_ticks = adjusted_freq * mcu_stepper.get_step_dist()
        ticks_per_step = math.ceil(dist_ticks / speed)
        return dist_ticks / ticks_per_step

    def motor_off(self, print_time):
        for axis, rail in self.rails.iteritems():
            rail.motor_enable(print_time, 0)

    def move(self, print_time, move):
        self.s_rail.motor_enable(print_time, 1)
        self.s_rail.step_itersolve(move.cmove)


def load_kinematics(toolhead, config):
    return IdlerPulleySelector(toolhead, config)
