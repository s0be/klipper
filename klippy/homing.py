# Code for state tracking during homing operations
#
# Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math

HOMING_STEP_DELAY = 0.00000025
HOMING_START_DELAY = 0.001
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4


class Homing:
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object('toolhead')
        self.changed_axes = []
        self.verify_retract = True

    def set_no_verify_retract(self):
        self.verify_retract = False

    def set_axes(self, axes):
        self.changed_axes = axes

    def get_axes(self):
        return self.changed_axes

    def _get_homing_speed(self, speed, endstops):
        # Round the requested homing speed so that it is an even
        # number of ticks per step.
        steppers = [stepper for endstop in endstops for stepper in endstop.get_steppers()]
        adjusted_freq = {stepper: stepper.get_mcu().get_adjusted_freq() for stepper in steppers}
        dist_ticks = min([adjusted_freq * stepper.get_step_dist()
                          for stepper, adjusted_freq in adjusted_freq.viewitems()])
        ticks_per_step = math.ceil(dist_ticks / speed)
        return dist_ticks / ticks_per_step

    def home_axes(self, axes):
        self.changed_axes = axes
        try:
            for kinematic in set([axis.kinematics for axis in axes]):
                kinematic.home(self)
        except EndstopError:
            self.toolhead.motor_off()
            raise

    def home_rails(self, rails, axi_start_end, axi_retract):
        # axi_start_end is a dictionary of: {axis: (forcepos: float, homepos: float)}
        # axi_retract is a dictionary of: {axis: float}

        # Alter axes to think they are at forcepos
        axi_force_move_dist = {axis: (forcepos, movepos, forcepos - movepos)
                               for axis, forcepos, movepos in axi_start_end.viewitems()}
        for axis, forcepos, homepos in axi_start_end.viewitems():
            axis.is_homing = True
            axis.set_position(forcepos)

        # Determine homing speed
        endstops = [es for rail in rails for es in rail.get_endstops()]
        max_velocity = min([axis.get_max_velocity() for axis in axi_start_end.viewkeys()])

        h1_min_speed = min([rail.get_homing_info().speed for rail in rails])
        h1_speed = self._get_homing_speed(min(h1_min_speed, max_velocity), endstops)

        # Calculate a CPU delay when homing a large axis
        axes_d = {axi: dist for axi, force, move, dist in axi_force_move_dist.viewitems()}
        est_move_d = sum([abs(dist) for axi, dist in axes_d.viewitems()])
        est_steps = sum([est_move_d / stepper.get_step_dist()
                         for endstop in endstops for stepper in endstop.get_steppers()])
        dwell_t = est_steps * HOMING_STEP_DELAY

        # Perform first home
        home_axi_moves = {axi: homepos for axi, forcepos, homepos, distance in axi_force_move_dist.viewitems()}
        self.homing_move(home_axi_moves, endstops, h1_speed, dwell_t=dwell_t)

        # Perform second home
        if any(distance for axi, distance in axi_retract):
            # Retract
            r_axi_moves = {axi: home_axi_moves[axi] + retract_d for axi, retract_d in axi_retract}
            self.toolhead.move(r_axi_moves, h1_speed)

            # Home again
            h2_min_speed = min([rail.get_homing_info().second_homing_speed for rail in rails])
            h2_speed = min(h2_min_speed, max_velocity)
            self.homing_move(home_axi_moves, endstops, h2_speed,
                             verify_movement=self.verify_retract)

        # Signal home operation complete
        self.printer.send_event("homing:homed_rails", self, rails)

    def homing_move(self, moves, endstops, speed, dwell_t=0.,
                    probe_pos=False, verify_movement=False):
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
                print_time, ENDSTOP_SAMPLE_TIME, ENDSTOP_SAMPLE_COUNT,
                min_step_dist / speed)
        self.toolhead.dwell(HOMING_START_DELAY, check_stall=False)
        # Issue move
        error = None
        try:
            self.toolhead.move(moves, speed)
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

        # self.toolhead._flush_lookahead() TODO do we need this?
        for mcu_endstop, name in endstops:
            mcu_endstop.home_finalize()
        if error is not None:
            raise EndstopError(error)

        # Check if some movement occurred
        if verify_movement:
            for s, name, pos in start_mcu_pos:
                if s.get_mcu_position() == pos:
                    if probe_pos:
                        raise EndstopError("Probe triggered prior to movement")
                    raise EndstopError(
                        "Endstop %s still triggered after retract" % (name,))


class EndstopError(Exception):
    pass


def EndstopMoveError(moves, msg="Move out of range"):
    return EndstopError("%s: %s" % (
        msg, ' '.join(["%s[%.3f]" % (axi, pos) for axi, pos in moves.viewitems()])))
