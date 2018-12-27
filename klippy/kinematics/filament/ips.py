# Code for handling the kinematics of IPS style filament systems (MMU2)
#
# Copyright (C) 2016-2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper, homing

class IdlerPulleySelector:

    def __init__(self, toolhead, config):
        self.toolhead = toolhead
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.limits = [(1.0, -1.0)] * 3
        # Setup axis rails
        self.rails = [stepper.LookupMultiRail(config.getsection('stepper_' + n))
                      for n in ['i', 'p', 's']]
        for rail, axis in zip(self.rails, 'ips'):
            rail.setup_itersolve('cartesian_stepper_alloc', axis)

    def get_steppers(self, flags=""):
        return [s for rail in self.rails for s in rail.get_steppers()]

    def calc_position(self):
        return [rail.get_commanded_position() for rail in self.rails]

    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()

    def home(self, homing_state):
        for axis in homing_state.get_axes():
            self._home_axis(homing_state, axis, self.rails[axis])

    def _home_axis(self, homing_state, axis, rail):
        # Determine movement
        position_min, position_max = rail.get_range()
        hi = rail.get_homing_info()
        homepos = [None, None, None]
        homepos[axis] = hi.position_endstop
        forcepos = list(homepos)
        if hi.positive_dir:
            forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
        else:
            forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
        # Perform homing
        homing_state.home_rails([rail], forcepos, homepos)

    def motor_off(self, print_time):
        self.limits = [(1.0, -1.0)] * 3
        for rail in self.rails:
            rail.motor_enable(print_time, 0)

    def check_move(self, move):
        return

    def move(self, print_time, move):
        return


def load_kinematics(toolhead, config):
    return IdlerPulleySelector(toolhead, config)
