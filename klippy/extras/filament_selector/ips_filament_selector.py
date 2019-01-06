# Filament selector based on Idler/Pulley/Selector style control
#
# Copyright (C) 2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper, homing


class IdlerPulleyFilamentSelector:
    def __init__(self, filament_toolhead, config):
        self.printer = config.get_printer()
        self.printer.add_object("ips_filament_selector", self)
        self.filament_toolhead = filament_toolhead
        self.gcode = self.printer.lookup_object('gcode')
        # Setup axis rails
        self.rail_i = stepper.LookupMultiRail(config.getsection('stepper_i'))
        self.rail_i.setup_itersolve('linear_stepper_alloc')

        self.rail_s = stepper.LookupMultiRail(config.getsection('stepper_s'))
        # TODO new single radial kinematic type?
        self.rail_s.setup_itersolve('linear_stepper_alloc')

    def home(self, params):
        logging.info("Homing IPS filament selector")


def load_kinematics(filament_toolhead, config):
    return IdlerPulleyFilamentSelector(filament_toolhead, config)
