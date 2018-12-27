# Code for handling the kinematics of mixed mechanics robots
#
# Copyright (C) 2016-2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import stepper, homing


class CombinedKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        kinematics_config = config.getsection("kinematics")
        toolhead_kinematics = kinematics_config.get('toolhead', None)
        filament_kinematics = kinematics_config.get('filament', None)
        # TODO how to init these

    def get_steppers(self, flags=""):
        logging.info("get_steppers")

    def calc_position(self):
        logging.info("calc_position")

    def set_position(self, newpos, homing_axes):
        logging.info("set_position")

    def home(self, homing_state):
        logging.info("home")

    def motor_off(self, print_time):
        logging.info("motor_off")

    def check_move(self, move):
        logging.info("check_move")

    def move(self, print_time, move):
        logging.info("move")


def load_kinematics(toolhead, config):
    return CombinedKinematics(toolhead, config)
