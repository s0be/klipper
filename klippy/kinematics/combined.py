# Code for handling the kinematics of mixed mechanics robots
#
# Copyright (C) 2016-2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, importlib


class CombinedKinematics:
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        kinematics_config = config.getsection("kinematics")
        self.toolhead_kinematics = self._load_kinematic(kinematics_config.get('toolhead', None), config)
        self.filament_kinematics = self._load_kinematic('filament.' + kinematics_config.get('filament', None), config)

    def _load_kinematic(self, kin_name, config):
        try:
            mod = importlib.import_module('kinematics.' + kin_name)
            return mod.load_kinematics(self.toolhead, config)
        except config.error as e:
            raise
        except self.printer.lookup_object('pins').error as e:
            raise
        except:
            msg = "Error loading kinematics '%s'" % (kin_name,)
            logging.exception(msg)
            raise config.error(msg)

    def get_steppers(self, flags=""):
        logging.info("Combined: get_steppers")
        # th_steppers = self.toolhead_kinematics.get_steppers(flags)
        f_steppers = self.filament_kinematics.get_steppers(flags)
        return f_steppers

    def calc_position(self):
        logging.info("Combined: calc_position")
        # th_posi = self.toolhead_kinematics.calc_position()
        f_posi = self.filament_kinematics.calc_position()
        return f_posi

    def set_position(self, newpos, homing_axes):
        logging.info("Combined: set_position")
        # self.toolhead_kinematics.set_position(newpos, homing_axes)
        self.filament_kinematics.set_position(newpos, homing_axes)

    def home(self, homing_state):
        logging.info("Combined: home")
        # self.toolhead_kinematics.home(homing_state)
        self.filament_kinematics.home(homing_state)

    def motor_off(self, print_time):
        logging.info("Combined: motor_off")
        # self.toolhead_kinematics.motor_off(print_time)
        self.filament_kinematics.motor_off(print_time)

    def check_move(self, move):
        logging.info("Combined: check_move")
        # self.toolhead_kinematics.check_move(move)
        self.filament_kinematics.check_move(move)

    def move(self, print_time, move):
        logging.info("Combined: move")
        # self.toolhead_kinematics.move(print_time, move)
        self.filament_kinematics.move(print_time, move)


def load_kinematics(toolhead, config):
    return CombinedKinematics(toolhead, config)
