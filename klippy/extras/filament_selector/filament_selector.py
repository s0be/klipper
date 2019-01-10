# Filament Selector support.
#
# Copyright (C) 2019  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, importlib


class FilamentSelector:

    def __init__(self, config):
        self.printer = config.get_printer()
        kinematic_type = config.get('kinematics')
        try:
            mod = importlib.import_module('extras.filament_selector.' + kinematic_type)
            self.kin = mod.load_kinematics(self, config)
        except config.error as e:
            raise
        except self.printer.lookup_object('pins').error as e:
            raise
        except:
            msg = "Error loading FilamentSelector kinematics '%s'" % (kinematic_type,)
            logging.exception(msg)
            raise config.error(msg)


def load_config(config):
    return FilamentSelector(config)
