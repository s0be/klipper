# Pulley stepper support.
# A pulley is like an extruder stepper without a heater/temp probe.
#
# Copyright (C) 2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import stepper
import chelper


class PulleyStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.stepper = stepper.PrinterStepper(config)

        ffi_main, ffi_lib = chelper.get_ffi()
        self.cmove = ffi_main.gc(ffi_lib.move_alloc(), ffi_lib.free)
        self.extruder_move_fill = ffi_lib.extruder_move_fill
        self.stepper.setup_itersolve('extruder_stepper_alloc')

