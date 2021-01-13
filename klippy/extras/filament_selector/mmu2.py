# Prusa MMU2 support.
#
# Copyright (C) 2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, pins, mcu, extras.hc595
import ips_filament_selector


class MMU2:
    FLASH_DELAY = .1

    sr_pins = {"stepper_1_dir": 'PG0', "stepper_1_enable": 'PG1',
               "stepper_2_dir": 'PG2', "stepper_2_enable": 'PG3',
               "stepper_3_dir": 'PG4', "stepper_3_enable": 'PG5'}

    def __init__(self, filament_toolhead, config):
        self._printer = config.get_printer()
        mcu_name = config.get('mcu', 'mmu2')
        self._mcu = mcu.get_printer_mcu(self._printer, mcu_name)
        self._reactor = self._printer.get_reactor()

        self._shift_reg = extras.hc595.load_config_prefix(extras.hc595.default_config(
            self._printer, 'hc595 mmu2_sr', mcu_name, 'PB5', 'PB6', 'PC7'
        ))
        self._printer.add_object('mmu2_sr', self._shift_reg)

        self._ips_impl = ips_filament_selector.load_kinematics(filament_toolhead, config)
        self._printer.add_object("ips_filament_selector", self._ips_impl)

        self._leds = []
        self._leds_red = []
        self._leds_green = []

        pins = self._printer.lookup_object("pins")
        self._led_r0 = pins.setup_pin('digital_out', "%s:PG7" % mcu_name)
        self._leds.append(self._led_r0)
        self._leds_red.append(self._led_r0)

        self._led_g0 = pins.setup_pin('digital_out', "%s:PG6" % mcu_name)
        self._leds.append(self._led_g0)
        self._leds_green.append(self._led_g0)

        self._led_r1 = pins.setup_pin('digital_out', "%s:PH7" % mcu_name)
        self._leds.append(self._led_r1)
        self._leds_red.append(self._led_r1)

        self._led_g1 = pins.setup_pin('digital_out', "%s:PH6" % mcu_name)
        self._leds.append(self._led_g1)
        self._leds_green.append(self._led_g1)

        self._led_r2 = pins.setup_pin('digital_out', "%s:PH5" % mcu_name)
        self._leds.append(self._led_r2)
        self._leds_red.append(self._led_r2)

        self._led_g2 = pins.setup_pin('digital_out', "%s:PH4" % mcu_name)
        self._leds.append(self._led_g2)
        self._leds_green.append(self._led_g2)

        self._led_r3 = pins.setup_pin('digital_out', "%s:PH3" % mcu_name)
        self._leds.append(self._led_r3)
        self._leds_red.append(self._led_r3)

        self._led_g3 = pins.setup_pin('digital_out', "%s:PH2" % mcu_name)
        self._leds.append(self._led_g3)
        self._leds_green.append(self._led_g3)

        self._led_r4 = pins.setup_pin('digital_out', "%s:PH1" % mcu_name)
        self._leds.append(self._led_r4)
        self._leds_red.append(self._led_r4)

        self._led_g4 = pins.setup_pin('digital_out', "%s:PH0" % mcu_name)
        self._leds.append(self._led_g4)
        self._leds_green.append(self._led_g4)

        for led in self._leds:
            led.setup_max_duration(0.)

        self._toolhead = None
        self._rs_led = 0

        self._printer.register_event_handler("klippy:ready", self.handle_ready)
        self._printer.register_event_handler("filament_selector:path_status", self.handle_path_status)

    def handle_ready(self):
        self._toolhead = self._printer.lookup_object('toolhead')
        self._reactor.register_timer(self._ready_led_sequence, self._reactor.NOW)

    def handle_path_status(self, path):
        logging.info("path update: %s" % path)
        number = path.number
        print_time = self._toolhead.get_last_move_time()
        self._leds_green[number].set_digital(print_time, path.get_status())
        self._toolhead.wait_moves()

    def _ready_led_sequence(self, time):
        print_time = self._toolhead.get_last_move_time()
        self._leds[self._rs_led].set_digital(print_time, 1)
        self._rs_led += 1
        if self._rs_led is 10:
            self._reactor.register_timer(self._clear_led, (time + self.FLASH_DELAY * 5))
        return time + self.FLASH_DELAY if self._rs_led < 10 else self._reactor.NEVER

    def _clear_led(self, time):
        if self._rs_led == 10:
            self._rs_led -= 1

        print_time = self._toolhead.get_last_move_time()
        self._leds[self._rs_led].set_digital(print_time, 0)

        self._rs_led -= 1

        if self._rs_led == -1:
            self._toolhead.wait_moves()
            return self._reactor.NEVER
        else:
            return time + self.FLASH_DELAY


def load_kinematics(filament_toolhead, config):
    return MMU2(filament_toolhead, config)
