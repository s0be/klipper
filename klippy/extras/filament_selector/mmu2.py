# Prusa MMU2 support.
#
# Copyright (C) 2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, pins, mcu, extras.hc595
import ips_filament_selector


class MMU2:
    FLASH_DELAY = .1

    def __init__(self, filament_toolhead, config):
        self.printer = config.get_printer()
        mcu_name = config.get('mcu', 'mmu2')
        self.mcu = mcu.get_printer_mcu(self.printer, mcu_name)
        self.pins = self.printer.lookup_object("pins")
        self.reactor = self.printer.get_reactor()

        # MMU2 Board needs shift registers in order to work. force it to have extra ports.
        self.shift_reg = extras.hc595.load_config_prefix(extras.hc595.default_config(
            self.printer, 'hc595 mmu2_sr', mcu_name, 'PB5', 'PB6', 'PC7'
        ))
        self.printer.add_object('mmu2_sr', self.shift_reg)
        pins.MCU_PINS["atmega32u4"] = pins.port_pins(8)

        self.ips_impl = ips_filament_selector.load_kinematics(filament_toolhead, config)
        self.printer.add_object("ips_filament_selector", self.ips_impl)

        self.led_g0 = self.pins.setup_pin('digital_out', "%s:PH0" % mcu_name)
        self.led_r0 = self.pins.setup_pin('digital_out', "%s:PH1" % mcu_name)
        self.led_g1 = self.pins.setup_pin('digital_out', "%s:PH2" % mcu_name)
        self.led_r1 = self.pins.setup_pin('digital_out', "%s:PH3" % mcu_name)
        self.led_g2 = self.pins.setup_pin('digital_out', "%s:PH4" % mcu_name)
        self.led_r2 = self.pins.setup_pin('digital_out', "%s:PH5" % mcu_name)
        self.led_g3 = self.pins.setup_pin('digital_out', "%s:PH6" % mcu_name)
        self.led_r3 = self.pins.setup_pin('digital_out', "%s:PH7" % mcu_name)
        self.led_g4 = self.pins.setup_pin('digital_out', "%s:PG6" % mcu_name)
        self.led_r4 = self.pins.setup_pin('digital_out', "%s:PG7" % mcu_name)
        self.leds = [self.led_g0, self.led_r0,
                     self.led_g1, self.led_r1,
                     self.led_g2, self.led_r2,
                     self.led_g3, self.led_r3,
                     self.led_g4, self.led_r4]
        for led in self.leds:
            led.setup_max_duration(0.)

        self.mcu.register_config_callback(self._build_config)
        self.toolhead = None
        self._rs_led = 0

    def _build_config(self):
        self.toolhead = self.printer.lookup_object('toolhead')

    def printer_state(self, state):
        if state == 'ready':
            self.reactor.register_timer(self._ready_led_sequence, self.reactor.NOW)

    def _ready_led_sequence(self, time):
        print_time = self.toolhead.get_last_move_time()
        self.leds[self._rs_led].set_digital(print_time, 1)
        self._rs_led += 1
        if self._rs_led is 10:
            self.reactor.register_timer(self._clear_led, (time + self.FLASH_DELAY * 5))
        return time + self.FLASH_DELAY if self._rs_led < 10 else self.reactor.NEVER

    def _clear_led(self, time):
        if self._rs_led == 10:
            self._rs_led -= 1
        print_time = self.toolhead.get_last_move_time()
        self.leds[self._rs_led].set_digital(print_time, 0)
        self._rs_led -= 1
        return self.reactor.NEVER if self._rs_led == -1 else time + self.FLASH_DELAY


def load_kinematics(filament_toolhead, config):
    return MMU2(filament_toolhead, config)
