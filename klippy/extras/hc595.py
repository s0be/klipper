# SNx4HC595 Shift Register Support via manual pin toggling
# Copyright (C) 2018  Trevor Jones <trevorjones141@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import mcu, configfile


class HC595(object):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[1]
        self.mcu = mcu.get_printer_mcu(self.printer, config.get('mcu'))
        self._oid = self.mcu.create_oid()
        self._data_pin = config.get('data_pin')
        self._latch_pin = config.get('latch_pin')
        self._clock_pin = config.get('clock_pin')
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.mcu.add_config_cmd("config_hc595 oid=%d data_pin=%s latch_pin=%s clock_pin=%s" % (
            self._oid, self._data_pin, self._latch_pin, self._clock_pin))


def load_config_prefix(config):
    return HC595(config)


class _DefaultShiftRegConfig:
    def __init__(self, printer, name, mcu_name, data_pin, latch_pin, clock_pin):
        self.values = {
            'printer': printer,
            'name': name,
            'mcu': mcu_name,
            'data_pin': data_pin,
            'latch_pin': latch_pin,
            'clock_pin': clock_pin
        }
        for key, val in self.values.iteritems():
            if val is None:
                raise configfile.error("Missing required option '%s' in hc595 default config" % key)

    def get_printer(self):
        return self.get('printer')

    def get_name(self):
        return self.get('name')

    class sentinel:
        pass

    def get(self, option, default=sentinel):
        val = self.values.get(option)
        if val is not None:
            return val
        if default is not self.sentinel:
            return default
        raise configfile.error("Unable to parse option '%s' in hc595 default config" % option)


def default_config(printer, name, mcu_name, data_pin, latch_pin, clock_pin):
    return _DefaultShiftRegConfig(printer, name, mcu_name, data_pin, latch_pin, clock_pin)
