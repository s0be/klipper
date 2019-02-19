# TMC2130 configuration
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import bus

TMC_FREQUENCY=13200000.
GCONF_EN_PWM_MODE=1<<2
GCONF_DIAG1_STALL=1<<8

Registers = {
    "GCONF": 0x00, "GSTAT": 0x01, "IOIN": 0x04, "IHOLD_IRUN": 0x10,
    "TPOWERDOWN": 0x11, "TSTEP": 0x12, "TPWMTHRS": 0x13, "TCOOLTHRS": 0x14,
    "THIGH": 0x15, "XDIRECT": 0x2d, "MSLUT0": 0x60, "MSLUTSEL": 0x68,
    "MSLUTSTART": 0x69, "MSCNT": 0x6a, "MSCURACT": 0x6b, "CHOPCONF": 0x6c,
    "COOLCONF": 0x6d, "DCCTRL": 0x6e, "DRV_STATUS": 0x6f, "PWMCONF": 0x70,
    "PWM_SCALE": 0x71, "ENCM_CTRL": 0x72, "LOST_STEPS": 0x73,
}

ReadRegisters = {
    "GCONF": ["I_scale_analog", "internal_Rsense", "en_pwm_mode", "enc_commutation",
              "shaft", "diag0_error", "diag0_otpw", "diag0_stall",
              "diag1_stall", "diag1_index", "diag1_steps_skipped", "diag0_int_pushpull",
              "diag0_int_pushpull", "small_hysteresis", "stop_enable", "direct_mode",
              "test_mode"],
    "GSTAT": ["reset", "drv_err", "uv_cp"],
    "IOIN": ["STEP", "DIR", "DCEN_CFG4", "DCEN_CFG5", "DRV_ENN_CFG6", "DCO"],
    "TSTEP": ["TSTEP:20"],
    "XDIRECT": [],
    "MSCNT": ["MSCNT:10"],
    "MSCURACT": ["CUR_A:9", "unused:7", "CUR_B:9"],
    "CHOPCONF": ["toff0", "toff1", "toff2", "toff3",
                 "hstrt0", "hstrt1", "hstrt2",
                 "hend0", "hend1", "hend2", "hend3",
                 "fd3", "disfdcc", "rndtf", "chm",
                 "tbl0", "tbl1",
                 "vsense", "vhighfs", "vhighchm",
                 "sync0", "sync1", "sync2", "sync3",
                 "mres0", "mres1", "mres2", "mres3",
                 "intpol", "dedge", "diss2g"],
    "DRV_STATUS": ["SG_RESULT:10", "reserved:5", "fsactive", "CS_ACTUAL:5",
                   "reserved:3", "stallGuard", "ot", "otpw",
                   "s2ga", "s2gb", "ola", "olb", "stst"],
    "PWM_SCALE": ["PWM_SCALE:8"],
    "LOST_STEPS": ["LOST_STEPS:20"]
}

class TMC2130:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[1]
        self.spi = bus.MCU_SPI_from_config(config, 3, default_speed=4000000)
        # Allow virtual endstop to be created
        self.diag1_pin = config.get('diag1_pin', None)
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("tmc2130_" + self.name, self)
        # Add DUMP_TMC command
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "DUMP_TMC", "STEPPER", self.name,
            self.cmd_DUMP_TMC, desc=self.cmd_DUMP_TMC_help)
        # Get config for initial driver settings
        run_current = config.getfloat('run_current', above=0., maxval=2.)
        hold_current = config.getfloat('hold_current', run_current,
                                       above=0., maxval=2.)
        sense_resistor = config.getfloat('sense_resistor', 0.110, above=0.)
        steps = {'256': 0, '128': 1, '64': 2, '32': 3, '16': 4,
                 '8': 5, '4': 6, '2': 7, '1': 8}
        self.mres = config.getchoice('microsteps', steps)
        interpolate = config.getboolean('interpolate', True)
        sc_velocity = config.getfloat('stealthchop_threshold', 0., minval=0.)
        sc_threshold = self.velocity_to_clock(config, sc_velocity)
        iholddelay = config.getint('driver_IHOLDDELAY', 8, minval=0, maxval=15)
        tpowerdown = config.getint('driver_TPOWERDOWN', 0, minval=0, maxval=255)
        blank_time_select = config.getint('driver_BLANK_TIME_SELECT', 1,
                                          minval=0, maxval=3)
        toff = config.getint('driver_TOFF', 4, minval=1, maxval=15)
        hend = config.getint('driver_HEND', 7, minval=0, maxval=15)
        hstrt = config.getint('driver_HSTRT', 0, minval=0, maxval=7)
        sfilt = 0 if config.getboolean('driver_SFILT', False) else 1
        sgt = config.getint('driver_SGT', 0, minval=-64, maxval=63) & 0x7f
        pwm_scale = config.getboolean('driver_PWM_AUTOSCALE', True)
        pwm_freq = config.getint('driver_PWM_FREQ', 1, minval=0, maxval=3)
        pwm_grad = config.getint('driver_PWM_GRAD', 4, minval=0, maxval=255)
        pwm_ampl = config.getint('driver_PWM_AMPL', 128, minval=0, maxval=255)
        # calculate current
        vsense = False
        irun = self.current_bits(run_current, sense_resistor, vsense)
        ihold = self.current_bits(hold_current, sense_resistor, vsense)
        if irun < 16 and ihold < 16:
            vsense = True
            irun = self.current_bits(run_current, sense_resistor, vsense)
            ihold = self.current_bits(hold_current, sense_resistor, vsense)
        # Configure registers
        self.reg_GCONF = (sc_velocity > 0.) << 2
        self.set_register("GCONF", self.reg_GCONF)
        self.set_register("CHOPCONF", (
            toff | (hstrt << 4) | (hend << 7) | (blank_time_select << 15)
            | (vsense << 17) | (self.mres << 24) | (interpolate << 28)))
        self.set_register("IHOLD_IRUN",
                          ihold | (irun << 8) | (iholddelay << 16))
        self.set_register("TPOWERDOWN", tpowerdown)
        self.set_register("TPWMTHRS", max(0, min(0xfffff, sc_threshold)))
        self.set_register("COOLCONF", (sfilt << 24 | sgt << 16))
        self.set_register("PWMCONF", (
            pwm_ampl | (pwm_grad << 8) | (pwm_freq << 16) | (pwm_scale << 18)))
    def current_bits(self, current, sense_resistor, vsense_on):
        sense_resistor += 0.020
        vsense = 0.32
        if vsense_on:
            vsense = 0.18
        cs = int(32. * current * sense_resistor * math.sqrt(2.) / vsense
                 - 1. + .5)
        return max(0, min(31, cs))
    def velocity_to_clock(self, config, velocity):
        if not velocity:
            return 0
        stepper_name = config.get_name().split()[1]
        stepper_config = config.getsection(stepper_name)
        step_dist = stepper_config.getfloat('step_distance')
        step_dist_256 = step_dist / (1 << self.mres)
        return int(TMC_FREQUENCY * step_dist_256 / velocity + .5)
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise pins.error("tmc2130 virtual endstop only useful as endstop")
        if pin_params['invert'] or pin_params['pullup']:
            raise pins.error("Can not pullup/invert tmc2130 virtual endstop")
        return TMC2130VirtualEndstop(self)
    def get_register(self, reg_name):
        reg = Registers[reg_name]
        self.spi.spi_send([reg, 0x00, 0x00, 0x00, 0x00])
        params = self.spi.spi_transfer([reg, 0x00, 0x00, 0x00, 0x00])
        pr = bytearray(params['response'])
        return (pr[1] << 24) | (pr[2] << 16) | (pr[3] << 8) | pr[4]
    def set_register(self, reg_name, val):
        reg = Registers[reg_name]
        data = [(reg | 0x80) & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff,
                (val >> 8) & 0xff, val & 0xff]
        self.spi.spi_send(data)
    def get_microsteps(self):
        return 256 >> self.mres
    def get_phase(self):
        return (self.get_register("MSCNT") & 0x3ff) >> self.mres
    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"
    def cmd_DUMP_TMC(self, params):
        self.printer.lookup_object('toolhead').get_last_move_time()
        gcode = self.printer.lookup_object('gcode')
        requested_reg = gcode.get_str('REGISTER', params, None)
        logging.info("DUMP_TMC %s", self.name)

        def _respond_raw(label, value):
            gcode.respond_info("%-15s %08x        %s" % (label + ":", value, bin(value)))

        def _respond_labeled(label, value):
            gcode.respond_info("%-15s %-15s %i" % (label + ":", value, int(value, 2)))

        if requested_reg in ReadRegisters:
            sub_names = ReadRegisters[requested_reg]
            val = self.get_register(requested_reg)
            _respond_raw(requested_reg, val)
            offset = 0

            for sub_name in sub_names:
                width = int(sub_name.split(":", 1)[1] if ":" in sub_name else 1)
                sub_val = bin(val)[2:][::-1][offset:offset + width]
                offset += width
                _respond_labeled(sub_name, sub_val)

        else:
            for reg_name in ReadRegisters:
                val = self.get_register(reg_name)
                msg = "%-15s %08x" % (reg_name + ":", val)
                logging.info(msg)
                gcode.respond_info(msg)

# Endstop wrapper that enables tmc2130 "sensorless homing"
class TMC2130VirtualEndstop:
    def __init__(self, tmc2130):
        self.tmc2130 = tmc2130
        if tmc2130.diag1_pin is None:
            raise pins.error("tmc2130 virtual endstop requires diag1_pin")
        ppins = tmc2130.printer.lookup_object('pins')
        self.mcu_endstop = ppins.setup_pin('endstop', tmc2130.diag1_pin)
        if self.mcu_endstop.get_mcu() is not tmc2130.spi.get_mcu():
            raise pins.error("tmc2130 virtual endstop must be on same mcu")
        # Wrappers
        self.get_mcu = self.mcu_endstop.get_mcu
        self.add_stepper = self.mcu_endstop.add_stepper
        self.get_steppers = self.mcu_endstop.get_steppers
        self.home_start = self.mcu_endstop.home_start
        self.home_wait = self.mcu_endstop.home_wait
        self.query_endstop = self.mcu_endstop.query_endstop
        self.query_endstop_wait = self.mcu_endstop.query_endstop_wait
        self.TimeoutError = self.mcu_endstop.TimeoutError
    def home_prepare(self):
        gconf = self.tmc2130.reg_GCONF
        gconf &= ~GCONF_EN_PWM_MODE
        gconf |= GCONF_DIAG1_STALL
        self.tmc2130.set_register("GCONF", gconf)
        self.tmc2130.set_register("TCOOLTHRS", 0xfffff)
        self.mcu_endstop.home_prepare()
    def home_finalize(self):
        self.tmc2130.set_register("GCONF", self.tmc2130.reg_GCONF)
        self.tmc2130.set_register("TCOOLTHRS", 0)
        self.mcu_endstop.home_finalize()

def load_config_prefix(config):
    return TMC2130(config)
