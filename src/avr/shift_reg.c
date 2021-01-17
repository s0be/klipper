// Generic Shift Register Support
//
// Copyright (C) 2018  Trevor Jones <trevorjones141@gmail.com>
// Copyright (C) 2021  Pat Erley <paerley@gmail.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <basecmd.h>
#include <stdbool.h>
#include "shift_reg.h"
#include "internal.h"
#include "command.h" // DECL_COMMAND
#include "compiler.h" // ARRAY_SIZE
#include "sched.h" // DECL_INIT

#define STR(X) #X

struct gpio_out data_pin;
struct gpio_out latch_pin;
struct gpio_out clock_pin;

struct fake_reg {
  volatile uint8_t in;
  volatile uint8_t mode;
  volatile uint8_t out;
};

#define reg_len (CONFIG_SHIFT_REG_LENGTH/8)
struct fake_reg shift_regs[reg_len];
uint8_t shift_regs_written[reg_len];
void *hc_start = &shift_regs[0];
void *hc_end = &shift_regs[reg_len - 1];

DECL_CONSTANT_STR("RESERVE_PINS_shift_reg",
		  STR(CONFIG_SR_DATA_GPIO) ","
		  STR(CONFIG_SR_LATCH_GPIO) ","
		  STR(CONFIG_SR_CLOCK_GPIO)
		 );
void sr_init(void)
{
    for (int i = 0; i < reg_len; i++) {
        shift_regs[i].in = 0;
	shift_regs[i].mode = 0;
	shift_regs[i].out = 0;
    }
    data_pin = gpio_out_setup(CONFIG_SR_DATA_GPIO, 0); // D9 PB5
    latch_pin = gpio_out_setup(CONFIG_SR_LATCH_GPIO, 1); // D10 PB6
    clock_pin = gpio_out_setup(CONFIG_SR_CLOCK_GPIO, 0); // D13 PC7
    sr_flush();
}
DECL_INIT(sr_init);

inline bool
is_shift_reg(void *g) {
    return (g >= hc_start) && (g <= hc_end);
}

struct gpio_digital_regs *
sr_get_fake_regs(uint8_t pin) {
	return (struct gpio_digital_regs *)(&shift_regs[pin/8]);
}

void
sr_flush(void)
{
    // flushing isn't cheap, so only do it if a value has changed;
    bool changed = false;
    for(int i = 0; i < reg_len; i++) {
        if(shift_regs_written[i] != shift_regs[i].out) {
            changed = true;
	    break;
	}
    }

    if(changed) {
        gpio_out_write(latch_pin, 0);
        //for each bit in reverse
        for (int ic = reg_len - 1; ic >= 0; --ic)
        {
            uint8_t reg = shift_regs[ic].out;
            for (int bit = 7; bit >= 0; --bit)
            {
                //set data
                gpio_out_write(data_pin, (reg >> bit) & 1);
                //clock pulse
                gpio_out_write(clock_pin, 1);
                gpio_out_write(clock_pin, 0);
            }
	    shift_regs_written[ic] = reg;
        }
        //latch on
        gpio_out_write(latch_pin, 1);
    }
}
