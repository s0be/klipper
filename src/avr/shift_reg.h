#ifndef __SHIFT_REG_H
#define __SHIFT_REG_H

#include "autoconf.h"
#include <stdint.h> // uint8_t
#include <stdbool.h>
#include "gpio.h" // gpio_out

struct gpio_digital_regs *sr_get_fake_regs(uint8_t pin);
bool is_shift_reg(void *r);
void sr_flush(void);

#endif //__SHIFT_REG_H
