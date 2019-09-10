/* linux/input/gsl1680.h
 *
 * Platform data for GSL1680 driver
 *
 * Copyright (c) 2015 Gerhard de Clercq (gerharddeclercq@outlook.com)
 *
 *  GPL LICENSE
 */

#ifndef __LINUX_I2C_GSL1680_H
#define __LINUX_I2C_GSL1680_H

/* Board specific touch screen initial values */
struct gsl1680_init_data {
	int	(*pinmux_fusion_pins)(void);
	int	gpio_int;
	int	gpio_reset;
};

#endif /*  __LINUX_I2C_GSL1680_H */