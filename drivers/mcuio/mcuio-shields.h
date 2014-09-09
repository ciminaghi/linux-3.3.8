#ifndef __MCUIO_SHIELDS_H__
#define __MCUIO_SHIELDS_H__

#include <linux/i2c.h>

struct mcuio_shld_i2c_info {
	unsigned short *paddr;
	struct i2c_client *i2c_client;
	struct i2c_board_info info;
	int gpio_irq;
};

struct mcuio_shld_data {
	struct i2c_adapter *i2c_adap;
	struct mcuio_shld_i2c_info *i2c_info;
	int i2c_cnt;
};

#define MCUIO_SHLD_I2C_DEV(t, pa, p, g) \
	{.paddr = pa, .gpio_irq = g,\
		.info = {.platform_data = p, .type = t}}

#endif /* __MCUIO_SHIELDS_H__ */
