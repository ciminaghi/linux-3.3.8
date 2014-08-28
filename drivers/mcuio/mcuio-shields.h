#ifndef __MCUIO_SHIELDS_H__
#define __MCUIO_SHIELDS_H__

#include <linux/i2c.h>

struct mcuio_shld_i2c_info {
	char *type;
	unsigned short *paddr;
	struct i2c_client *i2c_client;
	void *platform_data;
};

struct mcuio_shld_data {
	struct i2c_adapter *i2c_adap;
	struct mcuio_shld_i2c_info *i2c_info;
	int i2c_cnt;
};

#define MCUIO_SHLD_I2C_DEV(t, pa, p) \
	{.type = t, .paddr = pa, .platform_data = p}

#endif /* __MCUIO_SHIELDS_H__ */
