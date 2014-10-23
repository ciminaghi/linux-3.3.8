/*
 * Copyright 2011 Dog Hunter SA
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * GNU GPLv2 or later
 */

#define DEBUG

/* mcuio driver for joystick shield */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/hid.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/platform_data/ssd1307.h>


#include <linux/mcuio.h>
#include <linux/mcuio_ids.h>

#include "mcuio-internal.h"

static const struct regmap_config mcuio_irq_test_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.max_register = 0x8,
	.cache_type = REGCACHE_NONE,
};


static irqreturn_t mcuio_irq_test_irq_handler(int irq, void *__data)
{
	struct regmap *map = __data;
	int stat;
	u32 status;

	stat = regmap_read(map, 0xc, &status);
	WARN_ON(!status);

	return IRQ_HANDLED;
}

static int mcuio_irq_test_probe(struct mcuio_device *mdev)
{
	int ret = 0;
	unsigned int v = 1;
	struct regmap *map;

	dev_dbg(&mdev->dev, "%s entered\n", __func__);

	map = devm_regmap_init_mcuio(mdev, &mcuio_irq_test_regmap_config);
	if (IS_ERR(map)) {
		dev_err(&mdev->dev, "cannot init regmap\n");
		return PTR_ERR(map);
	}

	ret = devm_request_threaded_irq(&mdev->dev, mdev->irq,
					NULL,
					mcuio_irq_test_irq_handler,
					0,
					"mcuio-irq-test",
					map);
	if (ret < 0)
		return ret;

	ret = regmap_read(map, 0xc, &v);
	if (ret < 0) {
		dev_err(&mdev->dev, "Error cleaning up irq status\n");
		return ret;
	}

	/* Immediately enable interrupt */
	ret = regmap_write(map, 0xc, 1);
	if (ret < 0) {
		dev_err(&mdev->dev, "Error enabling interrupt\n");
		return ret;
	}


	dev_set_drvdata(&mdev->dev, map);

	dev_dbg(&mdev->dev, "%s returns ok\n", __func__);

	return ret;
}

static int mcuio_irq_test_remove(struct mcuio_device *mdev)
{
	int ret;
	unsigned int v = 0;
	struct regmap *map;

	map = dev_get_drvdata(&mdev->dev);

	ret = regmap_write(map, 0xc, v);
	if (ret < 0) {
		dev_err(&mdev->dev, "Error stopping irq tester\n");
		return ret;
	}
	return 0;
}

static const struct mcuio_device_id irq_test_drv_ids[] = {
	{
		.vendor = MCUIO_VENDOR_DOGHUNTER,
		.device = 0x1212,
	},
	/* Terminator */
	{
		.device = MCUIO_NO_DEVICE,
		.class = MCUIO_CLASS_UNDEFINED,
	},
};

static struct mcuio_driver mcuio_irq_test_driver = {
	.driver = {
		.name = "mcuio-irq-test",
	},
	.id_table = irq_test_drv_ids,
	.probe = mcuio_irq_test_probe,
	.remove = mcuio_irq_test_remove,
};

static int __init mcuio_irq_test_init(void)
{
	return mcuio_driver_register(&mcuio_irq_test_driver, THIS_MODULE);
}

static void __exit mcuio_irq_test_exit(void)
{
	return mcuio_driver_unregister(&mcuio_irq_test_driver);
}

subsys_initcall(mcuio_irq_test_init);
module_exit(mcuio_irq_test_exit);

MODULE_AUTHOR("Davide Ciminaghi");
MODULE_DESCRIPTION("MCUIO driver for irq test function");
MODULE_LICENSE("GPL v2");
