/*
 * Copyright 2014 Dog Hunter SA
 * Author: Aurelio Colosimo <aurelio@aureliocolosimo.it>
 *
 * GNU GPLv2 or later
 */

/* mcuio driver for ADC inputs */

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
#include <linux/hid.h>
#include <linux/platform_device.h>


#include <linux/mcuio.h>
#include <linux/mcuio_ids.h>
#include <linux/mcuio-proto.h>

#include "mcuio-internal.h"

static int mcuio_adc_probe(struct mcuio_device *mdev)
{
	return -ENODEV;
}

static int mcuio_adc_remove(struct mcuio_device *mdev)
{
	return 0;
}

static const struct mcuio_device_id adc_drv_ids[] = {
	{
		.class = MCUIO_CLASS_ADC,
		.class_mask = 0xffff,
	},
	/* Terminator */
	{
		.device = MCUIO_NO_DEVICE,
		.class = MCUIO_CLASS_UNDEFINED,
	},
};

static struct mcuio_driver mcuio_adc_driver = {
	.driver = {
		.name = "mcuio-adc",
	},
	.id_table = adc_drv_ids,
	.probe = mcuio_adc_probe,
	.remove = mcuio_adc_remove,
};

static int __init mcuio_adc_init(void)
{
	return mcuio_driver_register(&mcuio_adc_driver, THIS_MODULE);
}

static void __exit mcuio_adc_exit(void)
{
	return mcuio_driver_unregister(&mcuio_adc_driver);
}

subsys_initcall(mcuio_adc_init);
module_exit(mcuio_adc_exit);

MODULE_AUTHOR("Aurelio Colosimo");
MODULE_DESCRIPTION("MCUIO driver for ADC");
MODULE_LICENSE("GPL v2");
