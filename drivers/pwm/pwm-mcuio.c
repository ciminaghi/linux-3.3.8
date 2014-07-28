/*
 * Copyright 2014 Dog Hunter SA
 * Author: Aurelio Colosimo <aurelio@aureliocolosimo.it>
 *
 * GNU GPLv2 or later
 */

/* mcuio driver for PWM outputs */

#define DEBUG 1

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>

#include <linux/mcuio.h>
#include <linux/mcuio_ids.h>
#include <linux/mcuio-proto.h>

struct mcuio_pwm_chip {
	struct pwm_chip chip;
	/* FIXME TODO Add mcuio data */
};

static inline struct mcuio_pwm_chip *to_mcuio_pwm_chip(struct pwm_chip *chip)
{
	return container_of(chip, struct mcuio_pwm_chip, chip);
}

static int mcuio_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    int duty_ns, int period_ns)
{
	/* FIXME TODO */
	return -1;
}

static int mcuio_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				  enum pwm_polarity polarity)
{
	/* FIXME TODO */
	return -1;
}

static int mcuio_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	/* FIXME TODO */
	return -1;
}

static void mcuio_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	/* FIXME TODO */
}

static const struct pwm_ops mcuio_pwm_ops = {
	.config = mcuio_pwm_config,
	.set_polarity = mcuio_pwm_set_polarity,
	.enable = mcuio_pwm_enable,
	.disable = mcuio_pwm_disable,
	.owner = THIS_MODULE,
};

static int mcuio_pwm_probe(struct mcuio_device *mdev)
{
	/* FIXME TODO */
	return -1;
}

static int mcuio_pwm_remove(struct mcuio_device *mdev)
{
	/* FIXME TODO */
	return -1;
}

static const struct mcuio_device_id pwm_drv_ids[] = {
	{
		.class = MCUIO_CLASS_PWM,
		.class_mask = 0xffff,
	},
	/* Terminator */
	{
		.device = MCUIO_NO_DEVICE,
		.class = MCUIO_CLASS_UNDEFINED,
	},
};

static struct mcuio_driver mcuio_pwm_driver = {
	.driver = {
		.name = "mcuio-pwm",
	},
	.id_table = pwm_drv_ids,
	.probe = mcuio_pwm_probe,
	.remove = mcuio_pwm_remove,
};

static int __init mcuio_pwm_init(void)
{
	return mcuio_driver_register(&mcuio_pwm_driver, THIS_MODULE);
}

static void __exit mcuio_pwm_exit(void)
{
	mcuio_driver_unregister(&mcuio_pwm_driver);
}

subsys_initcall(mcuio_pwm_init);
module_exit(mcuio_pwm_exit);

MODULE_AUTHOR("Aurelio Colosimo");
MODULE_DESCRIPTION("MCUIO driver for PWM");
MODULE_LICENSE("GPL v2");
