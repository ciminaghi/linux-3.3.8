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
#include <linux/regmap.h>

#include <linux/mcuio.h>
#include <linux/mcuio_ids.h>
#include <linux/mcuio-proto.h>

static const struct regmap_config mcuio_pwm_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.max_register = 0xffc,
	.cache_type = REGCACHE_NONE,
};

struct mcuio_pwm_data {
	struct pwm_chip chip;
	struct regmap *map;
	u32 *ticks_ns;
	u32 *max_ticks;
};

static inline struct mcuio_pwm_data *to_mcuio_pwm_data(struct pwm_chip *chip)
{
	return container_of(chip, struct mcuio_pwm_data, chip);
}

static inline int pwm_idx(struct pwm_device *pwm)
{
	return pwm - pwm->chip->pwms;
}

static int mcuio_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    int duty_ns, int period_ns)
{
	struct mcuio_pwm_data *data = to_mcuio_pwm_data(chip);
	int idx = pwm_idx(pwm);

	if (regmap_write(data->map, 0x040 * (idx + 1) + 0x10,
		period_ns / data->ticks_ns[idx]))
		return -EIO;

	if (regmap_write(data->map, 0x040 * (idx + 1) + 0x14,
		duty_ns / data->ticks_ns[idx]))
		return -EIO;

	return 0;
}

static int mcuio_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				  enum pwm_polarity polarity)
{
	return -EINVAL;
}

static int mcuio_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct mcuio_pwm_data *data = to_mcuio_pwm_data(chip);
	int idx = pwm_idx(pwm);
	u32 st;
	u32 addr;

	addr = 0x040 * (idx + 1) + 0x0c;

	if (regmap_read(data->map, addr, &st))
		return -EIO;

	if (regmap_write(data->map, addr, st | 0x1))
		return -EIO;

	return 0;
}

static void mcuio_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct mcuio_pwm_data *data = to_mcuio_pwm_data(chip);
	int idx = pwm_idx(pwm);
	u32 st = 0;
	u32 addr;

	addr = 0x040 * (idx + 1) + 0x0c;

	if (regmap_read(data->map, addr, &st))
		dev_warn(chip->dev, "could not read current status while "
			"disabling pwm %d\n", idx);

	if (regmap_write(data->map, addr, st & ~0x1))
		dev_warn(chip->dev, "I/O error while disabling pwm %d\n",
			 idx);
}

static int mcuio_pwm_update_period(struct pwm_chip *chip,
				    struct pwm_device *pwm)
{
	struct mcuio_pwm_data *data = to_mcuio_pwm_data(chip);
	int idx = pwm_idx(pwm);
	u32 val;
	if (regmap_read(data->map, 0x040 * (idx + 1) + 0x10, &val))
		return -EIO;

	pwm->period = val * data->ticks_ns[idx];
	return 0;
}

static int mcuio_pwm_update_duty(struct pwm_chip *chip,
				 struct pwm_device *pwm)
{
	struct mcuio_pwm_data *data = to_mcuio_pwm_data(chip);
	int idx = pwm_idx(pwm);
	u32 val;
	if (regmap_read(data->map, 0x040 * (idx + 1) + 0x14, &val))
		return -EIO;

	pwm->duty_cycle = val * data->ticks_ns[idx];
	return 0;
}

static const struct pwm_ops mcuio_pwm_ops = {
	.config = mcuio_pwm_config,
	.set_polarity = mcuio_pwm_set_polarity,
	.enable = mcuio_pwm_enable,
	.disable = mcuio_pwm_disable,
	.update_period = mcuio_pwm_update_period,
	.update_duty = mcuio_pwm_update_duty,
	.owner = THIS_MODULE,
};

static int mcuio_pwm_probe(struct mcuio_device *mdev)
{
	int ret;
	struct mcuio_pwm_data *data = NULL;
	struct regmap *map = NULL;
	struct mcuio_device *hc = to_mcuio_dev(mdev->dev.parent);
	char *names;
	int i;

	if (!hc) {
		dev_err(&mdev->dev, "no parent for device\n");
		return -EINVAL;
	}

	map = devm_regmap_init_mcuio(mdev, &mcuio_pwm_regmap_config);

	if (IS_ERR(map)) {
		dev_err(&mdev->dev, "cannot setup regmap for device\n");
		return PTR_ERR(map);
	}

	data = devm_kzalloc(&mdev->dev, sizeof(*data), GFP_KERNEL);

	if (!data) {
		dev_err(&mdev->dev, "error creating pwm_data\n");
		ret = -ENOMEM;
		goto fail1;
	}

	data->map = map;

	pr_debug("mcuio pwm is %u:%u:%u\n",
		 mdev->bus, mdev->device, mdev->fn);

	data->chip.dev = &mdev->dev;
	data->chip.ops = &mcuio_pwm_ops;
	data->chip.base = -1;

	if (regmap_read(data->map, 0x08, &data->chip.npwm) < 0) {
		ret = -EIO;
		goto fail1;
	}

	dev_dbg(&mdev->dev, "%d pwm outputs detected\n", data->chip.npwm);

	dev_set_drvdata(&mdev->dev, data);

	ret = pwmchip_add(&data->chip);

	names = devm_kzalloc(&mdev->dev, 8 * data->chip.npwm, GFP_KERNEL);

	if (!names) {
		ret = -ENOMEM;
		goto fail1;
	}

	data->ticks_ns = devm_kzalloc(&mdev->dev, data->chip.npwm * sizeof(u32),
				      GFP_KERNEL);

	data->max_ticks = devm_kzalloc(&mdev->dev,
				      data->chip.npwm * sizeof(u32),
				      GFP_KERNEL);

	if (!data->max_ticks || !data->ticks_ns) {
		ret = -ENOMEM;
		goto fail1;
	}

	for (i = 0; i < data->chip.npwm; i++) {

		regmap_read(data->map, 0x040 * (i + 1), (u32*)&names[i * 8]);
		dev_dbg(&mdev->dev, "found pwm %d: %s\n", i, &names[i * 8]);

		data->chip.pwms[i].label = &names[i * 8];

		regmap_read(data->map, 0x040 * (i + 1) + 0x04,
			    &data->ticks_ns[i]);

		regmap_read(data->map, 0x040 * (i + 1) + 0x08,
			    &data->max_ticks[i]);
	}
	return ret;

fail1:
	return ret;
}

static int mcuio_pwm_remove(struct mcuio_device *mdev)
{
	struct mcuio_pwm_data *data = dev_get_drvdata(&mdev->dev);

	pr_debug("removing mcuio pwm %u:%u:%u\n",
		 mdev->bus, mdev->device, mdev->fn);
	BUG_ON(!data);

	return pwmchip_remove(&data->chip);
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
