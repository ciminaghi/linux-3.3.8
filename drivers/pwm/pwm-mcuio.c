/*
 * Copyright 2014 Dog Hunter SA
 * Author: Aurelio Colosimo <aurelio@aureliocolosimo.it>
 *
 * GNU GPLv2 or later
 */

/* mcuio driver for PWM outputs */

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

#define CAP_RES(x)     (x & 0x00ffffff)
#define CAP_CH_PER(x)  ((x >> 24) & 0x1)
#define CAP_CH_DC(x)   ((x >> 25) & 0x1)

static const struct regmap_config mcuio_pwm_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.max_register = 0xffc,
	.cache_type = REGCACHE_NONE,
};

struct mcuio_pwm_data {
	struct pwm_chip chip;
	struct regmap *map;
	u32 *capab;
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
	uint32_t duty_ticks;

	if (!CAP_CH_PER(data->capab[idx]))
		goto set_duty;

	if (regmap_write(data->map, 0x040 * (idx + 1) + 0x10,
		period_ns / pwm->resolution))
		return -EIO;

set_duty:
	if (!CAP_CH_DC(data->capab[idx]))
		goto done;

	duty_ticks = duty_ns / pwm->resolution;

	if (regmap_write(data->map, 0x040 * (idx + 1) + 0x14, duty_ticks))
		return -EIO;

done:
	return 0;
}

static int mcuio_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				  enum pwm_polarity polarity)
{
	struct mcuio_pwm_data *data = to_mcuio_pwm_data(chip);
	int idx = pwm_idx(pwm);
	u32 st;
	u32 addr;

	addr = 0x040 * (idx + 1) + 0x0c;

	if (regmap_read(data->map, addr, &st))
		return -EIO;

	if (polarity == PWM_POLARITY_NORMAL)
		st &= ~(1 << 1);
	else
		st |= (1 << 1);

	if (regmap_write(data->map, addr, st))
		return -EIO;

	return 0;
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

	pwm->period = val * pwm->resolution;
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

	pwm->duty_cycle = val * pwm->resolution;
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
	u32 max_ticks;

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

	pr_info("mcuio pwm is %u:%u:%u\n",
		 mdev->bus, mdev->device, mdev->fn);

	data->chip.dev = &mdev->dev;
	data->chip.ops = &mcuio_pwm_ops;
	data->chip.base = -1;

	if (regmap_read(data->map, 0x08, &data->chip.npwm) < 0) {
		ret = -EIO;
		goto fail1;
	}

	dev_info(&mdev->dev, "%d pwm outputs detected\n", data->chip.npwm);

	dev_set_drvdata(&mdev->dev, data);

	ret = pwmchip_add(&data->chip);

	names = devm_kzalloc(&mdev->dev, 8 * data->chip.npwm, GFP_KERNEL);

	if (!names) {
		ret = -ENOMEM;
		goto fail1;
	}

	data->capab = devm_kzalloc(&mdev->dev,
		sizeof(*data->capab) * data->chip.npwm,
		GFP_KERNEL);

	if (!data->capab) {
		ret = -ENOMEM;
		goto fail1;
	}

	for (i = 0; i < data->chip.npwm; i++) {

		regmap_read(data->map, 0x040 * (i + 1), (u32*)&names[i * 8]);
		dev_dbg(&mdev->dev, "found pwm %d: %s\n", i, &names[i * 8]);

		data->chip.pwms[i].name = &names[i * 8];

		regmap_read(data->map, 0x040 * (i + 1) + 0x04,
			    &data->capab[i]);

		data->chip.pwms[i].resolution = CAP_RES(data->capab[i]);

		regmap_read(data->map, 0x040 * (i + 1) + 0x08,
			    &max_ticks);

		data->chip.pwms[i].max = max_ticks *
			data->chip.pwms[i].resolution;
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
