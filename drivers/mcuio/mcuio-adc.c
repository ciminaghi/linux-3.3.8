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

static struct class *mcuio_adc_class;

static const struct regmap_config mcuio_adc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.max_register = 0xffc,
	.cache_type = REGCACHE_NONE,
};


struct mcuio_adc_in {
	const char *name;
	int mcu_in;
	uint32_t capab;
	struct device *dev;
	struct adc_data *data;
};

/* Mapping from mcu gpio to board inputs */
struct mcuio_adc_in linino_one_in[] = {
	{
		.name = "A0",
		.mcu_in = 7,
	},
	{
		.name = "A1",
		.mcu_in = 6,
	},
	{
		.name = "A2",
		.mcu_in = 5,
	},
	{
		.name = "A3",
		.mcu_in = 4,
	},
	{
		.name = "A4",
		.mcu_in = 1,
	},
	{
		.name = "A5",
		.mcu_in = 0,
	},
	{
		.name = NULL,
	}
};

struct adc_data {
	struct mcuio_adc_in *in;
	struct regmap *map;
	struct device *dev;
	int in_created;
};

#define __regmap_read(m, a, v) \
	if (regmap_read(m, a, v)) \
		dev_err(dev, "I/O error in %s\n", __func__); \

#define __regmap_write(m, a, v) \
	if (regmap_write(m, a, v)) \
		dev_err(dev, "I/O error in %s\n", __func__);


static inline void mcuio_adc_disable(struct adc_data *data) {
	regmap_write(data->map, 0x14, 0);
}

/* sysfs interface begin */

ssize_t show_value(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	uint32_t val;
	uint32_t addr;
	struct mcuio_adc_in *in = dev_get_drvdata(dev);

	addr = 0x040 * (in->mcu_in + 1) + 0x0c;
	__regmap_read(in->data->map, addr, &val);
	return sprintf(buf, "%d", val);
}

ssize_t show_nbits(struct device *dev, struct device_attribute *attr,
			char *buf)
{
#if 0 /* FIXME: to be fixed in bathos-mcuio */
	struct mcuio_adc_in *in = dev_get_drvdata(dev);
	return sprintf(buf, "%u", (in->capab >> 1) & 0x3f);
#endif
	return sprintf(buf, "10");
}

ssize_t show_signd(struct device *dev, struct device_attribute *attr,
			char *buf)
{
#if 0 /* FIXME: to be fixed in bathos-mcuio */
	struct mcuio_adc_in *in = dev_get_drvdata(dev);
	return sprintf(buf, "%u", in->capab & 0x1);
#endif
	return sprintf(buf, "0");
}

static DEVICE_ATTR(value, 0444, show_value, NULL);
static DEVICE_ATTR(nbits, 0444, show_nbits, NULL);
static DEVICE_ATTR(signd, 0444, show_signd, NULL);

static struct attribute *adc_in_dev_attrs[] = {
	&dev_attr_value.attr,
	&dev_attr_nbits.attr,
	&dev_attr_signd.attr,
	NULL,
};

struct attribute_group adc_in_dev_attr_group = {
	.attrs = adc_in_dev_attrs,
};

static ssize_t show_enable(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	uint32_t en = 0;
	struct adc_data *data;
	data = dev_get_drvdata(dev);
	__regmap_read(data->map, 0x14, &en);
	return sprintf(buf, "%u", en & 0x1);
}

static int in_devs_create(struct adc_data *data)
{
	struct mcuio_adc_in *in = data->in;
	uint32_t addr;
	mcuio_adc_class = class_create(THIS_MODULE, "mcuio_adc");
	while (in->name) {

		/* Read capabilities and store its constant value */
		addr = 0x040 * (in->mcu_in + 1) + 0x04;
		regmap_read(data->map, addr, &in->capab);

		in->data = data;

		in->dev = device_create(mcuio_adc_class, data->dev,
			MKDEV(0, in - data->in), in, in->name);

		if (!in->dev) {
			dev_err(data->dev, "error creating dev %s\n",
				in->name);
			return -ENOMEM;
		}

		sysfs_create_group(&in->dev->kobj,
					&adc_in_dev_attr_group);
		in++;
	}

	data->in_created = 1;

	return 0;
}

static void in_devs_destroy(struct adc_data *data)
{
	struct mcuio_adc_in *in = data->in;
	while (in->name) {
		if (in->dev) {
			device_destroy(mcuio_adc_class,
				       MKDEV(0, in - data->in));
			in->dev = NULL;
		}
		in++;
	}
	data->in_created = 0;
	class_destroy(mcuio_adc_class);
	mcuio_adc_class = NULL;
}

static ssize_t store_enable(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	uint32_t _en, en_new;
	struct adc_data *data = dev_get_drvdata(dev);

	__regmap_read(data->map, 0x14, &_en);

	en_new = (buf[0] - '0') & 0x1;

	__regmap_write(data->map, 0x14, (_en & ~0x1) | en_new);

	if (en_new && !data->in_created) {
		/* If switched to enabled, create nodes in
		 * /sys/class/mcuio_adc */
		in_devs_create(data);
	}
	else if (!en_new && data->in_created) {
		/* If switched to disabled, destroy nodes in
		 * /sys/class/mcuio_adc */
		in_devs_destroy(data);
	}

	return count;
}

static ssize_t show_aref(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	/* FIXME todo */
	return sprintf(buf, "Not implemented yet");
}

static ssize_t store_aref(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/* FIXME todo */
	return count;
}

static DEVICE_ATTR(enable, 0644, show_enable, store_enable);
/* FIXME todo
static DEVICE_ATTR(aref, 0644, show_aref, store_aref);
*/

/* sysfs interface end */

static int mcuio_adc_probe(struct mcuio_device *mdev)
{
	int ret;
	struct adc_data *data = NULL;
	struct regmap *map = NULL;
	struct mcuio_device *hc = to_mcuio_dev(mdev->dev.parent);

	if (!hc) {
		dev_err(&mdev->dev, "no parent for device\n");
		return -EINVAL;
	}

	map = devm_regmap_init_mcuio(mdev, &mcuio_adc_regmap_config);

	if (IS_ERR(map)) {
		dev_err(&mdev->dev, "cannot setup regmap for device\n");
		return PTR_ERR(map);
	}

	data = devm_kzalloc(&mdev->dev, sizeof(*data), GFP_KERNEL);

	if (!data) {
		dev_err(&mdev->dev, "error creating adc_data\n");
		ret = -ENOMEM;
		goto fail1;
	}

	data->map = map;

	pr_debug("mcuio adc is %u:%u:%u\n",
		 mdev->bus, mdev->device, mdev->fn);

	data->in = linino_one_in;
	data->dev = &mdev->dev;

	dev_set_drvdata(&mdev->dev, data);

	ret = sysfs_create_link(&hc->dev.kobj, &mdev->dev.kobj, "adc");
	if (ret)
		goto fail1;

	ret = device_create_file(&mdev->dev, &dev_attr_enable);
	if (ret) {
		dev_err(&mdev->dev, "error creating adc enable file\n");
		goto fail2;
	}

	mcuio_adc_disable(data);

	return 0;

fail2:
	sysfs_remove_link(&hc->dev.kobj, "adc");
fail1:
	return ret;
}

static int mcuio_adc_remove(struct mcuio_device *mdev)
{
	struct adc_data *data = dev_get_drvdata(&mdev->dev);
	struct mcuio_device *hc = to_mcuio_dev(mdev->dev.parent);

	pr_debug("removing mcuio adc %u:%u:%u\n",
		 mdev->bus, mdev->device, mdev->fn);
	BUG_ON(!data);

	if (data->in_created)
		in_devs_destroy(data);

	device_remove_file(&mdev->dev, &dev_attr_enable);

	sysfs_remove_link(&hc->dev.kobj, "adc");

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
	mcuio_driver_unregister(&mcuio_adc_driver);
}

subsys_initcall(mcuio_adc_init);
module_exit(mcuio_adc_exit);

MODULE_AUTHOR("Aurelio Colosimo");
MODULE_DESCRIPTION("MCUIO driver for ADC");
MODULE_LICENSE("GPL v2");
