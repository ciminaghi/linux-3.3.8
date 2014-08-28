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
#include <linux/mcuio-proto.h>

#include "mcuio-internal.h"

#define JS_MAX_NLEDS 2
#define LED_MAX_NAMELEN 40

/* The HID report descriptor (just 4 buttons at present) */
static char mcuio_js_report_descriptor[] = {
	/* USAGE_PAGE (Button) */
	0x05, 0x09,
	/* USAGE_MINIMUM (Button 1) */
	0x19, 0x01,
	/* USAGE_MAXIMUM (Button 6) */
	0x29, 0x06,
	/* LOGICAL_MINIMUM (0) */
	0x15, 0x00,
	/* LOGICAL_MAXIMUM (1) */
	0x25, 0x01,
	/* REPORT_COUNT (6) */
	0x95, 0x06,
	/* REPORT_SIZE (1) */
	0x75, 0x01,
	/* INPUT (Data,Var,Abs) */
	0x81, 0x02,
	/* REPORT_COUNT (2) */
	0x95, 0x02,
	/* REPORT_SIZE (1) */
	0x75, 0x01,
};

struct mcuio_js_data;
struct mcuio_js_gpio;
struct mcuio_js_led;

struct mcuio_js_gpio_config {
	const char *name;
	int (*setup)(struct mcuio_device *mdev, struct mcuio_js_gpio *data,
		     int index);
};

struct mcuio_js_gpio {
	const struct mcuio_js_gpio_config *cfg;
	unsigned gpio;
	int irq;
	int index;
	struct mcuio_js_data *js_data;
	struct mcuio_js_led *led;
	struct list_head list;
};

struct mcuio_js_led {
	struct mcuio_js_gpio *gpio;
	enum led_brightness curr_brightness;
	struct led_classdev led;
	struct work_struct work;
};

struct mcuio_js_data {
	struct hid_device *hid;
	u8 cached_gpios;
	int oled_reset_gpio;
	struct i2c_adapter *i2c_adap;
	struct i2c_client *oled_i2c_client;
	struct list_head gpios;
};


static int __match_gpiochip(struct gpio_chip *chip, void *__gpio_data)
{
	struct mcuio_js_gpio *data = __gpio_data;
	const char *ptr;
	int i;

	pr_debug("%s entered (name = %s)\n", __func__, data->cfg->name);

	if (!chip->names) {
		pr_debug("%s: gpiochip has no names\n", __func__);
		return 0;
	}
	for (i = 0; i < chip->ngpio; i++, ptr++) {
		ptr = chip->names[i];
		if (!ptr)
			continue;
		pr_debug("%s: found gpio %s\n", __func__, chip->names[i]);
		if (!strcmp(ptr, data->cfg->name)) {
			data->gpio = i + chip->base;
			pr_debug("%s: gpiochip found\n", __func__);
			return 1;
		}
	}
	pr_debug("%s: gpiochip not found\n", __func__);
	return 0;
}

static int __setup_button(struct mcuio_device *mdev, struct mcuio_js_gpio *data,
			  int index)
{
	int ret = -ENODEV;
	ret = devm_gpio_request_one(&mdev->dev, data->gpio, GPIOF_DIR_IN,
				    "js-shield");
	if (ret < 0)
		return ret;
	/* HACK FOR ATMEGA : THIS IS NEEDED TO ENABLE PULLUP */
	ret = gpio_direction_output(data->gpio, 1);
	if (ret) {
		dev_err(&mdev->dev,
			"gpio%u: error setting direction to output\n",
			data->gpio);
		return ret;
	}
	ret = gpio_direction_input(data->gpio);
	if (ret) {
		dev_err(&mdev->dev,
			"gpio%u: error setting direction to input\n",
			data->gpio);
		return ret;
	}
	ret = gpio_to_irq(data->gpio);
	if (ret < 0) {
		dev_err(&mdev->dev,
			"gpio%u: gpio_to_irq returned error\n",
			data->gpio);
		return ret;
	}
	data->irq = ret;
	data->index = index;
	return 0;
}

static void mcuio_js_led_work(struct work_struct *work)
{
	struct mcuio_js_led *led =
		container_of(work, struct mcuio_js_led, work);
	struct mcuio_js_gpio *gpio = led->gpio;

	/* Active low gpios */
	gpio_set_value_cansleep(gpio->gpio,
				led->curr_brightness != LED_OFF ? 0 : 1);
}


static void mcuio_js_led_brightness_set(struct led_classdev *led_cdev,
					enum led_brightness brightness)
{
	struct mcuio_js_led *led = container_of(led_cdev,
						struct mcuio_js_led,
						led);
	led->curr_brightness = brightness;
	schedule_work(&led->work);
}

static int __setup_led(struct mcuio_device *mdev, struct mcuio_js_gpio *data,
		       int index)
{
	int ret;
	char *n;
	const char *color;

	ret = devm_gpio_request_one(&mdev->dev, data->gpio, GPIOF_DIR_IN,
				    "js-shield");
	if (ret < 0)
		return ret;
	ret = gpio_direction_output(data->gpio, 1);
	if (ret) {
		dev_err(&mdev->dev,
			"gpio%u: error setting direction to output\n",
			data->gpio);
		return ret;
	}

	data->led = devm_kzalloc(&mdev->dev, sizeof(*data->led), GFP_KERNEL);
	if (!data->led)
		return -ENOMEM;
	n = devm_kzalloc(&mdev->dev, LED_MAX_NAMELEN, GFP_KERNEL);
	if (!n)
		return -ENOMEM;
	color = !strncmp(data->cfg->name, "LEDR", 4) ? "red" : "green";
	snprintf(n, LED_MAX_NAMELEN, "mcuio-%s-%s:%s:", dev_name(&mdev->dev),
		 data->cfg->name, color);
	data->led->gpio = data;
	data->led->led.name = n;
	data->led->led.brightness = LED_OFF;
	data->led->led.max_brightness = LED_FULL;
	data->led->led.brightness_set = mcuio_js_led_brightness_set;
	ret = led_classdev_register(&mdev->dev, &data->led->led);
	if (ret)
		return ret;

	INIT_WORK(&data->led->work, mcuio_js_led_work);
	data->irq = 0;
	data->index = index;
	return 0;
}

static int __setup_oled_reset(struct mcuio_device *mdev,
			      struct mcuio_js_gpio *data,
			      int index)
{
	data->js_data->oled_reset_gpio = data->gpio;
	data->irq = 0;
	data->index = index;
	return 0;
}

static irqreturn_t mcuio_js_irq_handler(int irq, void *__data)
{
	int v;
	struct mcuio_js_gpio *gpio_data = __data;
	struct mcuio_js_data *js_data = gpio_data->js_data;
	int gpio_index = gpio_data->index;
	pr_debug("%s entered (gpio %u), index = %d", __func__, gpio_data->gpio,
		 gpio_data->index);

	v = !!!gpio_get_value_cansleep(gpio_data->gpio);
	js_data->cached_gpios &= ~(1 << gpio_index);
	if (v)
		js_data->cached_gpios |= (1 << gpio_index);

	/* Send out a report now */
	hid_input_report(js_data->hid, HID_INPUT_REPORT, &js_data->cached_gpios,
			 1, 1);

	return IRQ_HANDLED;
}

static int mcuio_js_hid_get_raw_report(struct hid_device *hid,
				       unsigned char report_number,
				       __u8 *buf, size_t count,
				       unsigned char report_type)
{
	struct mcuio_device *mdev = hid->driver_data;
	struct mcuio_js_data *js_data;
	pr_debug("%s invoked, report_number = %u, report_type = %u\n",
		 __func__, report_number, report_type);
	if (!mdev) {
		pr_err("no mcuio device !\n");
		return -ENODEV;
	}
	js_data = dev_get_drvdata(&mdev->dev);
	if (!js_data) {
		dev_err(&mdev->dev, "no drv data !\n");
		return -ENODEV;
	}
	if (report_type == HID_OUTPUT_REPORT)
		return -EINVAL;
	if (report_type == HID_FEATURE_REPORT)
		/* Unsupported at the moment */
		return -EINVAL;
	if (count != 1) {
		pr_err("%s: invalid count %zu\n", __func__, count);
		return -EINVAL;
	}
	/* FIXME !! */
	buf[0] = js_data->cached_gpios;
	return 1;
}


static int mcuio_js_hid_output_raw_report(struct hid_device *hid, __u8 *buf,
					  size_t count,
					  unsigned char report_type)
{
	pr_debug("%s invoked, report_type = %u\n", __func__, report_type);
	return -EINVAL;
}


static int mcuio_js_hid_start(struct hid_device *hid)
{
	struct mcuio_device *mdev = hid->driver_data;
	struct mcuio_js_data *js_data;
	struct mcuio_js_gpio *data;
	int i = 0, ret;

	hid_dbg(hid, "%s invoked\n", __func__);
	if (!mdev) {
		hid_err(hid, "%s: mdev is NULL\n", __func__);
		return -ENODEV;
	}
	js_data = dev_get_drvdata(&mdev->dev);
	if (!js_data) {
		hid_err(hid, "%s: js_data is NULL\n", __func__);
		return -ENODEV;
	}
	list_for_each_entry(data, &js_data->gpios, list) {
		unsigned int v;
		if (!data->irq) {
			i++;
			continue;
		}
		ret = devm_request_threaded_irq(&mdev->dev, data->irq,
						NULL,
						mcuio_js_irq_handler,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						"mcuio-js",
						data);
		if (ret)
			return ret;
		v = !!!gpio_get_value_cansleep(data->gpio);
		if (v)
			js_data->cached_gpios |= (1 << i);
		i++;
	}
	hid_dbg(hid, "hw start ok\n");
	return 0;
}

static void mcuio_js_hid_stop(struct hid_device *hid)
{
}

static int mcuio_js_hid_open(struct hid_device *hid)
{
	pr_debug("%s invoked\n", __func__);
	return 0;
}

static void mcuio_js_hid_close(struct hid_device *hid)
{
	pr_debug("%s invoked\n", __func__);
}

static int mcuio_js_hid_input(struct input_dev *input, unsigned int type,
			      unsigned int code, int value)
{
	pr_debug("%s invoked\n", __func__);
	return 0;
}

static int mcuio_js_hid_parse(struct hid_device *hid)
{
	return hid_parse_report(hid, mcuio_js_report_descriptor,
				sizeof(mcuio_js_report_descriptor));
}


static struct hid_ll_driver mcuio_js_hid_ll_driver = {
	.start = mcuio_js_hid_start,
	.stop = mcuio_js_hid_stop,
	.open = mcuio_js_hid_open,
	.close = mcuio_js_hid_close,
	.hidinput_input_event = mcuio_js_hid_input,
	.parse = mcuio_js_hid_parse,
};

/* gpio configs for "old" joystick shield */
static const struct mcuio_js_gpio_config js_gpios[] = {
	{
		.name = "SCL",
		.setup = __setup_button,
	},
	{
		.name = "D4",
		.setup = __setup_button,
	},
	{
		.name = "D5",
		.setup = __setup_button,
	},
	{
		.name = "D6",
		.setup = __setup_button,
	},
	{
		.name = NULL,
	},
};

static const struct mcuio_js_gpio_config lucky_gpios[] = {
	{
		.name = "JOYU",
		.setup = __setup_button,
	},
#if 0
	/* D7 is the handshake signal, not available */
	{
		.name = "JOYC",
		.setup = __setup_button,
	},
#endif
	{
		.name = "JOYR",
		.setup = __setup_button,
	},
	{
		.name = "JOYD",
		.setup = __setup_button,
	},
	{
		.name = "JOYL",
		.setup = __setup_button,
	},
	{
		.name = "PB1",
		.setup = __setup_button,
	},
	{
		.name = "LEDR",
		.setup = __setup_led,
	},
	{
		.name = "LEDY",
		.setup = __setup_led,
	},
	{
		.name = "ORES",
		.setup = __setup_oled_reset,
	},
	{
		.name = NULL,
	},
};

static int __setup_i2c_adapter(struct mcuio_device *mdev,
			      struct mcuio_js_data *js_data)
{
	js_data->i2c_adap = mcuio_get_i2c_adapter(mdev);
	return js_data->i2c_adap ? 0 : -ENODEV;
}

static int __instantiate_oled(struct mcuio_device *mdev,
			      struct mcuio_js_data *js_data)
{
	struct ssd1307_platform_data plat = {
		.type = SSD1307_TYPE_1306,
		.reset_gpio = js_data->oled_reset_gpio,
		.width = 128,
		.height = 64,
		.page_offset = 0,
		.pins_config = 0x12,
		.display_offset = 0,
	};
	struct i2c_board_info oled_board_info = {
		I2C_BOARD_INFO("ssd1307fb", 0x3c),
	};
	struct i2c_board_info *binfo = devm_kzalloc(&mdev->dev,
						    sizeof(plat) +
						    sizeof(oled_board_info),
						    GFP_KERNEL);
	if (!binfo)
		return -ENOMEM;

	memcpy(binfo, &oled_board_info, sizeof(*binfo));
	binfo->platform_data = &binfo[1];
	memcpy(&binfo[1], &plat, sizeof(plat));

	js_data->oled_i2c_client = i2c_new_device(js_data->i2c_adap, binfo);
	if (IS_ERR(js_data->oled_i2c_client))
		return PTR_ERR(js_data->oled_i2c_client);
	return 0;
}

static int mcuio_js_probe(struct mcuio_device *mdev)
{
	int i, ret = 0;
	struct hid_device *hid;
	struct mcuio_js_data *js_data;
	const struct mcuio_js_gpio_config *cfg = NULL;

	dev_dbg(&mdev->dev, "%s entered\n", __func__);
	js_data = devm_kzalloc(&mdev->dev, sizeof(*js_data), GFP_KERNEL);
	if (!js_data) {
		dev_err(&mdev->dev, "no memory for js data structure\n");
		return -ENOMEM;
	}
	js_data->oled_reset_gpio = -1;
	INIT_LIST_HEAD(&js_data->gpios);
	dev_dbg(&mdev->dev, "%s: device = 0x%04x\n", __func__, mdev->device);
	if (mdev->id.device == MCUIO_DEVICE_JOYSTICK_SHIELD)
		cfg = js_gpios;
	if (mdev->id.device == MCUIO_DEVICE_LUCKY_SHIELD)
		cfg = lucky_gpios;
	if (!cfg) {
		WARN_ON(1);
		return -ENODEV;
	}

	for (i = 0; cfg->name && cfg->name[0]; cfg++, i++) {
		struct gpio_chip *chip;
		struct mcuio_js_gpio *data = devm_kzalloc(&mdev->dev,
							  sizeof(*data),
							  GFP_KERNEL);
		if (!data) {
			dev_err(&mdev->dev, "no memory for gpio structure\n");
			return -ENOMEM;
		}
		data->cfg = cfg;
		data->js_data = js_data;
		chip = gpiochip_find(data, __match_gpiochip);
		if (!chip) {
			dev_dbg(&mdev->dev,
				"%s: gpiochip not found\n", __func__);
			return ret;
		}
		if (data->cfg->setup) {
			ret = data->cfg->setup(mdev, data, i);
			if (ret < 0)
				return ret;
		}
		list_add_tail(&data->list, &js_data->gpios);
	}

	ret = __setup_i2c_adapter(mdev, js_data);
	if (ret < 0) {
		dev_err(&mdev->dev, "error setting up i2c adapter\n");
		return ret;
	}

	if (js_data->oled_reset_gpio >= 0) {
		ret = __instantiate_oled(mdev, js_data);
		if (ret < 0) {
			dev_err(&mdev->dev,
				"error instantiating oled device\n");
			return ret;
		}
	}

	hid = hid_allocate_device();
	if (IS_ERR(hid)) {
		dev_err(&mdev->dev, "error allocating hid device\n");
		return PTR_ERR(hid);
	}

	js_data->hid = hid;
	dev_set_drvdata(&mdev->dev, js_data);

	hid->driver_data = mdev;
	hid->ll_driver = &mcuio_js_hid_ll_driver;
	hid->hid_get_raw_report = mcuio_js_hid_get_raw_report;
	hid->hid_output_raw_report = mcuio_js_hid_output_raw_report;
	hid->dev.parent = &mdev->dev;
	hid->bus = BUS_VIRTUAL;
	hid->version = 0;
	hid->vendor = MCUIO_VENDOR_DOGHUNTER;
	hid->product = mdev->device;

	snprintf(hid->name, sizeof(hid->name), "mcuio-js-shield %04hX:%04hX",
		 hid->vendor, hid->product);

	ret = hid_add_device(hid);
	if (ret) {
		if (ret != -ENODEV)
			hid_err(mdev, "can't add hid device: %d\n", ret);
		hid_destroy_device(hid);
	}
	dev_dbg(&mdev->dev, "%s returns ok\n", __func__);

	return 0;
}

static int mcuio_js_remove(struct mcuio_device *mdev)
{
	struct mcuio_js_gpio *gpio;
	struct mcuio_js_data *js_data = dev_get_drvdata(&mdev->dev);
	if (!js_data) {
		WARN_ON(1);
		return -ENODEV;
	}
	list_for_each_entry(gpio, &js_data->gpios, list) {
		if (!gpio->led)
			continue;
		led_classdev_unregister(&gpio->led->led);
	}

	i2c_unregister_device(js_data->oled_i2c_client);

	hid_destroy_device(js_data->hid);
	return 0;
}

static const struct mcuio_device_id js_drv_ids[] = {
	{
		.vendor = MCUIO_VENDOR_DOGHUNTER,
		.device = MCUIO_DEVICE_JOYSTICK_SHIELD,
	},
	/* Terminator */
	{
		.device = MCUIO_NO_DEVICE,
		.class = MCUIO_CLASS_UNDEFINED,
	},
};

static struct mcuio_driver mcuio_js_driver = {
	.driver = {
		.name = "mcuio-js-shield",
	},
	.id_table = js_drv_ids,
	.probe = mcuio_js_probe,
	.remove = mcuio_js_remove,
};

static int __init mcuio_js_init(void)
{
	return mcuio_driver_register(&mcuio_js_driver, THIS_MODULE);
}

static void __exit mcuio_js_exit(void)
{
	return mcuio_driver_unregister(&mcuio_js_driver);
}

subsys_initcall(mcuio_js_init);
module_exit(mcuio_js_exit);

MODULE_AUTHOR("Davide Ciminaghi");
MODULE_DESCRIPTION("MCUIO driver for joystick shield");
MODULE_LICENSE("GPL v2");
