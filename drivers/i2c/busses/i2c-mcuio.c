/*
 * mcuio I2c controller driver
 * Some code from drivers/i2c/busses/i2c-nomadik
 * smbus xfer comes from drivers/i2c/busses/i2c-stub.c
 */
/* #define DEBUG 1 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/circ_buf.h>

#include <linux/mcuio.h>
#include <linux/mcuio_ids.h>
#include <linux/mcuio-proto.h>

#define I2C_MCUIO_BUF_MAX_SIZE 0x100
#define I2C_MCUIO_IBUF_MAX_SIZE I2C_MCUIO_BUF_MAX_SIZE
#define I2C_MCUIO_OBUF_MAX_SIZE I2C_MCUIO_BUF_MAX_SIZE

#define LOCAL_BUF_SIZE 2048

/*
 * Xfer data register (xfers are limited to 255 bytes)
 *
 * Byte  0   -> slave address
 * Byte  1   -> obuf length
 * Byte  2   -> ibuf length
 */
#define I2C_MCUIO_XFER_DATA	0x008
#define I2C_REG_START		I2C_XFER_DATA

/*
 * Status register:
 *
 * Byte 0 -> flags
 * Byte 1 -> ibuf count
 * Byte 2 -> obuf space
 */
#define I2C_MCUIO_STATUS	0x00c
#define	  TRANSACTION_OK	  0x1
#define	  OBUF_LO_WM_REACHED	  0x02
#define	  IBUF_HI_WM_REACHED	  0x04
#define	  NAK_RECEIVED		  0x80
#define	  INVALID_LEN		  0x81
#define	  BUSY			  0x82

#define I2C_MCUIO_CFG		0x010
#define I2C_MCUIO_BRATE		0x014
#define I2C_MCUIO_CMD		0x018
#define	  START_TRANSACTION	  0x1
#define	  INTEN			  0x2
#define I2C_MCUIO_BUF_SIZE	0x020
#define I2C_REG_END		(I2C_MCUIO_IBUF + I2C_MCUIO_IBUF_MAX_SIZE)

#define I2C_MCUIO_OBUF		0x040
#define I2C_MCUIO_IBUF		(I2C_MCUIO_OBUF + I2C_MCUIO_OBUF_MAX_SIZE)

struct mcuio_i2c_dev {
	struct i2c_adapter adapter;
	struct mcuio_device *mdev;
	struct regmap *map_dw;
	struct regmap *map_b;
	struct completion xfer_complete;
	int xfer_status;
	unsigned buf_size;
	int olen;
	int ilen;
	unsigned short flags;
	unsigned short addr;
	int sent;
	int received;
	u8 buf[LOCAL_BUF_SIZE];
};

static inline u8 __flags(u32 v)
{
	return v & 0xff;
}

static inline u8 __space(u32 v)
{
	return (v >> 16) & 0xff;
}

static inline u8 __count(u32 v)
{
	return (v >> 8) & 0xff;
}

static inline int __get_status(struct mcuio_i2c_dev *i2cd, u32 *s)
{
	return regmap_read(i2cd->map_dw, I2C_MCUIO_STATUS, s);
}

static inline int __get_space(struct mcuio_i2c_dev *i2cd)
{
	int stat;
	u32 s;

	stat = __get_status(i2cd, &s);
	if (stat < 0)
		return stat;
	return __space(s);
}

static inline int __get_count(struct mcuio_i2c_dev *i2cd)
{
	int stat;
	u32 s;

	stat = __get_status(i2cd, &s);
	if (stat < 0)
		return stat;
	return __count(s);
}

static int ___send_obuf(struct mcuio_i2c_dev *i2cd, int space)
{
	int togo, l, stat;

	togo = i2cd->olen - i2cd->sent;
	/*
	 * Make size an integer multiple of the number of data bytes in
	 * a single wrb mcuio packet (with fill flag set).
	 * This is to reduce the mcuio overhead (try sending as few single
	 * byte packets as possible).
	 */
	if (space > 8)
		space = (space >> 3) << 3;
	l = min(space, togo);
	pr_debug("%s: space = %d, togo = %d, l = %d, sent = %d\n",
		 __func__, space, togo, l, i2cd->sent);
	stat = regmap_raw_write(i2cd->map_b, I2C_MCUIO_OBUF,
				&i2cd->buf[i2cd->sent], l);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev, "error sending output buffer\n");
		return stat;
	}
	i2cd->sent += l;
	return stat;
}

static int __send_obuf(struct mcuio_i2c_dev *i2cd)
{
	int space;

	space = __get_space(i2cd);
	if (space < 0) {
		dev_err(&i2cd->mdev->dev, "error reading output space\n");
		return space;
	}
	return ___send_obuf(i2cd, space);
}

static int ___get_ibuf(struct mcuio_i2c_dev *i2cd, int count)
{
	int togo, l, stat;

	togo = i2cd->ilen - i2cd->received;
	l = min(count, togo);
	pr_debug("%s: count = %d, togo = %d, l = %d, received = %d\n",
		 __func__, count, togo, l, i2cd->received);
	stat = regmap_raw_read(i2cd->map_b, I2C_MCUIO_IBUF,
			       &i2cd->buf[i2cd->received], l);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev, "error reading input buffer\n");
		return stat;
	}
	i2cd->received += l;
	return stat;
}

static int __get_ibuf(struct mcuio_i2c_dev *i2cd)
{
	int count;

	count = __get_count(i2cd);
	if (count < 0) {
		dev_dbg(&i2cd->mdev->dev, "error reading input count\n");
		return count;
	}
	if (!count) {
		dev_dbg(&i2cd->mdev->dev, "nothing in input buffer\n");
		return count;
	}
	return ___get_ibuf(i2cd, count);
}

static irqreturn_t mcuio_i2c_irq_handler(int irq, void *devid)
{
	/* Read status register and complete current transaction */
	struct mcuio_i2c_dev *i2cd = devid;
	u32 v;
	u8 flags, count, space;
	int stat;

	BUG_ON(!i2cd || !i2cd->map_dw);
	stat = __get_status(i2cd, &v);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev, "error reading i2c status\n");
		/* This will make the transaction end with -EIO */
		v = NAK_RECEIVED;
	}
	dev_dbg(&i2cd->mdev->dev, "%s: status = 0x%08x\n", __func__, v);

	flags = __flags(v);
	count = __count(v);
	space = __space(v);

	if (!flags) {
		dev_err(&i2cd->mdev->dev, "spurious irq\n");
		return IRQ_HANDLED;
	}

	if ((flags & NAK_RECEIVED) || (flags & TRANSACTION_OK)) {
		i2cd->xfer_status = (stat < 0 || (v & NAK_RECEIVED)) ? -EIO : 0;
		dev_dbg(&i2cd->mdev->dev, "%s: flags = 0x%04x\n", __func__,
			i2cd->flags);
		if ((i2cd->flags & I2C_M_RD) && !i2cd->xfer_status) {
			dev_dbg(&i2cd->mdev->dev, "%s: reading ibuf\n",
				__func__);
			__get_ibuf(i2cd);
		}
		complete(&i2cd->xfer_complete);
		return IRQ_HANDLED;
	}
	if ((flags & OBUF_LO_WM_REACHED) && space && (i2cd->olen - i2cd->sent))
		___send_obuf(i2cd, space);
	if ((flags & IBUF_HI_WM_REACHED) && count &&
	    (i2cd->ilen - i2cd->received))
		___get_ibuf(i2cd, count);
	dev_dbg(&i2cd->mdev->dev, "%s: xfer_status = %d\n", __func__,
		i2cd->xfer_status);
	return IRQ_HANDLED;
}

#ifdef DEBUG
static void __dump_message(struct device *dev, struct i2c_msg *msg)
{
	int i;
	dev_dbg(dev, "i2c msg len = %u\n", msg->len);
	printk(KERN_DEBUG);
	for (i = 0; i < msg->len; i++) {
		printk("0x%02x\t", msg->buf[i]);
		if (i && !(i % 8)) {
			printk("\n");
			printk(KERN_DEBUG);
		}
	}
	printk("\n");
}
#else
static inline void __dump_message(struct device *dev, struct i2c_msg *msg)
{
}
#endif

static int __do_xfer(struct i2c_adapter * a)
{
	u32 v;
	int ret, timeout;
	struct mcuio_i2c_dev *i2cd = i2c_get_adapdata(a);

	/* Set slave address, ilength, olength */
	v = (((u32)i2cd->addr) << 1) | (((u32)i2cd->olen) << 8) |
		(((u32)i2cd->ilen) << 20);
	dev_dbg(&i2cd->mdev->dev, "setting xfer data 0x%08x\n", v);
	ret = regmap_write(i2cd->map_dw, I2C_MCUIO_XFER_DATA, v);
	if (ret < 0) {
		dev_err(&i2cd->mdev->dev, "error setting xfer data\n");
		return ret;
	}

	/* Write message to buffer */
	/* FIXME: check whether regmap_raw_write works */
	i2cd->received = 0;
	i2cd->sent = 0;
	if (i2cd->olen) {
		ret = __send_obuf(i2cd);
		if (ret < 0) {
			dev_err(&i2cd->mdev->dev,
				"error sending output buffer\n");
			return ret;
		}
	}

	/* Initialize xfer status and init xfer completion struct */
	i2cd->xfer_status = -ETIMEDOUT;
	init_completion(&i2cd->xfer_complete);

	/* Issue xmit command and enable interrupt */
	v = START_TRANSACTION | INTEN;
	dev_dbg(&i2cd->mdev->dev, "starting transaction\n");
	ret = regmap_write(i2cd->map_dw, I2C_MCUIO_CMD, v);
	if (ret < 0) {
		dev_err(&i2cd->mdev->dev, "error starting transaction\n");
		return ret;
	}

	/* Wait for transfer complete */
	timeout = wait_for_completion_timeout(&i2cd->xfer_complete,
					      a->timeout);
	if (timeout < 0) {
		dev_err(&i2cd->mdev->dev, "error %d on slave xfer\n", timeout);
		ret = timeout;
	}
	if (!timeout)
		/*
		 * In case of timeout, the initial value of i2cd->xfer_status
		 * will be returned
		 */
		dev_err(&i2cd->mdev->dev, "timeout on slave xfer\n");
	else
		dev_dbg(&i2cd->mdev->dev, "transaction done\n");
	return ret < 0 ? ret : i2cd->xfer_status;
}

static int mcuio_simple_i2c_xfer_one(struct i2c_adapter *a,
				     struct i2c_msg *msg)
{
	struct mcuio_i2c_dev *i2cd = i2c_get_adapdata(a);
	int ret;

	if (!i2cd) {
		WARN_ON(1);
		return -ENODEV;
	}

	if (msg->len > LOCAL_BUF_SIZE) {
		dev_dbg(&i2cd->mdev->dev, "i2c message is too long\n");
		return -EINVAL;
	}

	__dump_message(&i2cd->mdev->dev, msg);

	i2cd->olen = i2cd->ilen = 0;
	i2cd->flags = msg->flags;
	i2cd->addr = msg->addr;

	if (msg->flags & I2C_M_RD)
		i2cd->ilen = msg->len;
	else
		i2cd->olen = msg->len;

	if (i2cd->olen)
		memcpy(i2cd->buf, msg->buf, i2cd->olen);

	ret = __do_xfer(a);
	if (ret < 0 || !i2cd->ilen)
		return ret;

	memcpy(msg->buf, i2cd->buf, i2cd->ilen);
	return ret;
}

static int mcuio_simple_i2c_xfer(struct i2c_adapter *a,
				 struct i2c_msg msgs[], int num_msgs)
{
	int i, ret = 0;

	for (i = 0; i < num_msgs && !ret; i++)
		ret = mcuio_simple_i2c_xfer_one(a, &msgs[i]);
	return ret < 0 ? ret : num_msgs;
}

/* Return negative errno on error. */
static s32 mcuio_simple_smbus_xfer(struct i2c_adapter * adap, u16 addr,
				   unsigned short flags, char read_write,
				   u8 command, int size,
				   union i2c_smbus_data * data)
{
	struct mcuio_i2c_dev *i2cd = i2c_get_adapdata(adap);
	s32 ret;
	u32 ilen = 0, olen = 0;
	int len;

	if (!i2cd) {
		WARN_ON(1);
		return -ENODEV;
	}

	switch (size) {

	case I2C_SMBUS_QUICK:
		dev_dbg(&adap->dev, "smbus quick - addr 0x%02x\n", addr);
		ret = 0;
		break;

	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_WRITE) {
			dev_dbg(&adap->dev, "smbus byte - wr addr 0x%02x, "
					"write 0x%02x.\n",
					addr, command);
			olen = 1;
			i2cd->buf[0] = command;
		} else {
			dev_dbg(&adap->dev, "smbus byte - rd addr 0x%02x\n",
				addr);
			ilen = 1;
		}
		ret = 0;
		break;

	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			olen = 2;
			i2cd->buf[0] = command;
			i2cd->buf[1] = data->byte;
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"write 0x%02x at 0x%02x.\n",
					addr, data->byte, command);
		} else {
			olen = 1;
			ilen = 1;
			i2cd->buf[0] = command;
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"read at 0x%02x.\n",
					addr, command);
		}
		ret = 0;
		break;

	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			olen = 3;
			i2cd->buf[0] = command;
			i2cd->buf[1] = data->word & (u16)0x00ff;
			i2cd->buf[2] = data->word >> 8;
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"write 0x%04x at 0x%02x.\n",
					addr, data->word, command);
		} else {
			i2cd->buf[0] = command;
			olen = 1;
			i2cd->buf[0] = command;
			ilen = 2;
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"read at 0x%02x.\n",
					addr, command);
		}

		ret = 0;
		break;

	case I2C_SMBUS_I2C_BLOCK_DATA:
		len = data->block[0];
		/* smbus limit */
		if (len < 0 || len > 32)
			return -EINVAL;
		i2cd->buf[0] = command;
		if (read_write == I2C_SMBUS_WRITE) {
			olen = 1 + len;
			memcpy(&i2cd->buf[1], &data->block[1], len);
		} else {
			olen = 1;
			ilen = len;
		}
		ret = 0;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			len = data->block[0];
			/* smbus limit */
			if (len < 0 || len > 32)
				return -EINVAL;
			/*
			 * FIXME: CHECK mcuio DEVICE BUFFER SIZE, disable this
			 * if bufsize < 32
			 */
			olen = 1 + len;
			i2cd->buf[0] = command;
			memcpy(&i2cd->buf[1], &data->block[1], len);
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"write %d bytes at 0x%02x.\n",
					addr, len, command);
		} else {
			olen = 1;
			/* The device tells how long the block shall be */
			ilen = -1;
			i2cd->buf[0] = command;
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"read  ? bytes at 0x%02x.\n",
					addr, command);
		}

		ret = 0;
		break;

	default:
		dev_dbg(&adap->dev, "Unsupported I2C/SMBus command 0x%08x\n",
			size);
		ret = -EOPNOTSUPP;
		break;
	} /* switch (size) */

	if (ret < 0)
		return ret;

	i2cd->olen = olen;
	i2cd->ilen = ilen;
	i2cd->flags = read_write == I2C_SMBUS_READ ? I2C_M_RD : 0;
	i2cd->addr = addr;

	ret = __do_xfer(adap);

	if (ret < 0 || read_write == I2C_SMBUS_WRITE)
		return ret;

	switch (size) {
	case I2C_SMBUS_WORD_DATA:
		data->word = (i2cd->buf[1] << 8) | (i2cd->buf[0]);
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		data->block[0] = i2cd->ilen;
		memcpy(&data->block[1], i2cd->buf, i2cd->ilen);
		break;
	default:
		memcpy(&data->byte, i2cd->buf, i2cd->ilen);
	}
	return ret;
}


static unsigned int mcuio_simple_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_BLOCK_DATA;
}

static const struct i2c_algorithm mcuio_simple_i2c_algo = {
	.master_xfer	= mcuio_simple_i2c_xfer,
	.smbus_xfer	= mcuio_simple_smbus_xfer,
	.functionality	= mcuio_simple_i2c_functionality
};

static const struct regmap_config mcuio_i2c_regmap_config_dw = {
	.reg_bits = 32,
	.val_bits = 32,
	.max_register = 0x240,
	.cache_type = REGCACHE_NONE,
	.name = "mcuio-i2c-dw",
};

static const struct regmap_config mcuio_i2c_regmap_config_b = {
	.reg_bits = 32,
	.val_bits = 8,
	.max_register = 0x240,
	.cache_type = REGCACHE_NONE,
	.name = "mcuio=i2c-b",
};

static int mcuio_simple_i2c_probe(struct mcuio_device *mdev)
{
	int ret;
	struct mcuio_i2c_dev *i2cd;
	struct i2c_adapter *a;
	u32 v;

	i2cd = devm_kzalloc(&mdev->dev, sizeof(*i2cd), GFP_KERNEL);
	if (!i2cd)
		return -ENOMEM;
	i2cd->mdev = mdev;
	i2cd->map_dw = devm_regmap_init_mcuio(mdev,
					      &mcuio_i2c_regmap_config_dw);
	if (IS_ERR(i2cd->map_dw)) {
		dev_err(&mdev->dev, "cannot setup regmap (dw) for device\n");
		return PTR_ERR(i2cd->map_dw);
	}
	i2cd->map_b = devm_regmap_init_mcuio(mdev,
					     &mcuio_i2c_regmap_config_b);
	if (IS_ERR(i2cd->map_b)) {
		dev_err(&mdev->dev, "cannot setup regmap (b) for device\n");
		return PTR_ERR(i2cd->map_b);
	}

	ret = regmap_read(i2cd->map_dw, I2C_MCUIO_BUF_SIZE, &v);
	if (ret < 0) {
		dev_err(&mdev->dev, "error reading mcu buffer size\n");
		return ret;
	}
	i2cd->buf_size = v;

	a = &i2cd->adapter;

	a->dev.parent	= &mdev->dev;
	a->owner	= THIS_MODULE;
	a->class	= I2C_CLASS_HWMON | I2C_CLASS_SPD;
	a->algo		= &mcuio_simple_i2c_algo;
	/* FIXME: read timeout from mcu ? */
	a->timeout	= msecs_to_jiffies(20000);
	snprintf(a->name, sizeof(a->name),
		 "Mcuio I2C %s", dev_name(&mdev->dev));

	ret = request_threaded_irq(mdev->irq, NULL, mcuio_i2c_irq_handler,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   "mcuio-i2c", i2cd);
	if (ret < 0) {
		dev_err(&mdev->dev, "failed requesting irq\n");
		return ret;
	}

	i2c_set_adapdata(a, i2cd);

	ret = i2c_add_adapter(a);
	if (ret) {
		dev_err(&mdev->dev, "failed to add adapter\n");
		free_irq(mdev->irq, i2cd);
		return ret;
	}
	dev_set_drvdata(&mdev->dev, i2cd);
	return ret;
}

static int mcuio_simple_i2c_remove(struct mcuio_device *mdev)
{
	struct mcuio_i2c_dev *i2cd = dev_get_drvdata(&mdev->dev);
	i2c_del_adapter(&i2cd->adapter);
	free_irq(mdev->irq, i2cd);
	return 0;
}

static const struct mcuio_device_id mcuio_simple_i2c_drv_ids[] = {
	{
		.class = MCUIO_CLASS_I2C_CONTROLLER,
		.class_mask = 0xffff,
	},
	/* Terminator */
	{
		.device = MCUIO_NO_DEVICE,
		.class = MCUIO_CLASS_UNDEFINED,
	},
};


static struct mcuio_driver mcuio_simple_i2c_driver = {
	.driver = {
		.name = "mcuio-simple-i2c",
	},
	.id_table = mcuio_simple_i2c_drv_ids,
	.probe = mcuio_simple_i2c_probe,
	.remove = mcuio_simple_i2c_remove,
};

static int __init mcuio_simple_i2c_init(void)
{
	return mcuio_driver_register(&mcuio_simple_i2c_driver, THIS_MODULE);
}

static void __exit mcuio_simple_i2c_exit(void)
{
	return mcuio_driver_unregister(&mcuio_simple_i2c_driver);
}

subsys_initcall(mcuio_simple_i2c_init);
module_exit(mcuio_simple_i2c_exit);

MODULE_AUTHOR("Davide Ciminaghi");
MODULE_DESCRIPTION("MCUIO simple i2c controller driver");
MODULE_LICENSE("GPL v2");
