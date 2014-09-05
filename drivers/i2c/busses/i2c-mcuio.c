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

#define I2C_MCUIO_SADDR	   0x008
#define I2C_MCUIO_STATUS   0x00c
#define	  TRANSACTION_OK       0x1
#define	  LOW_OBUF_WATERMARK   0x2
#define   HI_IBUF_WATERMARK    0x4
#define	  NAK_RECEIVED	       0x80
#define I2C_MCUIO_CFG	   0x010
#define I2C_MCUIO_BRATE	   0x014
#define I2C_MCUIO_CMD	   0x018
#define	  START_TRANSACTION 0x1

#define I2C_MCUIO_INTEN	   0x01c
#define I2C_MCUIO_BUF_SIZE 0x020
#define I2C_MCUIO_OBUF_LEN 0x024
#define I2C_MCUIO_IBUF_LEN 0x028
#define I2C_MCUIO_OBUF_HEAD 0x02c
#define I2C_MCUIO_OBUF_TAIL 0x030
#define I2C_MCUIO_IBUF_HEAD 0x034
#define I2C_MCUIO_IBUF_TAIL 0x038
#define I2C_MCUIO_OBUF	   0x040
#define I2C_MCUIO_IBUF	   (I2C_MCUIO_OBUF + I2C_MCUIO_OBUF_MAX_SIZE)

struct mcuio_i2c_dev {
	struct i2c_adapter adapter;
	struct mcuio_device *mdev;
	struct regmap *map_dw;
	struct regmap *map_w;
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
	union {
		u32 dw[LOCAL_BUF_SIZE/sizeof(u32)];
		u8 b[LOCAL_BUF_SIZE];
	} buf;
};

#ifdef DEBUG
static void __dump_ibuf(struct mcuio_i2c_dev *i2cd)
{
	int i;
	printk(KERN_DEBUG "input buffer\n");
	for (i = 0; i < i2cd->received; i++) {
		printk("0x%02x\t", i2cd->buf.b[i]);
		if (i && !(i % 8)) {
			printk("\n");
			printk(KERN_DEBUG);
		}
	}
	printk("\n");
}
#else
#define __dump_ibuf(a)
#endif

static int __send_obuf(struct mcuio_i2c_dev *i2cd)
{
	u32 h, t;
	int space, space_to_end;
	int stat, togo, l, l1, l2;

	stat = regmap_read(i2cd->map_dw, I2C_MCUIO_OBUF_HEAD, &h);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev, "error reading obuf head\n");
		return stat;
	}
	stat = regmap_read(i2cd->map_dw, I2C_MCUIO_OBUF_TAIL, &t);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev, "error reading obuf tail\n");
		return stat;
	}
	space = CIRC_SPACE(h, t, i2cd->buf_size);
	space_to_end = CIRC_SPACE_TO_END(h, t, i2cd->buf_size);
	dev_dbg(&i2cd->mdev->dev, "h = %u, t = %u, space = %d\n", h, t, space);
	if (!space) {
		dev_dbg(&i2cd->mdev->dev, "no space in output buffer\n");
		return 0;
	}
	togo = i2cd->olen - i2cd->sent;
	l = min(space, togo);
	l1 = min(l, space_to_end);
	l2 = l - space_to_end;
	dev_dbg(&i2cd->mdev->dev, "togo = %d, l = %d\n", togo, l);
	stat = regmap_raw_write(i2cd->map_b, I2C_MCUIO_OBUF + h,
				&i2cd->buf.b[i2cd->sent], l1);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev, "error sending obuf\n");
		return stat;
	}
	h = (h + l1) & (i2cd->buf_size - 1);
	i2cd->sent += l1;
	if (l2 > 0) {
		/* Wrap around */
		stat = regmap_raw_write(i2cd->map_b, I2C_MCUIO_OBUF + h,
					&i2cd->buf.b[i2cd->sent], l2);
		if (stat < 0) {
			dev_err(&i2cd->mdev->dev, "error sending obuf\n");
			return stat;
		}
		h = (h + l2) & (i2cd->buf_size - 1);
		i2cd->sent += l2;
	}
	dev_dbg(&i2cd->mdev->dev, "sent = %d\n", i2cd->sent);

	stat = regmap_write(i2cd->map_dw, I2C_MCUIO_OBUF_HEAD, h);
	if (stat < 0)
		dev_err(&i2cd->mdev->dev, "error setting new obuf head\n");
	return stat;
}

static int __get_ibuf(struct mcuio_i2c_dev *i2cd)
{
	u32 h, t;
	int count, count_to_end;
	int stat, togo, l, l1, l2;

	stat = regmap_read(i2cd->map_dw, I2C_MCUIO_IBUF_TAIL, &t);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev, "error reading ibuf tail\n");
		return stat;
	}
	stat = regmap_read(i2cd->map_dw, I2C_MCUIO_IBUF_HEAD, &h);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev,
			"error reading input buffer head\n");
		return stat;
	}
	count = CIRC_CNT(h, t, i2cd->buf_size);
	count_to_end = CIRC_CNT_TO_END(h, t, i2cd->buf_size);
	dev_dbg(&i2cd->mdev->dev, "h = %u, t = %u, count = %d\n", h, t, count);
	if (!count) {
		dev_dbg(&i2cd->mdev->dev, "nothing in input buffer\n");
		return 0;
	}
	if (i2cd->ilen == -1) {
		u8 v;
		/* SMBUS read block cmd, the device must tell us how many
		   bytes have to be read
		*/
		stat = regmap_raw_read(i2cd->map_b, I2C_MCUIO_IBUF + t,
				       &v, 1);
		if (stat < 0) {
			dev_err(&i2cd->mdev->dev, "error reading ibuf len\n");
			return stat;
		}
		i2cd->ilen = v;
		dev_dbg(&i2cd->mdev->dev, "smbus block, ilen = %d\n",
			i2cd->ilen);
	}
	togo = i2cd->ilen - i2cd->received;
	l = min(count, togo);
	l1 = min(l, count_to_end);
	l2 = l - count_to_end;
	dev_dbg(&i2cd->mdev->dev, "togo = %d, l = %d\n", togo, l);
	stat = regmap_raw_read(i2cd->map_b, I2C_MCUIO_IBUF + t,
			       &i2cd->buf.b[i2cd->received], l1);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev, "Error reading ibuf\n");
		return stat;
	}
	t = (t + l1) & (i2cd->buf_size - 1);
	i2cd->received += l1;
	if (l2 > 0) {
		/* Wrap around */
		stat = regmap_raw_read(i2cd->map_b, I2C_MCUIO_IBUF + t,
				       &i2cd->buf.b[i2cd->received], l2);
		if (stat < 0) {
			dev_err(&i2cd->mdev->dev, "Error reading ibuf\n");
			return stat;
		}
		t = (t + l2) & (i2cd->buf_size - 1);
		i2cd->received += l2;
	}
	__dump_ibuf(i2cd);
	dev_dbg(&i2cd->mdev->dev, "received = %d\n", i2cd->received);

	stat = regmap_write(i2cd->map_dw, I2C_MCUIO_IBUF_TAIL, t);
	if (stat < 0) {
		dev_err(&i2cd->mdev->dev,
			"error setting new ibuf tail\n");
		return stat;
	}
	return stat;
}

static irqreturn_t mcuio_i2c_irq_handler(int irq, void *devid)
{
	/* Read status register and complete current transaction */
	struct mcuio_i2c_dev *i2cd = devid;
	u32 v;
	int stat;

	BUG_ON(!i2cd || !i2cd->map_dw);
	stat = regmap_read(i2cd->map_dw, I2C_MCUIO_STATUS, &v);
	dev_dbg(&i2cd->mdev->dev, "%s: status = 0x%08x\n", __func__, v);
	if (v & LOW_OBUF_WATERMARK)
		__send_obuf(i2cd);
	if (v & HI_IBUF_WATERMARK)
		__get_ibuf(i2cd);
	if ((v & NAK_RECEIVED) || (v & TRANSACTION_OK)) {
		i2cd->xfer_status = (stat < 0 || (v & NAK_RECEIVED)) ? -EIO : 0;
		dev_dbg(&i2cd->mdev->dev, "%s: flags = 0x%04x\n", __func__,
			i2cd->flags);
		if (i2cd->flags & I2C_M_RD) {
			dev_dbg(&i2cd->mdev->dev, "%s: reading ibuf\n",
				__func__);
			__get_ibuf(i2cd);
		}
		complete(&i2cd->xfer_complete);
	}
	dev_dbg(&i2cd->mdev->dev, "%s: xfer_status = %d\n", __func__,
		i2cd->xfer_status);
	return IRQ_HANDLED;
}

static void __dump_message(struct device *dev, struct i2c_msg *msg)
{
	int i;
	dev_dbg(dev, "i2c msg len = %u\n", msg->len);
#ifdef DEBUG
	printk(KERN_DEBUG);
	for (i = 0; i < msg->len; i++) {
		printk("0x%02x\t", msg->buf[i]);
		if (i && !(i % 8)) {
			printk("\n");
			printk(KERN_DEBUG);
		}
	}
	printk("\n");
#endif
}

static int __do_xfer(struct i2c_adapter * a)
{
	u32 v;
	int ret, timeout;
	struct mcuio_i2c_dev *i2cd = i2c_get_adapdata(a);

	/* Set slave address + R/W */
	v = i2cd->addr << 1;
	dev_dbg(&i2cd->mdev->dev, "setting slave addr 0x%08x\n", v);
	ret = regmap_write(i2cd->map_dw, I2C_MCUIO_SADDR, v);
	if (ret < 0) {
		dev_err(&i2cd->mdev->dev, "error setting slave address\n");
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

	/* Write message length(s) */
	v = i2cd->olen;
	dev_dbg(&i2cd->mdev->dev, "writing msg olen = %u\n", v);
	ret = regmap_write(i2cd->map_dw, I2C_MCUIO_OBUF_LEN, v);
	if (ret < 0) {
		dev_err(&i2cd->mdev->dev, "error writing message out length\n");
		return ret;
	}
	v = i2cd->ilen;
	dev_dbg(&i2cd->mdev->dev, "writing msg ilen = %u\n", v);
	ret = regmap_write(i2cd->map_dw, I2C_MCUIO_IBUF_LEN, v);
	if (ret < 0) {
		dev_err(&i2cd->mdev->dev, "error writing message in length\n");
		return ret;
	}

	/* Enable interrupt */
	v = TRANSACTION_OK | NAK_RECEIVED | LOW_OBUF_WATERMARK;
	dev_dbg(&i2cd->mdev->dev, "enabling interrupt\n");
	ret = regmap_write(i2cd->map_dw, I2C_MCUIO_INTEN, v);
	if (ret < 0) {
		dev_err(&i2cd->mdev->dev, "error enabling interrupt\n");
		return ret;
	}

	/* Initialize xfer status and init xfer completion struct */
	i2cd->xfer_status = -ETIMEDOUT;
	init_completion(&i2cd->xfer_complete);

	/* Issue xmit command */
	v = START_TRANSACTION;
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
		memcpy(i2cd->buf.b, msg->buf, i2cd->olen);

	ret = __do_xfer(a);
	if (ret < 0 || !i2cd->ilen)
		return ret;

	memcpy(msg->buf, i2cd->buf.b, i2cd->ilen);
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
			i2cd->buf.b[0] = command;
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
			i2cd->buf.b[0] = command;
			i2cd->buf.b[1] = data->byte;
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"write 0x%02x at 0x%02x.\n",
					addr, data->byte, command);
		} else {
			olen = 1;
			ilen = 1;
			i2cd->buf.b[0] = command;
			dev_dbg(&adap->dev, "smbus byte data - addr 0x%02x, "
					"read at 0x%02x.\n",
					addr, command);
		}
		ret = 0;
		break;

	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			olen = 3;
			i2cd->buf.b[0] = command;
			i2cd->buf.b[1] = data->word & (u16)0x00ff;
			i2cd->buf.b[2] = data->word >> 8;
			dev_dbg(&adap->dev, "smbus word data - addr 0x%02x, "
					"write 0x%04x at 0x%02x.\n",
					addr, data->word, command);
		} else {
			i2cd->buf.b[0] = command;
			olen = 1;
			i2cd->buf.b[0] = command;
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
		i2cd->buf.b[0] = command;
		if (read_write == I2C_SMBUS_WRITE) {
			olen = 1 + len;
			memcpy(&i2cd->buf.b[1], &data->block[1], len);
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
			i2cd->buf.b[0] = command;
			memcpy(&i2cd->buf.b[1], &data->block[1], len);
			dev_dbg(&adap->dev, "i2c block data - addr 0x%02x, "
					"write %d bytes at 0x%02x.\n",
					addr, len, command);
		} else {
			olen = 1;
			/* The device tells how long the block shall be */
			ilen = -1;
			i2cd->buf.b[0] = command;
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
		data->word = (i2cd->buf.b[1] << 8) | (i2cd->buf.b[0]);
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		data->block[0] = i2cd->ilen;
		memcpy(&data->block[1], i2cd->buf.b, i2cd->ilen);
		break;
	default:
		memcpy(&data->byte, i2cd->buf.b, i2cd->ilen);
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
};

static const struct regmap_config mcuio_i2c_regmap_config_w = {
	.reg_bits = 32,
	.val_bits = 16,
	.max_register = 0x240,
	.cache_type = REGCACHE_NONE,
};

static const struct regmap_config mcuio_i2c_regmap_config_b = {
	.reg_bits = 32,
	.val_bits = 8,
	.max_register = 0x240,
	.cache_type = REGCACHE_NONE,
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
	i2cd->map_w = devm_regmap_init_mcuio(mdev,
					     &mcuio_i2c_regmap_config_w);
	if (IS_ERR(i2cd->map_w)) {
		dev_err(&mdev->dev, "cannot setup regmap (w) for device\n");
		return PTR_ERR(i2cd->map_w);
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
	free_irq(mdev->irq, i2cd);
	i2c_del_adapter(&i2cd->adapter);
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
