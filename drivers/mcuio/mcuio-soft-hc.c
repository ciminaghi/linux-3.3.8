/*
 * Copyright 2011 Dog Hunter SA
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * GNU GPLv2 or later
 */

/* mcuio generic soft host controller functions */

#include <linux/mcuio.h>
#include <linux/circ_buf.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/circ_buf.h>
#include <linux/mcuio_ids.h>

#include <linux/mcuio.h>
#include <linux/mcuio-proto.h>
#include <linux/mcuio-hc.h>
#include <linux/mcuio-soft-hc.h>
#include "mcuio-internal.h"

static struct notifier_block device_nb;

static bool mcuio_soft_hc_readable(struct device *dev, unsigned int reg)
{
	return true;
}

static bool mcuio_soft_hc_writeable(struct device *dev, unsigned int reg)
{
	return (reg >= MCUIO_HC_OUTBUF && reg < MCUIO_HC_INBUF) ||
		reg == MCUIO_IRQ_CLR;
	return true;
}

/*
 * regmap config for line discipline based mcuio host controller
 */
static struct regmap_config proto = {
	.name = "mcuio-ldisc",
	.reg_bits = 8,
	.val_bits = 32,
	.max_register = MCUIO_HC_MAX_REGISTER,
	.readable_reg = mcuio_soft_hc_readable,
	.writeable_reg = mcuio_soft_hc_writeable,
	.cache_type = REGCACHE_NONE,
};

static int mcuio_soft_hc_read_inbuf(struct mcuio_soft_hc *shc,
			       unsigned int reg,
			       unsigned int *val)
{
	int i, s = sizeof(shc->rx_buf);
	u8 *out = (u8 *)val;
	struct circ_buf *buf = &shc->rx_circ_buf;

	if (CIRC_CNT(buf->head, buf->tail, s) < sizeof(u32))
		return -EAGAIN;
	for (i = 0; i < sizeof(u32); i++) {
		out[i] = buf->buf[buf->tail++];
		buf->tail &= (s - 1);
	}
	return sizeof(unsigned int);
}

static int mcuio_soft_hc_reg_read(void *context, unsigned int reg,
				  unsigned int *val)
{
	struct mcuio_soft_hc *shc = context;
	if (!shc)
		return -EINVAL;
	if (reg >= MCUIO_HC_INBUF && reg < MCUIO_RX_CNT)
		return mcuio_soft_hc_read_inbuf(shc, reg, val);
	switch(reg) {
	case MCUIO_RX_CNT:
	{
		struct circ_buf *buf = &shc->rx_circ_buf;
		*val = CIRC_CNT(buf->head, buf->tail, sizeof(shc->rx_buf));
		return sizeof(*val);
	}
	case MCUIO_IRQ:
		*val = shc->irqno;
		return sizeof(*val);
	case MCUIO_IRQ_STAT:
	{
		struct circ_buf *buf = &shc->rx_circ_buf;
		unsigned int new_irqstat;

		*val = shc->irqstat;

		pr_debug("%s: autoclear shc irqstat\n", __func__);
		new_irqstat = CIRC_CNT(buf->head, buf->tail,
				       sizeof(shc->rx_buf)) ? RX_RDY : 0;
		shc->irqstat = new_irqstat;
		pr_debug("%s: new shc irqstat = 0x%08x\n", __func__,
			shc->irqstat);
		return sizeof(*val);
	}
	default:
		return -EPERM;
	}
	/* NEVER REACHED */
	return -EPERM;
}

static int mcuio_soft_hc_reg_write(void *context,
				   unsigned int reg, unsigned int val)
{
	struct mcuio_soft_hc *shc = context;
	u8 *out = (u8 *)&val;
	if (!shc)
		return -EINVAL;
	if (reg >= MCUIO_HC_OUTBUF && reg < MCUIO_HC_INBUF)
		return shc->ops->write(shc, out, sizeof(val));
	if (reg == MCUIO_IRQ_CLR) {
		shc->irqstat &= ~val;
		return 0;
	}
	return -EPERM;
}

int mcuio_soft_hc_push_chars(struct mcuio_soft_hc *shc, const u8 *in, int len)
{
	int s = sizeof(shc->rx_buf), available, actual;
	struct circ_buf *buf = &shc->rx_circ_buf;
	available = CIRC_SPACE_TO_END(buf->head, buf->tail, s);
	if (available < sizeof(u32)) {
		pr_debug("%s %d\n", __func__, __LINE__);
		return -EAGAIN;
	}
	actual = min(len, available);
	memcpy(&buf->buf[buf->head], in, actual);
	buf->head = (buf->head + actual) & (s - 1);
	/* set irq status register RX_RDY bit */
	shc->irqstat |= RX_RDY;
	if (shc->irq_enabled)
		queue_kthread_work(&shc->irq_kworker, &shc->do_irq);
	return actual;
}
EXPORT_SYMBOL(mcuio_soft_hc_push_chars);

static struct regmap_config *mcuio_soft_hc_setup_regmap_config(void)
{
	struct regmap_config *out = kzalloc(sizeof(*out), GFP_KERNEL);
	if (!out)
		return out;
	*out = proto;
	out->reg_read = mcuio_soft_hc_reg_read;
	out->reg_write = mcuio_soft_hc_reg_write;
	return out;
}

static struct regmap *
mcuio_soft_hc_setup_regmap(struct device *dev,
			   void *__plat)
{
	struct mcuio_hc_platform_data *plat = __plat;
	struct regmap_config *map_cfg = mcuio_soft_hc_setup_regmap_config();
	struct mcuio_soft_hc *shc;
	struct regmap *out = ERR_PTR(-ENOMEM);
	if (!map_cfg) {
		dev_err(dev, "%s: cannot setup regmap config\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	shc = plat->data;
	if (!shc) {
		dev_err(dev, "%s: no platform data\n", __func__);
		return out;
	}
	/*
	  no_bus regmap with reg_read and reg_write, use soft controller
	  structure as regmap context
	*/
	return regmap_init(dev, NULL, shc, map_cfg);
}

static void mcuio_soft_hc_irq_mask(struct irq_data *d)
{
	struct irq_chip *chip = irq_data_get_irq_chip(d);
	struct mcuio_soft_hc *shc =
		container_of(chip, struct mcuio_soft_hc, chip);

	shc->irq_enabled = 0;
}

static void mcuio_soft_hc_irq_unmask(struct irq_data *d)
{
	struct irq_chip *chip = irq_data_get_irq_chip(d);
	struct mcuio_soft_hc *shc =
		container_of(chip, struct mcuio_soft_hc, chip);

	shc->irq_enabled = 1;
}

static void __do_irq(struct kthread_work *work)
{
	struct mcuio_soft_hc *shc =
		container_of(work, struct mcuio_soft_hc, do_irq);

	handle_nested_irq(shc->irqno);
}

static struct mcuio_soft_hc *__setup_shc(const struct mcuio_soft_hc_ops *ops,
					 void *priv)
{
	struct mcuio_soft_hc *shc = kzalloc(sizeof(*shc), GFP_KERNEL);
	if (!shc)
		return ERR_PTR(-ENOMEM);
	init_kthread_worker(&shc->irq_kworker);
	shc->irq_kworker_task = kthread_run(kthread_worker_fn,
					    &shc->irq_kworker,
					    "shc_irq");
	if (IS_ERR(shc->irq_kworker_task)) {
		pr_err("failed to create irq tsk for shc\n");
		return ERR_PTR(PTR_ERR(shc->irq_kworker_task));
	}
	init_kthread_work(&shc->do_irq, __do_irq);
	shc->ops = ops;
	shc->priv = priv;
	shc->rx_circ_buf.head = shc->rx_circ_buf.tail = 0;
	shc->rx_circ_buf.buf = shc->rx_buf;
	shc->chip.name = "MCUIO-SHC";
	shc->chip.irq_mask = mcuio_soft_hc_irq_mask;
	shc->chip.irq_unmask = mcuio_soft_hc_irq_unmask;
	shc->irqno = irq_alloc_desc(0);
	irq_set_chip(shc->irqno, &shc->chip);
	irq_set_handler(shc->irqno, &handle_simple_irq);
	irq_modify_status(shc->irqno,
			  IRQ_NOREQUEST | IRQ_NOAUTOEN,
			  IRQ_NOPROBE);
	return shc;
}

static struct mcuio_device_id default_soft_hc_id = {
	.device = MCUIO_DEVICE_SOFT_HC,
	.vendor = MCUIO_VENDOR_DOGHUNTER,
	.class = MCUIO_CLASS_SOFT_HOST_CONTROLLER,
};

static void mcuio_soft_hc_release(struct device *device)
{
	struct mcuio_hc_platform_data *plat = dev_get_platdata(device);
	struct mcuio_soft_hc *shc;
	int i;
	if (!plat) {
		WARN_ON(1);
		return;
	}
	shc = plat->data;
	bus_unregister_notifier(&mcuio_bus_type, &device_nb);
	/* Unregister all irq controllers */
	for (i = 0; i < MCUIO_DEVS_PER_BUS; i++)
		if (shc->irq_controllers[i])
			mcuio_device_unregister(shc->irq_controllers[i]);
	irq_set_handler(shc->irqno, NULL);
	irq_set_chip(shc->irqno, NULL);
	irq_free_desc(shc->irqno);
	kfree(shc);
	mcuio_hc_dev_default_release(device);
}

static void __device_added(struct device *dev)
{
	struct mcuio_device *mdev = to_mcuio_dev(dev);
	struct mcuio_device *hc;
	struct mcuio_soft_hc *shc;
	struct mcuio_device *ic;
	struct mcuio_hc_platform_data *plat;
	int base_irq;

	/* Ignore the hc */
	if (!mdev->device)
		return;
	hc = to_mcuio_dev(dev->parent);
	plat = dev_get_platdata(&hc->dev);
	if (!plat) {
		WARN_ON(1);
		return;
	}
	shc = plat->data;
	if (!shc) {
		WARN_ON(1);
		return;
	}
	/* FIXME: ADD LOCKING */
	ic = shc->irq_controllers[mdev->device];
	if (ic)
		return;
	base_irq = irq_alloc_descs(-1, 0, MCUIO_FUNCS_PER_DEV, 0);
	/* New device, add soft local irq controller */
	ic = mcuio_add_soft_local_irq_ctrl(hc, mdev->device, base_irq);
	if (!ic) {
		pr_err("mcuio soft hc: error adding irq ctrl for dev %d\n",
		       mdev->device);
		return;
	}
	shc->irq_controllers[mdev->device] = ic;
	/*
	  This is the first function of the new device. When the corresponding
	  mcuio_device was instantiated, the hc had no irqs, fix the field
	  up now
	*/
	mdev->irq = base_irq + mdev->fn;
}

static int mcuio_add_notifier(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	struct device *dev = data;

	switch (action) {
	case BUS_NOTIFY_ADD_DEVICE:
		__device_added(dev);
		break;
	default:
		return NOTIFY_DONE;
	}
	return NOTIFY_OK;
}

static struct notifier_block device_nb = {
	.notifier_call = mcuio_add_notifier,
};

struct device *mcuio_add_soft_hc(struct mcuio_device_id *id,
				 const struct mcuio_soft_hc_ops *ops,
				 void *priv)
{
	struct mcuio_hc_platform_data *plat;
	struct mcuio_soft_hc *shc = __setup_shc(ops, priv);
	struct device *out;
	int stat;
	if (IS_ERR(shc))
		return ERR_PTR(PTR_ERR(shc));
	plat = kzalloc(sizeof(*plat), GFP_KERNEL);
	if (!plat) {
		kfree(shc);
		return ERR_PTR(-ENOMEM);
	}
	plat->setup_regmap = mcuio_soft_hc_setup_regmap;
	plat->data = shc;

	stat = bus_register_notifier(&mcuio_bus_type, &device_nb);
	if (stat < 0) {
		kfree(shc);
		return ERR_PTR(stat);
	}

	out = mcuio_add_hc_device(id ? id : &default_soft_hc_id, plat,
				  mcuio_soft_hc_release);
	if (IS_ERR(out)) {
		kfree(shc);
		bus_unregister_notifier(&mcuio_bus_type, &device_nb);
		return out;
	}
	shc->hc = to_mcuio_dev(out);
	return out;
}
EXPORT_SYMBOL(mcuio_add_soft_hc);

MODULE_AUTHOR("Davide Ciminaghi");
MODULE_DESCRIPTION("MCUIO soft host controller code");
MODULE_LICENSE("GPL v2");
