/*
 * Copyright 2011 Dog Hunter SA
 * Author: Davide Ciminaghi <ciminaghi@gnudd.com>
 *
 * GNU GPLv2 or later
 */
/* #define DEBUG */

/* MCUIO local msg irq controller device */
/*
 * This module implements a soft local irq controller device, which is
 * instantiated by the soft hc controller.
 * Actual interrupts are triggered by remote device write accesses to
 * this device.
 *
 *  +---------------------+          +----------------------+
 *  |MPU                  |          | MCU                  |
 *  |                     |          |                      |
 *  |          +------------------------+                   |
 *  |          V wr@0xXXX |          |  |                   |
 *  |+----------------+   |          |+--------------+      |
 *  || local irq ctrl |   |          ||irq ctrl msg  |      |
 *  |+----------------+   |          |+--------------+      |
 *  +---------------------+          +----------------------+
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/mcuio.h>
#include <linux/mcuio-hc.h>
#include <linux/mcuio_ids.h>
#include <linux/mcuio-proto.h>

#include "mcuio-internal.h"

/* Interrupt messages */
#define MCUIO_IRQ_TRIGGER 0xf80

/*
 * Data concerning irq of a single function
 *
 * @irq_kwork: kthread_worker for irq generation
 * @irq_kworker_task: irq generation task
 * @do_irq: kthread_work for irq generation
 */
struct fn_irqdata {
	struct kthread_worker irq_kworker;
	struct task_struct *irq_kworker_task;
	struct kthread_work do_irq;
	int irq;
};

/*
 * private data for message based soft local irq controller
 */
struct soft_local_irq_ctrl_msg {
	struct mcuio_device mdev;
	struct mcuio_request write_req;
	struct irq_chip chip;
	int base_irq;
	struct fn_irqdata irqdata[MCUIO_FUNCS_PER_DEV];
	atomic_t removing;
};

static const struct attribute_group *soft_local_irq_ctrl_dev_attr_groups[] = {
	&mcuio_default_dev_attr_group,
	NULL,
};

void mcuio_soft_local_irq_ctrl_release(struct device *dev)
{
	struct mcuio_device *mdev = to_mcuio_dev(dev);
	struct soft_local_irq_ctrl_msg *slicm =
		container_of(mdev, struct soft_local_irq_ctrl_msg, mdev);
	int i;

	pr_debug("%s entered\n", __func__);
	atomic_set(&slicm->removing, 1);
	pr_debug("canceling write req cb\n");
	mcuio_cancel_cb(&slicm->write_req);
	for (i = 0; i < MCUIO_FUNCS_PER_DEV; i++) {
		struct fn_irqdata *idata = &slicm->irqdata[i];

		pr_debug("stopping thread for device %d\n", i);
		flush_kthread_worker(&idata->irq_kworker);
		kthread_stop(idata->irq_kworker_task);
	}
	pr_debug("freeing irq descriptors\n");
	irq_free_descs(slicm->base_irq, MCUIO_FUNCS_PER_DEV);
	kfree(slicm);
}


static struct device_type mcuio_soft_local_irq_ctrl_device_type = {
	.name = "mcuio-soft-local-irq-ctrl",
	.groups = soft_local_irq_ctrl_dev_attr_groups,
	.release = mcuio_soft_local_irq_ctrl_release,
};

static void mcuio_soft_local_irq_controller_msg_wcb(struct mcuio_request *r)
{
	struct soft_local_irq_ctrl_msg *slicm = r->cb_data;
	struct fn_irqdata *idata;
	int fn;

	pr_debug("%s %d, offset = 0x%04x, data[0] = 0x%08x\n",
		 __func__, __LINE__, r->offset, r->data[0]);

	if (atomic_read(&slicm->removing))
		return;

	fn = (r->offset - MCUIO_IRQ_TRIGGER) / sizeof(u32);

	if ((fn < 0) || (fn >= MCUIO_FUNCS_PER_DEV)) {
		dev_err(&slicm->mdev.dev, "UNHANDLED WRITE REQ TO 0x%04x\n",
			r->offset);
		return;
	}

	idata = &slicm->irqdata[fn];

	queue_kthread_work(&idata->irq_kworker, &idata->do_irq);
}

static void mcuio_soft_local_irq_mask(struct irq_data *d)
{
}

static void mcuio_soft_local_irq_unmask(struct irq_data *d)
{
}

static void __do_irq(struct kthread_work *work)
{
	struct fn_irqdata *idata =
		container_of(work, struct fn_irqdata, do_irq);

	handle_nested_irq(idata->irq);
}

struct mcuio_device *
mcuio_add_soft_local_irq_ctrl(struct mcuio_device *hc, int fn, int base_irq)
{
	struct soft_local_irq_ctrl_msg *slicm = kzalloc(sizeof(*slicm),
							GFP_KERNEL);
	struct mcuio_device *out, *mdev;
	int i, ret, dev_irqs[MCUIO_FUNCS_PER_DEV];

	pr_debug("%s entered\n", __func__);
	if (!slicm) {
		dev_err(&hc->dev, "error allocating soft local irq struct\n");
		return ERR_PTR(-ENOMEM);
	}

	mdev = &slicm->mdev;
	slicm->chip.name = "MCUIO-SOFT-LOCAL-IRQ-CTRL";
	slicm->chip.irq_mask = mcuio_soft_local_irq_mask;
	slicm->chip.irq_unmask = mcuio_soft_local_irq_unmask;
	slicm->base_irq = base_irq;

	for (i = 0; i < MCUIO_FUNCS_PER_DEV; i++) {
		int irq = slicm->base_irq + i;
		struct fn_irqdata *idata = &slicm->irqdata[i];

		irq_set_chip_data(irq, slicm);
		irq_set_handler_data(irq, idata);
		irq_set_chip(irq, &slicm->chip);
		irq_set_handler(irq, &handle_simple_irq);
		irq_modify_status(irq,
				  IRQ_NOREQUEST | IRQ_NOAUTOEN, IRQ_NOPROBE);
		dev_irqs[i] = irq;

		idata->irq = irq;
		init_kthread_worker(&idata->irq_kworker);
		idata->irq_kworker_task = kthread_run(kthread_worker_fn,
						      &idata->irq_kworker,
						      "mcuio_irq_%d_%d",
						      fn, i);
		if (IS_ERR(idata->irq_kworker_task)) {
			int j;

			pr_err("failed to create irq tsk for %d, fn %d\n",
			       hc->device, i);
			mdev = ERR_PTR(PTR_ERR(idata->irq_kworker_task));
			for (j = 0; j < i; j++) {
				idata = &slicm->irqdata[j];
				kthread_stop(idata->irq_kworker_task);
			}
			break;
		}
		init_kthread_work(&idata->do_irq, __do_irq);
	}
	if (IS_ERR(mdev)) {
		out = mdev;
		goto err0;
	}
	mdev->id.device = MCUIO_DEVICE_LOCAL_IRQC_MSG;
	mdev->id.vendor = MCUIO_VENDOR_DOGHUNTER;
	mdev->id.class = MCUIO_CLASS_SOFT_LOCAL_IRQ_CONTROLLER_PROTO;
	mdev->id.class_mask = 0xffffffff;
	mdev->bus = hc->bus;
	/* Local device */
	mdev->device = 0;
	mdev->fn = fn;
	pr_debug("%s %d, device = 0x%04x, vendor = 0x%04x, "
		 "class = 0x%04x\n", __func__, __LINE__, mdev->id.device,
		 mdev->id.vendor, mdev->id.class);

	/*
	 * Register a callback for write requests to this device
	 * over the mcuio bus
	 */
	slicm->write_req.cb = mcuio_soft_local_irq_controller_msg_wcb;
	slicm->write_req.cb_data = slicm;

	mcuio_init_request(&slicm->write_req, hc, mdev->device, mdev->fn,
			   mcuio_type_wrdw, 0, 0, 0);

	ret = mcuio_setup_cb(&slicm->write_req);
	if (ret < 0) {
		dev_err(&hc->dev, "error setting up write callback\n");
		out = ERR_PTR(ret);
		goto err0;
	}

	/*
	 * The hc is not our parent
	 */
	ret = mcuio_device_register(mdev,
				    &mcuio_soft_local_irq_ctrl_device_type,
				    NULL);
	if (ret < 0) {
		dev_err(&hc->dev,
			"error registering device %u:%u.%u\n", hc->bus, 0, fn);
		out = ERR_PTR(ret);
		goto err1;
	}

	ret = mcuio_hc_set_irqs(hc, fn, dev_irqs);
	if (ret < 0) {
		dev_err(&hc->dev, "error setting irqs\n");
		mcuio_device_unregister(mdev);
		out = ERR_PTR(ret);
		goto err1;
	}

	out = mdev;

	pr_debug("%s successful, created device %s\n",
		 __func__, dev_name(&out->dev));

	return out;


err1:
	mcuio_cancel_cb(&slicm->write_req);
err0:
	irq_free_descs(slicm->base_irq, MCUIO_FUNCS_PER_DEV);
	kfree(mdev);
	return out;
}
EXPORT_SYMBOL(mcuio_add_soft_local_irq_ctrl);

MODULE_AUTHOR("Davide Ciminaghi");
MODULE_DESCRIPTION("MCUIO irq controller driver");
MODULE_LICENSE("GPL v2");
