#ifndef __MCUIO_H__
#define __MCUIO_H__

#ifdef __KERNEL__

#include <linux/device.h>
#include <linux/module.h>

struct mcuio_packet;

/*
 * Id of an mcuio device.
 */
struct mcuio_device_id {
	unsigned int device;
	unsigned int vendor;
	unsigned int class;
	unsigned int class_mask;
};

/*
 * An mcuio device.
 * @id: device id, as defined above
 * @bus: bus number
 * @device: device number (0 for host controllers)
 * @fn: function number (0 for host controllers)
 * @dev: the relevant device
 * @irq: irq number for device
 */
struct mcuio_device {
	struct mcuio_device_id id;
	unsigned bus, device, fn;
	struct device dev;
	int irq;
};

#define to_mcuio_dev(_dev) container_of(_dev, struct mcuio_device, dev)

/*
 * mcuio_driver -- an mcuio driver struc
 */
struct mcuio_driver {
	const struct mcuio_device_id	*id_table;
	int (*probe)(struct mcuio_device *dev);
	int (*remove)(struct mcuio_device *dev);
	int (*input_ready)(struct mcuio_device *dev);
	struct device_driver		driver;
};

#define to_mcuio_drv(_drv) container_of(_drv, struct mcuio_driver, driver)

/*
 * The parent of all mcuio controllers on this machine
 */
extern struct device mcuio_bus;

int mcuio_driver_register(struct mcuio_driver *drv, struct module *owner);
void mcuio_driver_unregister(struct mcuio_driver *drv);
int mcuio_device_register(struct mcuio_device *dev,
			  struct device_type *type,
			  struct device *parents);
void mcuio_device_unregister(struct mcuio_device *dev);

struct mcuio_request;

typedef void (*request_cb)(struct mcuio_request *);

/*
 * mcuio request flags
 */
/*
 * fill: if this is !0 the resulting request packet shall have its fill data
 *       flag set
 */
#define MCUIO_REQUEST_FILL BIT(0)
/*
 * incoming: if this is !0 the request is incoming (outgoing otherwise)
 */
#define MCUIO_REQUEST_INCOMING BIT(1)

/*
 * This represents an mcuio request
 * @hc: pointer to host controller mcuio device
 * @dev: destination device
 * @func: destination function
 * @offset: offset within function address space
 * @offset_mask: this mask is applied to incoming packets' offsets when
 *		 looking for matching pending requests
 * @type: request type
 * @cb: pointer to callback function
 * @cb_data: callback data.
 * @status: status of request (0 completed OK, -EXXXX errors)
 * @data: request data
 * @list: used for enqueueing requests
 * @to_work: delayed_work struct for request timeout management
 * @priv: private data. FIX THIS
 * @flags: request flags (MCUIO_REQUEST_XXX)
 * @release: pointer to memory release function
 */
struct mcuio_request {
	struct mcuio_device *hc;
	unsigned dev;
	unsigned func;
	unsigned offset;
	unsigned offset_mask;
	unsigned type;
	request_cb cb;
	void *cb_data;
	int status;
	uint32_t data[2];
	struct list_head list;
	struct delayed_work to_work;
	void *priv;
	unsigned int flags;
	void (*release)(struct mcuio_request *);
};

static inline int mcuio_request_is_fill(struct mcuio_request *r)
{
	return r->flags & MCUIO_REQUEST_FILL;
}

static inline void mcuio_request_set_fill(struct mcuio_request *r, int f)
{
	r->flags &= ~MCUIO_REQUEST_FILL;
	if (f)
		r->flags |= MCUIO_REQUEST_FILL;
}

static inline int mcuio_request_is_incoming(struct mcuio_request *r)
{
	return r->flags & MCUIO_REQUEST_INCOMING;
}

static inline void mcuio_request_set_incoming(struct mcuio_request *r, int f)
{
	r->flags &= ~MCUIO_REQUEST_INCOMING;
	if (f)
		r->flags |= MCUIO_REQUEST_INCOMING;
}

/*
 * Submit a request, block until request done
 *
 * @r: pointer to request
 */
int mcuio_submit_request(struct mcuio_request *r);

/*
 * Setup a callback for an incoming request
 *
 * @r: pointer to corresponding request
 */
int mcuio_setup_cb(struct mcuio_request *r);

/*
 * Cancel a callback for an incoming request
 *
 * @r: pointer to corresponding request
 */
int mcuio_cancel_cb(struct mcuio_request *r);

/*
 * Fill a non-dynamically allocated mcuio request
 */
void mcuio_init_request(struct mcuio_request *r,
			struct mcuio_device *mdev,
			unsigned dev, unsigned func,
			unsigned type,
			int fill,
			unsigned offset,
			unsigned offset_mask);

/*
 * Dynamically allocate an mcuio request and initialize it
 */
struct mcuio_request *mcuio_make_request(struct mcuio_device *mdev,
					 unsigned dev, unsigned func,
					 unsigned type,
					 int fill,
					 unsigned offset,
					 unsigned offset_mask);

/*
 * Free an mcuio request
 */
static inline void mcuio_free_request(struct mcuio_request *r)
{
	if (r->release)
		r->release(r);
}

#endif /* __KERNEL__ */

#endif /* __MCUIO_H__ */
