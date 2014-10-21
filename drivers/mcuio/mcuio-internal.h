#ifndef __MCUIO_INTERNAL_H__
#define __MCUIO_INTERNAL_H__

#include <linux/version.h>
#include <linux/mcuio-proto.h>

extern struct bus_type mcuio_bus_type;
extern struct device mcuio_bus;
extern struct attribute_group mcuio_default_dev_attr_group;

int mcuio_get_bus(void);
void mcuio_put_bus(unsigned bus);

struct i2c_adapter *mcuio_get_i2c_adapter(struct mcuio_device *mdev);

struct mcuio_device *mcuio_bus_find_hc(int bus);

struct mcuio_device *mcuio_add_soft_local_irq_ctrl(struct mcuio_device *hc,
						   int fn, int base_irq);

#ifdef DEBUG
static inline void dump_packet(struct mcuio_packet *p)
{
	int i;
	uint8_t *ptr;
	pr_debug("Packet dump:\n");
	pr_debug("\t%s,%s,%s,%u:%u.%u,%s,@0x%04x",
		 mcuio_packet_is_error(p) ? "ERR" : "NOERR" ,
		 mcuio_packet_is_reply(p) ? "REP" : "req",
		 mcuio_packet_is_fill_data(p) ? "[FILL]" : "[NOFILL]",
		 mcuio_packet_bus(p), mcuio_packet_dev(p),
		 mcuio_packet_func(p),
		 mcuio_packet_type_to_str(mcuio_packet_type(p)),
		 mcuio_packet_offset(p));
	printk(KERN_DEBUG "\tpacket: ");
	for (i = 0, ptr = (uint8_t *)p; i < 2*sizeof(uint64_t); i++)
		printk("0x%02x ", ptr[i]);
	printk("\n");
}
#else
static inline void dump_packet(const struct mcuio_packet *packet)
{

}
#endif


#endif /* __MCUIO_INTERNAL_H__ */
