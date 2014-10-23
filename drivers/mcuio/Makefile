# This comes from the zio Makefile

LINUX ?= /lib/modules/$(shell uname -r)/build

mcuio-y := core.o bus.o sysfs.o

obj-$(CONFIG_MCUIO) += mcuio.o mcuio-hc-drv.o mcuio-hc-dev.o mcuio-soft-hc.o \
mcuio-soft-local-irq-ctrl-msg-dev.o
obj-$(CONFIG_MCUIO_LDISC_HC) += mcuio-hc-ldisc.o
obj-$(CONFIG_MCUIO_JS_SHIELD) += mcuio-js-shield.o
obj-$(CONFIG_MCUIO_LUCKY_SHIELD) += mcuio-lucky-shield.o
obj-$(CONFIG_MCUIO_DIGITALIO_SHIELD) += mcuio-dio-shield.o
obj-$(CONFIG_MCUIO_DOGOLED_SHIELD) += mcuio-dogoled-shield.o
obj-$(CONFIG_MCUIO_SHIELDS_MANUAL_PROBE) += mcuio-shields-manprobe.o
obj-$(CONFIG_MCUIO_IRQ_TEST) += mcuio-irq-test.o
