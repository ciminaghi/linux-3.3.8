#ifndef __MCUIO_IDS_H__
#define __MCUIO_IDS_H__

/* Various class definitions */

#define MCUIO_CLASS_UNDEFINED	    0x0000
/* Hardware implementation of host controller */
#define MCUIO_CLASS_HOST_CONTROLLER 0x0001
/* Software implementation of host controller (line discipline) */
#define MCUIO_CLASS_SOFT_HOST_CONTROLLER 0x0011
#define MCUIO_CLASS_GPIO	    0x0002
#define MCUIO_CLASS_ADC		    0x0003
#define MCUIO_CLASS_DAC		    0x0004
#define MCUIO_CLASS_PWM		    0x0005
#define MCUIO_CLASS_GRAPHIC_DISPLAY 0x0006
#define MCUIO_CLASS_TEXT_DISPLAY    0x0007
#define MCUIO_CLASS_I2C_CONTROLLER  0x0008
#define MCUIO_CLASS_SPI_CONTROLLER  0x0009
/* Send irqs from MCU via communication protocol */
#define MCUIO_CLASS_IRQ_CONTROLLER_PROTO  0x000a
/* Send irqs from MCU via gpio */
#define MCUIO_CLASS_IRQ_CONTROLLER_WIRE   0x000b
/* Add-on shield */
#define MCUIO_CLASS_SHIELD	    0x000c
/* Local message based irq controller */
#define MCUIO_CLASS_SOFT_LOCAL_IRQ_CONTROLLER_PROTO 0x000d


/* Invalid device id (used for id table termination */
#define MCUIO_NO_DEVICE		    0x0000


/* Vendors */
#define MCUIO_VENDOR_DOGHUNTER	    0x0001


/* Specific devices */
#define MCUIO_DEVICE_JOYSTICK_SHIELD 0x0001
#define MCUIO_DEVICE_LUCKY_SHIELD    0x0002
#define MCUIO_DEVICE_DIGITALIO_SHIELD 0x0003
#define MCUIO_DEVICE_SOFT_HC	     0x0004
#define MCUIO_DEVICE_GENERIC_HC	     0x0005
#define MCUIO_DEVICE_LOCAL_IRQC_MSG  0x0006

#endif /* __MCUIO_IDS_H__ */
