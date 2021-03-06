/*
 *  Linino board support
 *
 *  Copyright (C) 2011-2012 Gabor Juhos <juhosg@openwrt.org>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License version 2 as published
 *  by the Free Software Foundation.
 */

#include "dev-eth.h"
#include "dev-gpio-buttons.h"
#include "dev-leds-gpio.h"
#include "dev-m25p80.h"
#include "dev-spi.h"
#include "dev-usb.h"
#include "dev-wmac.h"
#include "machtypes.h"
#include <asm/mach-ath79/ar71xx_regs.h>
#include <asm/mach-ath79/ath79.h>
#include <asm/mach-ath79/mach-linino.h>
#include "common.h"
#include "gpio.h"
#include "linux/gpio.h"
#include <linux/platform_device.h>
#include <linux/spi/spi_gpio.h>

/* * * * * * * * * * * * * * * * * * * LED * * * * * * * * * * * * * * * * * */

static struct gpio_led ds_leds_gpio[] __initdata = {
	{
		.name = "ds:green:usb",
		.gpio = DS_GPIO_LED_USB,
		.active_low = 0,
	},
	{
		.name = "ds:green:wlan",
		.gpio = DS_GPIO_LED_WLAN,
		.active_low = 0,
	},
};

/* * * * * * * * * * * * * * * * * BUTTONS * * * * * * * * * * * * * * * * * */

static struct gpio_keys_button ds_gpio_keys[] __initdata = {
	{
		.desc = "configuration button",
		.type = EV_KEY,
		.code = KEY_WPS_BUTTON,
		.debounce_interval = DS_KEYS_DEBOUNCE_INTERVAL,
		.gpio = DS_GPIO_CONF_BTN,
		.active_low = 1,
	},
};


/* * * * * * * * * * * * * * * * * * * SPI * * * * * * * * * * * * * * * * * */

/*
 * The SPI bus between the main processor and the MCU is available only in the
 * following board: YUN, FREEDOG
 */

static struct spi_gpio_platform_data spi_bus1 = {
	.sck = LININO_GPIO_SPI_SCK,
	.mosi = LININO_GPIO_SPI_MOSI,
	.miso = LININO_GPIO_SPI_MISO,
	.num_chipselect = LININO_N_SPI_CHIP_SELECT,
};

static struct platform_device linino_spi1_device = {
	.name	= "spi_gpio",
	.id	= 1,
	.dev.platform_data = &spi_bus1,
};

/* SPI devices on Linino */
static struct spi_board_info linino_spi_info[] = {
	/*{
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 10000000,
		.mode			= 0,
		.modalias		= "spidev",
		.controller_data	= (void *) SPI_CHIP_SELECT,
	},*/
	{
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 10000000, /* unused but required */
		.mode			= 0,
		.modalias		= "atmega32u4",
		.controller_data	= (void *) LININO_GPIO_SPI_CS0,
		.platform_data		= (void *) LININO_GPIO_SPI_INTERRUPT,
	},
};

/**
 * Enable the software SPI controller emulated by GPIO signals
 */
static void ds_register_spi(void) {
	pr_info("mach-linino: enabling GPIO SPI Controller");

	/* Enable level shifter on SPI signals */
	gpio_set_value(DS_GPIO_OE, 1);
	/* Enable level shifter on AVR interrupt */
	gpio_set_value(DS_GPIO_OE2, 1);
	/* Register SPI devices */
	spi_register_board_info(linino_spi_info, ARRAY_SIZE(linino_spi_info));
	/* Register GPIO SPI controller */
	platform_device_register(&linino_spi1_device);
}


/* * * * * * * * * * * * * * * * * * SETUP * * * * * * * * * * * * * * * * * */

static void __init ds_common_setup(void)
{
	static u8 mac[6];

	u8 *art = (u8 *) KSEG1ADDR(0x1fff0000);
	ath79_register_m25p80(NULL);

	if (ar93xx_wmac_read_mac_address(mac)) {
		ath79_register_wmac(NULL, NULL);
	} else {
		ath79_register_wmac(art + DS_CALDATA_OFFSET,
				art + DS_WMAC_MAC_OFFSET);
		memcpy(mac, art + DS_WMAC_MAC_OFFSET, sizeof(mac));
	}

	mac[3] |= 0x08;
	ath79_init_mac(ath79_eth0_data.mac_addr, mac, 0);

	mac[3] &= 0xF7;
	ath79_init_mac(ath79_eth1_data.mac_addr, mac, 0);

	ath79_register_mdio(0, 0x0);

	/* LAN ports */
	ath79_register_eth(1);

	/* WAN port */
	ath79_register_eth(0);
}

/*
 * Enable level shifters
 */
static void __init ds_setup_level_shifter_oe(void)
{
	int err;

	/* enable OE of level shifter */
	pr_info("Setting GPIO OE %d\n", DS_GPIO_OE);
	err = gpio_request_one(DS_GPIO_OE,
			       GPIOF_OUT_INIT_LOW | GPIOF_EXPORT_DIR_FIXED,
			       "OE-1");
	if (err)
		pr_err("mach-linino: error setting GPIO OE\n");

	/* enable OE2 of level shifter */
	pr_info("Setting GPIO OE2 %d\n", DS_GPIO_OE2);
	err= gpio_request_one(DS_GPIO_OE2,
			      GPIOF_OUT_INIT_LOW | GPIOF_EXPORT_DIR_FIXED,
			      "OE-2");
	if (err)
		pr_err("mach-linino: error setting GPIO OE2\n");
}


/*
 * Enable UART
 */
static void ds_setup_uart_enable(void)
{
	int err;

	pr_info("Setting GPIO UART-ENA %d\n", DS_GPIO_UART_ENA);
	err = gpio_request_one(DS_GPIO_UART_ENA,
			       GPIOF_OUT_INIT_LOW | GPIOF_EXPORT_DIR_FIXED,
			       "UART-ENA");
	if (err)
		pr_err("mach-linino: error setting GPIO Uart Enable\n");
}


static void __init ds_setup(void)
{
	u32 t=0;

	ds_common_setup();

	ath79_register_leds_gpio(-1, ARRAY_SIZE(ds_leds_gpio), ds_leds_gpio);
	ath79_register_gpio_keys_polled(-1, DS_KEYS_POLL_INTERVAL,
			ARRAY_SIZE(ds_gpio_keys), ds_gpio_keys);
	ath79_register_usb();

	/*
	 * Disable the Function for some pins to have GPIO functionality active
	 * GPIO6-7-8 and GPIO11
	 */
	ath79_gpio_function_setup(GPIO_FUNC_SET, GPIO_FUNC_CLEAR);

	ath79_gpio_function2_setup(GPIO_FUNC2_SET, GPIO_FUNC2_CLEAR);

	pr_info("mach-linino: setting GPIO\n");

	/* Enable GPIO26 instead of MDC function */
	t = ath79_reset_rr(AR933X_RESET_REG_BOOTSTRAP);
	t |= AR933X_BOOTSTRAP_MDIO_GPIO_EN;
	ath79_reset_wr(AR933X_RESET_REG_BOOTSTRAP, t);

	/* enable OE of level shifters */
	ds_setup_level_shifter_oe();
	/* enable uart */
	ds_setup_uart_enable();

	/* Register Software SPI controller */
	ds_register_spi();
}
