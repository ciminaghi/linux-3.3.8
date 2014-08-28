/*
 * Platform data definition for the ssd1306/1307 oled display
 *
 * Copyright (C) Doghunter
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#ifndef _SSD1307_H
#define _SSD1307_H

#define SSD1307_TYPE_1306 1
#define SSD1307_TYPE_1307 2

struct ssd1307_platform_data {
	/* SSD1307_TYPE_xxxx */
	int type;
	int reset_gpio;
	u32 width;
	u32 height;
	u32 page_offset;
	u32 display_offset;
	u8 pins_config;
};

#endif /* _SSD1307_H */
