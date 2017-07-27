/*
 *  Quanta header file
 *  
 *  Copyright (c) 2010 Atmel Corporation
 *  Copyright (C) 2010 Ulf Samuelsson (ulf@atmel.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 *  See the file "COPYING" in the main directory of this archive
 *  for more details.
 *
 */
#include <linux/gpio.h>

#include "gpio-names.h"
#include "hw_version.h"

#if defined( CONFIG_PROJECT_FG6Q)
#define BOARD_ID_0	TEGRA_GPIO_PH6
#define BOARD_ID_1	TEGRA_GPIO_PB1
#define RAMCODES_ID_0	TEGRA_GPIO_PG4
#define RAMCODES_ID_1	TEGRA_GPIO_PG5
#endif

int qci_mainboard_version( void )
{	
	static int board_version = 0xff;

	gpio_request(BOARD_ID_0, "HW Mainboard ID");
	gpio_request(BOARD_ID_1, "HW Mainboard ID");
	gpio_direction_input(BOARD_ID_0);
	gpio_direction_input(BOARD_ID_1);

	/* Get version */
	board_version = gpio_get_value( BOARD_ID_1 );
	board_version <<= 1;
	board_version += gpio_get_value( BOARD_ID_0 );
	//printk(KERN_INFO
	//       "HW Mainboard Version is : 0x%x\n", board_version );

	gpio_free(BOARD_ID_1);
	gpio_free(BOARD_ID_0);

	return board_version;
}

int qci_ramcodes_version( void )
{
	static int ramcodes_version = 0xff;

	gpio_request(RAMCODES_ID_0, "HW RAMCODES0 ID");
	gpio_request(RAMCODES_ID_1, "HW RAMCODES1 ID");
	gpio_direction_input(RAMCODES_ID_0);
	gpio_direction_input(RAMCODES_ID_1);

	/* Get RAM_CODE[1:0] */
	ramcodes_version = gpio_get_value( RAMCODES_ID_1 );
	ramcodes_version <<= 1;
	ramcodes_version += gpio_get_value( RAMCODES_ID_0 );

	gpio_free(RAMCODES_ID_1);
	gpio_free(RAMCODES_ID_0);
	
	return ramcodes_version;
}

