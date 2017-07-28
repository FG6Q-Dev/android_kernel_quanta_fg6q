/*
 * arch/arm/mach-tegra/board-dalmore-pinmux.c
 *
 * Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <mach/pinmux.h>
#include <mach/gpio-tegra.h>
#include "board.h"
#include "board-dalmore.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra-board-id.h"

#include <mach/pinmux-t11.h>

static __initdata struct tegra_drive_pingroup_config dalmore_drive_pinmux[] = {
	/* DEFAULT_DRIVE(<pin_group>), */
	/* SDMMC1 */
	SET_DRIVE(SDIO1, ENABLE, DISABLE, DIV_1, 36, 20, SLOW, SLOW),

	/* SDMMC3 */
	SET_DRIVE(SDIO3, ENABLE, DISABLE, DIV_1, 22, 36, FASTEST, FASTEST),

	/* SDMMC4 */
	SET_DRIVE_WITH_TYPE(GMA, ENABLE, DISABLE, DIV_1, 2, 2, FASTEST,
								FASTEST, 1),
};

#include "board-dalmore-pinmux-t11x-ep5n.h"


/* THIS IS FOR TESTING OR WORKAROUND PURPOSES. ANYTHING INSIDE THIS TABLE
 * SHOULD BE PUSHED TO PINMUX SPREADSHEET FOR AUTOGEN OR FIXED
 * */
static __initdata struct tegra_pingroup_config manual_config_pinmux[] = {
	/* SDMMC CLKs are not supposed to be input enabled.
	 * Verify if these can be removed. */
	DEFAULT_PINMUX(SDMMC1_CLK,    SDMMC1,      NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(SDMMC3_CLK,    SDMMC3,      NORMAL,    NORMAL,   INPUT),
	DEFAULT_PINMUX(SDMMC4_CLK,    SDMMC4,      NORMAL,    NORMAL,   INPUT),

	/* DDC_PINMUX is not implemented in the spreadsheet.
	 * Please file a bug and specify the right rules for DDC_PINMUX
	 * so that an effort can be made. */
	DDC_PINMUX(DDC_SCL, I2C4, NORMAL, NORMAL, INPUT, DISABLE, HIGH),
	DDC_PINMUX(DDC_SDA, I2C4, NORMAL, NORMAL, INPUT, DISABLE, HIGH),

};

static __initdata struct tegra_pingroup_config dalmore_e1611_1000[] = {
	/* io rdy Lock rotation */
	DEFAULT_PINMUX(GMI_IORDY,	GMI,	NORMAL,	NORMAL,	INPUT),
};

static __initdata struct tegra_pingroup_config dalmore_e1611_1001[] = {
	/* kbcol1 rdy Lock rotation */
	DEFAULT_PINMUX(KB_COL1,       KBC,         NORMAL,   NORMAL, INPUT),
};

static void __init dalmore_gpio_init_configure(void)
{
	int len;
	int i;
	struct gpio_init_pin_info *pins_info;

	len = ARRAY_SIZE(init_gpio_mode_dalmore_common);
	pins_info = init_gpio_mode_dalmore_common;

	for (i = 0; i < len; ++i) {
		tegra_gpio_init_configure(pins_info->gpio_nr,
			pins_info->is_input, pins_info->value);
		pins_info++;
	}
}

int __init dalmore_pinmux_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	dalmore_gpio_init_configure();

	tegra_pinmux_config_table(dalmore_pinmux_common, ARRAY_SIZE(dalmore_pinmux_common));
	tegra_drive_pinmux_config_table(dalmore_drive_pinmux,
					ARRAY_SIZE(dalmore_drive_pinmux));
	tegra_pinmux_config_table(unused_pins_lowpower,
		ARRAY_SIZE(unused_pins_lowpower));

	if ((board_info.board_id == BOARD_E1611) && (board_info.sku == 1000))
		tegra_pinmux_config_table(dalmore_e1611_1000,
			ARRAY_SIZE(dalmore_e1611_1000));
	else if ((board_info.board_id == BOARD_E1611) && (board_info.sku == 1001))
		tegra_pinmux_config_table(dalmore_e1611_1001,
			ARRAY_SIZE(dalmore_e1611_1001));

	tegra_pinmux_config_table(manual_config_pinmux,
		ARRAY_SIZE(manual_config_pinmux));

	return 0;
}
