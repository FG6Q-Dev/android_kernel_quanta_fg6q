/*
 * arch/arm/mach-tegra/panel-c-11-6.c
 *
 * Copyright (c) 2013, QUANTA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <mach/dc.h>
#include <mach/iomap.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/tegra_pwm_bl.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm_backlight.h>
#include <linux/leds.h>
#include <linux/ioport.h>
#include "board.h"
#include "board-panel.h"
#include "devices.h"
#include "gpio-names.h"
#include "tegra11_host1x_devices.h"

#define TEGRA_DSI_GANGED_MODE	0

#define DSI_PANEL_RESET		0
#define DSI_PANEL_BL_PWM	TEGRA_GPIO_PH1

#define DC_CTRL_MODE	TEGRA_DC_OUT_CONTINUOUS_MODE

#define LVDS_EN_1V8	TEGRA_GPIO_PG3
#define LVDS_SHDN	TEGRA_GPIO_PR5

#define BL_EN		TEGRA_GPIO_PX4

static bool reg_requested;
static bool gpio_requested;
static struct platform_device *disp_device;
static struct regulator *vdd_lcd_bl_en;

static tegra_dc_bl_output dsi_c_1080p_11_6_bl_output_measured = {
	0, 0, 1, 2, 3, 4, 5, 6,
	7, 8, 9, 9, 10, 11, 12, 13,
	13, 14, 15, 16, 17, 17, 18, 19,
	20, 21, 22, 22, 23, 24, 25, 26,
	27, 27, 28, 29, 30, 31, 32, 32,
	33, 34, 35, 36, 37, 37, 38, 39,
	40, 41, 42, 42, 43, 44, 45, 46,
	47, 48, 48, 49, 50, 51, 52, 53,
	54, 55, 56, 57, 57, 58, 59, 60,
	61, 62, 63, 64, 65, 66, 67, 68,
	69, 70, 71, 71, 72, 73, 74, 75,
	76, 77, 77, 78, 79, 80, 81, 82,
	83, 84, 85, 87, 88, 89, 90, 91,
	92, 93, 94, 95, 96, 97, 98, 99,
	100, 101, 102, 103, 104, 105, 106, 107,
	108, 109, 110, 111, 112, 113, 115, 116,
	117, 118, 119, 120, 121, 122, 123, 124,
	125, 126, 127, 128, 129, 130, 131, 132,
	133, 134, 135, 136, 137, 138, 139, 141,
	142, 143, 144, 146, 147, 148, 149, 151,
	152, 153, 154, 155, 156, 157, 158, 158,
	159, 160, 161, 162, 163, 165, 166, 167,
	168, 169, 170, 171, 172, 173, 174, 176,
	177, 178, 179, 180, 182, 183, 184, 185,
	186, 187, 188, 189, 190, 191, 192, 194,
	195, 196, 197, 198, 199, 200, 201, 202,
	203, 204, 205, 206, 207, 208, 209, 210,
	211, 212, 213, 214, 215, 216, 217, 219,
	220, 221, 222, 224, 225, 226, 227, 229,
	230, 231, 232, 233, 234, 235, 236, 238,
	239, 240, 241, 242, 243, 244, 245, 246,
	247, 248, 249, 250, 251, 252, 253, 255
};

static struct tegra_dsi_cmd dsi_c_1080p_11_6_init_cmd[] = {
	/* no init command required */
};

static struct tegra_dsi_out dsi_c_1080p_11_6_pdata = {
	.controller_vs = DSI_VS_1,
	.dsi2edp_bridge_enable = true,

	.n_data_lanes = 4,
	.video_burst_mode = TEGRA_DSI_VIDEO_NONE_BURST_MODE_WITH_SYNC_END,

	.pixel_format = TEGRA_DSI_PIXEL_FORMAT_24BIT_P,
	.refresh_rate = 60,
	.virtual_channel = TEGRA_DSI_VIRTUAL_CHANNEL_0,

	.dsi_instance = DSI_INSTANCE_0,

	.panel_reset = DSI_PANEL_RESET,
	.power_saving_suspend = true,
	.video_data_type = TEGRA_DSI_VIDEO_TYPE_VIDEO_MODE,
	.video_clock_mode = TEGRA_DSI_VIDEO_CLOCK_TX_ONLY,
	.dsi_init_cmd = dsi_c_1080p_11_6_init_cmd,
	.n_init_cmd = ARRAY_SIZE(dsi_c_1080p_11_6_init_cmd),
};

static int dsi_regulator_get(struct device *dev)
{
	int err = 0;

	if (reg_requested)
		return 0;

	vdd_lcd_bl_en = regulator_get(dev, "vdd_lcd_bl_en");
	if (IS_ERR_OR_NULL(vdd_lcd_bl_en)) {
		pr_err("vdd_lcd_bl_en regulator get failed\n");
		err = PTR_ERR(vdd_lcd_bl_en);
		vdd_lcd_bl_en = NULL;
		goto fail;
	}
	reg_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_gpio_get(void)
{
	int err = 0;

	if (gpio_requested)
		return 0;

	err = gpio_request(LVDS_EN_1V8, "LVDS_EN_1V8");
	if (err < 0) {
		pr_err("LVDS_EN_1V8 gpio request failed\n");
		goto fail;
	}

	err = gpio_request(LVDS_SHDN, "LVDS_SHDN");
	if (err < 0) {
		pr_err("LVDS_SHDN gpio request failed\n");
		goto fail;
	}

	err = gpio_request(BL_EN, "BL_EN");
	if (err < 0) {
		pr_err("BL_EN gpio request failed\n");
		goto fail;
	}

	/* free pwm GPIO */
	err = gpio_request(DSI_PANEL_BL_PWM, "panel pwm");
	if (err < 0) {
		pr_err("panel pwm gpio request failed\n");
		goto fail;
	}
	gpio_free(DSI_PANEL_BL_PWM);

	gpio_requested = true;
	return 0;
fail:
	return err;
}

static int dsi_c_1080p_11_6_enable(struct device *dev)
{
	int err = 0;

	err = dsi_regulator_get(dev);
	if (err < 0) {
		pr_err("dsi regulator get failed\n");
		goto fail;
	}
	err = dsi_gpio_get();
	if (err < 0) {
		pr_err("dsi gpio request failed\n");
		goto fail;
	}

	gpio_direction_output(LVDS_EN_1V8, 1);
	msleep(10);
	gpio_direction_output(LVDS_SHDN, 0);
	msleep(20);
	gpio_set_value(LVDS_SHDN, 1);
	msleep(10);

	gpio_direction_output(BL_EN, 1);
	msleep(40);

	if (vdd_lcd_bl_en) {
		err = regulator_enable(vdd_lcd_bl_en);
		if (err < 0) {
			pr_err("vdd_lcd_bl_en regulator enable failed\n");
			goto fail;
		}
	}

	return 0;
fail:
	return err;
}

static int dsi_c_1080p_11_6_disable(void)
{
	if (vdd_lcd_bl_en)
		regulator_disable(vdd_lcd_bl_en);
	msleep(30);

	gpio_set_value(BL_EN, 0);
	msleep(10);

	gpio_set_value(LVDS_SHDN, 0);

	gpio_set_value(LVDS_EN_1V8, 0);

	return 0;
}

static int dsi_c_1080p_11_6_postsuspend(void)
{
	return 0;
}

static struct tegra_dc_mode dsi_c_1080p_11_6_modes[] = {
	{
		.pclk = 138777600,
		.h_ref_to_sync = 4,
		.v_ref_to_sync = 1,
		.h_sync_width = 32,
		.v_sync_width = 5,
		.h_back_porch = 80,
		.v_back_porch = 24,
		.h_active = 1920,
		.v_active = 1080,
		.h_front_porch = 48,
		.v_front_porch = 3,
	},
};

static int dsi_c_1080p_11_6_bl_notify(struct device *unused, int brightness)
{
	int cur_sd_brightness = atomic_read(&sd_brightness);

	/* Apply any backlight response curve */
	if (brightness > 255)
		pr_info("Error: Brightness > 255!\n");
	else
		brightness = dsi_c_1080p_11_6_bl_output_measured[brightness];

	/* SD brightness is a percentage */
	brightness = (brightness * cur_sd_brightness) / 255;

	return brightness;
}

static int dsi_c_1080p_11_6_check_fb(struct device *dev, struct fb_info *info)
{
	return info->device == &disp_device->dev;
}

static struct platform_pwm_backlight_data dsi_c_1080p_11_6_bl_data = {
	.pwm_id		= 1,
	.max_brightness	= 255,
	.dft_brightness	= 224,
	.pwm_period_ns	= 1000000,
	.notify		= dsi_c_1080p_11_6_bl_notify,
	/* Only toggle backlight on fb blank notifications for disp1 */
	.check_fb	= dsi_c_1080p_11_6_check_fb,
};

static struct platform_device __maybe_unused
		dsi_c_1080p_11_6_bl_device __initdata = {
	.name	= "pwm-backlight",
	.id	= -1,
	.dev	= {
		.platform_data = &dsi_c_1080p_11_6_bl_data,
	},
};

static struct platform_device __maybe_unused
			*dsi_c_1080p_11_6_bl_devices[] __initdata = {
	&tegra_pwfm1_device,
	&dsi_c_1080p_11_6_bl_device,
};

static int  __init dsi_c_1080p_11_6_register_bl_dev(void)
{
	int err = 0;
	err = platform_add_devices(dsi_c_1080p_11_6_bl_devices,
				ARRAY_SIZE(dsi_c_1080p_11_6_bl_devices));
	if (err) {
		pr_err("disp1 bl device registration failed");
		return err;
	}
	return err;
}

static void dsi_c_1080p_11_6_set_disp_device(
	struct platform_device *tegradc_display_device)
{
	disp_device = tegradc_display_device;
}

static void dsi_c_1080p_11_6_resources_init(struct resource *
resources, int n_resources)
{
	int i;
	for (i = 0; i < n_resources; i++) {
		struct resource *r = &resources[i];
		if (resource_type(r) == IORESOURCE_MEM &&
			!strcmp(r->name, "dsi_regs")) {
			r->start = TEGRA_DSI_BASE;
			r->end = TEGRA_DSI_BASE + TEGRA_DSI_SIZE - 1;
		}
	}
}

static void dsi_c_1080p_11_6_dc_out_init(struct tegra_dc_out *dc)
{
	dc->dsi = &dsi_c_1080p_11_6_pdata;
	dc->parent_clk = "pll_d_out0";
	dc->modes = dsi_c_1080p_11_6_modes;
	dc->n_modes = ARRAY_SIZE(dsi_c_1080p_11_6_modes);
	dc->enable = dsi_c_1080p_11_6_enable;
	dc->disable = dsi_c_1080p_11_6_disable;
	dc->postsuspend	= dsi_c_1080p_11_6_postsuspend,
	dc->width = 256;
	dc->height = 144;
	dc->flags = DC_CTRL_MODE;
}

static void dsi_c_1080p_11_6_fb_data_init(struct tegra_fb_data *fb)
{
	fb->xres = dsi_c_1080p_11_6_modes[0].h_active;
	fb->yres = dsi_c_1080p_11_6_modes[0].v_active;
}

static void
dsi_c_1080p_11_6_sd_settings_init(struct tegra_dc_sd_settings *settings)
{
	settings->bl_device_name = "pwm-backlight";
}

struct tegra_panel __initdata dsi_c_1080p_11_6 = {
	.init_sd_settings = dsi_c_1080p_11_6_sd_settings_init,
	.init_dc_out = dsi_c_1080p_11_6_dc_out_init,
	.init_fb_data = dsi_c_1080p_11_6_fb_data_init,
	.init_resources = dsi_c_1080p_11_6_resources_init,
	.register_bl_dev = dsi_c_1080p_11_6_register_bl_dev,
	.set_disp_device = dsi_c_1080p_11_6_set_disp_device,
};
EXPORT_SYMBOL(dsi_c_1080p_11_6);

