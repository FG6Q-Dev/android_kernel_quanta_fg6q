/*
 * arch/arm/mach-tegra/board-macallan.c
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/serial_8250.h>
#include <linux/i2c.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/i2c-tegra.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/spi/spi.h>
#include <linux/spi/rm31080a_ts.h>
#include <linux/tegra_uart.h>
#include <linux/memblock.h>
#include <linux/spi-tegra.h>
#include <linux/nfc/pn544.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/regulator/consumer.h>
#include <linux/smb349-charger.h>
#include <linux/max17048_battery.h>
#include <linux/leds.h>
#include <linux/i2c/at24.h>
#include <linux/of_platform.h>
#include <linux/edp.h>

#include <asm/hardware/gic.h>

#include <mach/clk.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/pinmux.h>
#include <mach/pinmux-t11.h>
#include <mach/iomap.h>
#include <mach/io.h>
#include <mach/io_dpd.h>
#include <mach/i2s.h>
#include <mach/tegra_asoc_pdata.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/usb_phy.h>
#include <mach/gpio-tegra.h>
#include <mach/tegra_fiq_debugger.h>
#include <linux/platform_data/tegra_usb_modem_power.h>
#include <mach/hardware.h>
#include <mach/xusb.h>

#include "board-touch-raydium.h"
#include "board.h"
#include "board-common.h"
#include "clock.h"
#include "board-macallan.h"
#include "devices.h"
#include "gpio-names.h"
#include "fuse.h"
#include "pm.h"
#include "pm-irq.h"
#include "common.h"
#include "tegra-board-id.h"
#ifdef CONFIG_TOUCHSCREEN_FT5X06
#include <linux/ft5x06_ts.h>
#endif

#ifdef CONFIG_QIC_MULTIPLE_MODEM
#include "qic_modem.h"
#endif

#define GEN3_TOUCH_IRQ_1		TEGRA_GPIO_PK2
#define GEN3_TOUCH_RESET		TEGRA_GPIO_PK4

#if defined CONFIG_TI_ST || defined CONFIG_TI_ST_MODULE
struct ti_st_plat_data macallan_wilink_pdata = {
	.nshutdown_gpio = TEGRA_GPIO_PQ7,
	.dev_name = BLUETOOTH_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3000000,
};

static struct platform_device wl128x_device = {
	.name           = "kim",
	.id             = -1,
	.dev.platform_data = &macallan_wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static noinline void __init macallan_bt_st(void)
{
	pr_info("macallan_bt_st");

	platform_device_register(&wl128x_device);
	platform_device_register(&btwilink_device);
}

static struct resource macallan_st_host_wake_resources[] = {
	[0] = {
		.name = "host_wake",
		.flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
	},
};

static struct platform_device macallan_st_host_wake_device = {
	.name           = "st_host_wake",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(macallan_st_host_wake_resources),
	.resource       = macallan_st_host_wake_resources,
};

static noinline void __init macallan_tegra_setup_st_host_wake(void)
{
	macallan_st_host_wake_resources[0].start =
		macallan_st_host_wake_resources[0].end =
		gpio_to_irq(TEGRA_GPIO_PU6);
	platform_device_register(&macallan_st_host_wake_device);
}
#endif

#if defined CONFIG_BLUEDROID_PM
static struct resource macallan_bluedroid_pm_resources[] = {
       [0] = {
               .name   = "shutdown_gpio",    //BT_EN
               .start  = TEGRA_GPIO_PQ7,
               .end    = TEGRA_GPIO_PQ7,
               .flags  = IORESOURCE_IO,
       },
       [1] = {
               .name = "host_wake",
               .flags  = IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHEDGE,
       },
       [2] = {
               .name = "gpio_ext_wake",        //BT_WAKE_UP
               .start  = TEGRA_GPIO_PEE1,
               .end    = TEGRA_GPIO_PEE1,
               .flags  = IORESOURCE_IO,
       },
       [3] = {
               .name = "gpio_host_wake",       
               .start  = TEGRA_GPIO_PU6,
               .end    = TEGRA_GPIO_PU6,
               /*
               .start  = TEGRA_GPIO_PS0,
               .end    = TEGRA_GPIO_PS0,
               */
               .flags  = IORESOURCE_IO,
       },
       
       [4] = {
               .name = "reset_gpio",
               .start  = TEGRA_GPIO_PQ2,
               .end    = TEGRA_GPIO_PQ2,
               .flags  = IORESOURCE_IO,
       },
       
};

static struct platform_device macallan_bluedroid_pm_device = {
       .name = "bluedroid_pm",
       .id             = 0,
       .num_resources  = ARRAY_SIZE(macallan_bluedroid_pm_resources),
       .resource       = macallan_bluedroid_pm_resources,
};

static noinline void __init macallan_setup_bluedroid_pm(void)
{
       macallan_bluedroid_pm_resources[1].start =
               macallan_bluedroid_pm_resources[1].end =
                               gpio_to_irq(TEGRA_GPIO_PU6);
                               //gpio_to_irq(TEGRA_GPIO_PS0);
       platform_device_register(&macallan_bluedroid_pm_device);
}
#endif

static __initdata struct tegra_clk_init_table macallan_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "pll_m",	NULL,		0,		false},
	{ "hda",	"pll_p",	108000000,	false},
	{ "hda2codec_2x", "pll_p",	48000000,	false},
	{ "pwm",	"pll_p",	3187500,	false},
	{ "blink",	"clk_32k",	32768,		true},
	{ "i2s1",	"pll_a_out0",	0,		false},
	{ "i2s3",	"pll_a_out0",	0,		false},
	{ "i2s4",	"pll_a_out0",	0,		false},
	{ "spdif_out",	"pll_a_out0",	0,		false},
	{ "d_audio",	"clk_m",	12000000,	false},
	{ "dam0",	"clk_m",	12000000,	false},
	{ "dam1",	"clk_m",	12000000,	false},
	{ "dam2",	"clk_m",	12000000,	false},
	{ "audio1",	"i2s1_sync",	0,		false},
	{ "audio3",	"i2s3_sync",	0,		false},
	/* Setting vi_sensor-clk to true for validation purpose, will imapact
	 * power, later set to be false.*/
	//{ "vi_sensor",	"pll_p",	150000000,	false}, // MCLK, default is 150MHz and measured 136MHz, change to 24MHz
	{ "vi_sensor",	"pll_p",	24000000,	false},
	{ "cilab",	"pll_p",	150000000,	false},
	{ "cilcd",	"pll_p",	150000000,	false},
	{ "cile",	"pll_p",	150000000,	false},
	{ "i2c1",	"pll_p",	3200000,	false},
	{ "i2c2",	"pll_p",	3200000,	false},
	{ "i2c3",	"pll_p",	3200000,	false},
	{ "i2c4",	"pll_p",	3200000,	false},
	{ "i2c5",	"pll_p",	3200000,	false},
	{ "extern3","clk_32k",	32768,		false},
	{ NULL,		NULL,		0,		0},
};

static struct tegra_i2c_platform_data macallan_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C1_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C1_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data macallan_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 1,
	.bus_clk_rate	= { 100000, 0 },
	.is_clkon_always = true,
	.scl_gpio		= {TEGRA_GPIO_I2C2_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C2_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data macallan_i2c3_platform_data = {
	.adapter_nr	= 2,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C3_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C3_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data macallan_i2c4_platform_data = {
	.adapter_nr	= 3,
	.bus_count	= 1,
	.bus_clk_rate	= { 10000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C4_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C4_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct tegra_i2c_platform_data macallan_i2c5_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.scl_gpio		= {TEGRA_GPIO_I2C5_SCL, 0},
	.sda_gpio		= {TEGRA_GPIO_I2C5_SDA, 0},
	.arb_recovery = arb_lost_recovery,
};

static struct i2c_board_info __initdata rt5640_board_info = {
	I2C_BOARD_INFO("rt5640", 0x1c),
};

#ifdef CONFIG_TOUCHSCREEN_FT5X06
void ft5x06_gpio_reset(void)
{
	/* Pull Low Reset Pin and Turn Off Power */
	gpio_direction_output(GEN3_TOUCH_RESET, 0);
	mdelay (2);

	/* Turn On Power and Pull High Reset Pin */
	gpio_direction_output(GEN3_TOUCH_RESET, 1);
	msleep (200);
}
int ft5x06_touch_power(int on)
{
	static struct regulator *vdd_lcd_hv_r_tsp;
	int ret;
	static int current_state = 0;

	if(on == current_state)
		return 0;

	if(!vdd_lcd_hv_r_tsp)
	{
		vdd_lcd_hv_r_tsp = regulator_get(NULL, "vdd_lcd_hv_r_tsp");
		if (IS_ERR_OR_NULL(vdd_lcd_hv_r_tsp)) {
			pr_err("vdd_lcd_hv_r_tsp regulator get failed\n");
			vdd_lcd_hv_r_tsp = NULL;
			return -EINVAL;
		}
	}

	if(on) {  //power on
		gpio_direction_output(GEN3_TOUCH_RESET, 0);
		msleep(2);

		ret = regulator_enable(vdd_lcd_hv_r_tsp);
		if(ret < 0)
		{
			pr_err("vdd_lcd_hv_r_tsp regulator enable failed\n");
			return ret;
		}
		gpio_direction_output(GEN3_TOUCH_RESET, 0);
		msleep(5);
		gpio_direction_output(GEN3_TOUCH_RESET, 1);
		msleep(200);
	} else {  //power off
		gpio_direction_output(GEN3_TOUCH_RESET, 0);
		msleep (2);

		ret = regulator_disable(vdd_lcd_hv_r_tsp);
		if(ret < 0)
		{
			pr_err("vdd_lcd_hv_r_tsp regulator disable failed\n");
			return ret;
		}
	}

	current_state = on;

	return 0;
}
#endif

static struct i2c_board_info __initdata gen3_i2c_bus2_touch_info[] = {
#ifdef CONFIG_TOUCHSCREEN_FT5X06
       {
               I2C_BOARD_INFO("fts", 0x38),
               .flags = I2C_CLIENT_WAKE,
       },

#endif
};

static void macallan_i2c_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);
	tegra11_i2c_device1.dev.platform_data = &macallan_i2c1_platform_data;
	tegra11_i2c_device2.dev.platform_data = &macallan_i2c2_platform_data;
	tegra11_i2c_device3.dev.platform_data = &macallan_i2c3_platform_data;
	tegra11_i2c_device4.dev.platform_data = &macallan_i2c4_platform_data;
	tegra11_i2c_device5.dev.platform_data = &macallan_i2c5_platform_data;

	platform_device_register(&tegra11_i2c_device5);
	platform_device_register(&tegra11_i2c_device4);
	platform_device_register(&tegra11_i2c_device3);
	platform_device_register(&tegra11_i2c_device2);
	platform_device_register(&tegra11_i2c_device1);

	i2c_register_board_info(0, &rt5640_board_info, 1);
}

static struct platform_device *macallan_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
};
static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "clk_m"},
	[1] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[2] = {.name = "pll_m"},
#endif
};

static struct tegra_uart_platform_data macallan_uart_pdata;
static struct tegra_uart_platform_data macallan_loopback_uart_pdata;

static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = uart_console_debug_init(3);
	if (debug_port_id < 0)
		return;

	macallan_uart_devices[debug_port_id] = uart_console_debug_device;
}

static void __init macallan_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	macallan_uart_pdata.parent_clk_list = uart_parent_clk;
	macallan_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);
	macallan_loopback_uart_pdata.parent_clk_list = uart_parent_clk;
	macallan_loopback_uart_pdata.parent_clk_count =
						ARRAY_SIZE(uart_parent_clk);
	macallan_loopback_uart_pdata.is_loopback = true;
	tegra_uarta_device.dev.platform_data = &macallan_uart_pdata;
	tegra_uartb_device.dev.platform_data = &macallan_uart_pdata;
	tegra_uartc_device.dev.platform_data = &macallan_uart_pdata;
	tegra_uartd_device.dev.platform_data = &macallan_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs())
		uart_debug_init();

	platform_add_devices(macallan_uart_devices,
				ARRAY_SIZE(macallan_uart_devices));
}

static struct resource tegra_rtc_resources[] = {
	[0] = {
		.start = TEGRA_RTC_BASE,
		.end = TEGRA_RTC_BASE + TEGRA_RTC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = INT_RTC,
		.end = INT_RTC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device tegra_rtc_device = {
	.name = "tegra_rtc",
	.id   = -1,
	.resource = tegra_rtc_resources,
	.num_resources = ARRAY_SIZE(tegra_rtc_resources),
};

static struct tegra_asoc_platform_data macallan_audio_pdata = {
	.gpio_spkr_en		= TEGRA_GPIO_SPKR_EN,
	.gpio_hp_det		= TEGRA_GPIO_HP_DET,
	.gpio_hp_mute		= -1,
	.gpio_int_mic_en	= TEGRA_GPIO_INT_MIC_EN,
	.gpio_ext_mic_en	= TEGRA_GPIO_EXT_MIC_EN,
	.gpio_ldo1_en		= TEGRA_GPIO_LDO1_EN,
	.edp_support		= false,
	.edp_states		= {1100, 1100, 0},
	.edp_vol		= {0x8, 0x8, 0x27},
	.gpio_codec1 = TEGRA_GPIO_CODEC1_EN,
	.gpio_codec2 = TEGRA_GPIO_CODEC2_EN,
	.gpio_codec3 = TEGRA_GPIO_CODEC3_EN,
	.i2s_param[HIFI_CODEC]	= {
		.audio_port_id	= 1,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_I2S,
	},
	.i2s_param[BT_SCO]	= {
		.audio_port_id	= 3,
		.is_i2s_master	= 1,
		.i2s_mode	= TEGRA_DAIFMT_DSP_A,
	},
};

static struct platform_device macallan_audio_device = {
	.name	= "tegra-snd-rt5640",
	.id	= 0,
	.dev	= {
		.platform_data = &macallan_audio_pdata,
	},
};

#ifdef CONFIG_QIC_MULTIPLE_MODEM
static struct qic_modem_platform_data qic_modem_device_data = {
	.gpio_power_enable = QIC_MODEM_GPIO_NULL,
	.gpio_host_wakeup_wwan = QIC_MODEM_GPIO_NULL,  /*TEGRA_GPIO_PS1,*/
	.gpio_radio_off = TEGRA_GPIO_PP2,
	.gpio_reset = TEGRA_GPIO_PP0,
	.gpio_onkey = TEGRA_GPIO_PH3,
	.gpio_fatal = QIC_MODEM_GPIO_NULL, /*TEGRA_GPIO_PR7,*/
	.gpio_modem_wakeup_host = TEGRA_GPIO_PV1,
#ifdef QIC_MODEM_SIM_DETECTION_ENHANCEMENT
	.gpio_sim_detection = TEGRA_GPIO_PH7,	/* [I] sim card detection from machinical GPIO_56 low active*/
#else
	.gpio_sim_detection = QIC_MODEM_GPIO_NULL,	/*TEGRA_GPIO_PQ7,*/
#endif
#if 0
	.gpio_radio_off_dummy = TEGRA_GPIO_PK5
#endif
};
static struct platform_device qic_modem_device = {
	.name           = "qic_modem",
	.id             = 0,
	.dev            = {
		.platform_data = &qic_modem_device_data,
	},
};
#endif


static struct platform_device *macallan_devices[] __initdata = {
	&tegra_pmu_device,
	&tegra_rtc_device,
	&tegra_udc_device,
#if defined(CONFIG_TEGRA_IOVMM_SMMU) || defined(CONFIG_TEGRA_IOMMU_SMMU)
	&tegra_smmu_device,
#endif
#if defined(CONFIG_TEGRA_AVP)
	&tegra_avp_device,
#endif
#if defined(CONFIG_CRYPTO_DEV_TEGRA_SE)
	&tegra11_se_device,
#endif
	&tegra_ahub_device,
	&tegra_dam_device0,
	&tegra_dam_device1,
	&tegra_dam_device2,
	&tegra_i2s_device1,
	&tegra_i2s_device3,
	&tegra_i2s_device4,
	&tegra_spdif_device,
	&spdif_dit_device,
	&bluetooth_dit_device,
	&tegra_pcm_device,
	&macallan_audio_device,
	&tegra_hda_device,
#if defined(CONFIG_CRYPTO_DEV_TEGRA_AES)
	&tegra_aes_device,
#endif
#ifdef CONFIG_QIC_MULTIPLE_MODEM
	&qic_modem_device,
#endif
};

#ifdef CONFIG_USB_SUPPORT
static struct tegra_usb_platform_data tegra_udc_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.support_pmu_vbus = true,
	.id_det_type = TEGRA_USB_PMU_ID,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.unaligned_dma_buf_supported = false,
	.op_mode = TEGRA_USB_OPMODE_DEVICE,
	.u_data.dev = {
		.vbus_pmu_irq = 0,
		.vbus_gpio = -1,
		.charging_supported = false,
		.remote_wakeup_supported = false,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 8,
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
	},
};

static struct tegra_usb_platform_data tegra_ehci1_utmi_pdata = {
	.port_otg = true,
	.has_hostpc = true,
	.support_pmu_vbus = true,
	.id_det_type = TEGRA_USB_PMU_ID,
	.unaligned_dma_buf_supported = false,
	.phy_intf = TEGRA_USB_PHY_INTF_UTMI,
	.op_mode = TEGRA_USB_OPMODE_HOST,
	.u_data.host = {
		.vbus_gpio = -1,
		.hot_plug = false,
		.remote_wakeup_supported = true,
		.power_off_on_suspend = true,
		.turn_off_vbus_on_lp0 = true,
	},
	.u_cfg.utmi = {
		.hssync_start_delay = 0,
		.elastic_limit = 16,
		.idle_wait_delay = 17,
		.term_range_adj = 6,
		.xcvr_setup = 15,
		.xcvr_lsfslew = 0,
		.xcvr_lsrslew = 3,
		.xcvr_setup_offset = 0,
		.xcvr_use_fuses = 1,
		.vbus_oc_map = 0x4,
	},
};


static struct tegra_xusb_board_data xusb_bdata = {
	.portmap = TEGRA_XUSB_SS_P0 | TEGRA_XUSB_USB2_P0,
	/* ss_portmap[0:3] = SS0 map, ss_portmap[4:7] = SS1 map */
	.ss_portmap = (TEGRA_XUSB_SS_PORT_MAP_USB2_P0 << 0),
};

static struct tegra_usb_otg_data tegra_otg_pdata = {
	.ehci_device = &tegra_ehci1_device,
	.ehci_pdata = &tegra_ehci1_utmi_pdata,
	.vbus_extcon_dev_name = "palmas-extcon",
	.id_extcon_dev_name = "palmas-extcon",
};

static void macallan_usb_init(void)
{
	struct tegra_xusb_platform_data *xusb_pdata;
	int usb_port_owner_info = tegra_get_usb_port_owner_info();
	printk(KERN_ERR "RYINFO: usb_port_owner_info=0x%x \n",usb_port_owner_info);

	if ((usb_port_owner_info & UTMI1_PORT_OWNER_XUSB)) {
		xusb_pdata = tegra_xusb_init(&xusb_bdata);
		tegra_otg_pdata.is_xhci = true;
		tegra_otg_pdata.xhci_device = &tegra_xhci_device;
		tegra_otg_pdata.xhci_pdata = xusb_pdata;
	} else {
		tegra_otg_pdata.is_xhci = false;
	}
	tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
	platform_device_register(&tegra_otg_device);

	/* Setup the udc platform data */
	tegra_udc_device.dev.platform_data = &tegra_udc_pdata;

}

#else
static void macallan_usb_init(void) { }
#endif

static void macallan_audio_init(void)
{
	struct board_info board_info;

	tegra_get_board_info(&board_info);

	macallan_audio_pdata.codec_name = "rt5640.0-001c";
	macallan_audio_pdata.codec_dai_name = "rt5640-aif1";
}


static struct platform_device *macallan_spi_devices[] __initdata = {
	&tegra11_spi_device1,
};

struct spi_clk_parent spi_parent_clk_macallan[] = {
	[0] = {.name = "pll_p"},
#ifndef CONFIG_TEGRA_PLLM_RESTRICTED
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
#else
	[1] = {.name = "clk_m"},
#endif
};

static struct tegra_spi_platform_data macallan_spi_pdata = {
	.max_dma_buffer         = 16 * 1024,
	.is_clkon_always        = false,
	.max_rate               = 25000000,
};

static void __init macallan_spi_init(void)
{
	int i;
	struct clk *c;
	struct board_info board_info, display_board_info;

	tegra_get_board_info(&board_info);
	tegra_get_display_board_info(&display_board_info);

	for (i = 0; i < ARRAY_SIZE(spi_parent_clk_macallan); ++i) {
		c = tegra_get_clock_by_name(spi_parent_clk_macallan[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
					spi_parent_clk_macallan[i].name);
			continue;
		}
		spi_parent_clk_macallan[i].parent_clk = c;
		spi_parent_clk_macallan[i].fixed_clk_rate = clk_get_rate(c);
	}
	macallan_spi_pdata.parent_clk_list = spi_parent_clk_macallan;
	macallan_spi_pdata.parent_clk_count = ARRAY_SIZE(spi_parent_clk_macallan);
	macallan_spi_pdata.is_dma_based = (tegra_revision == TEGRA_REVISION_A01)
							? false : true ;
	tegra11_spi_device1.dev.platform_data = &macallan_spi_pdata;
	platform_add_devices(macallan_spi_devices,
				ARRAY_SIZE(macallan_spi_devices));
}

struct rm_spi_ts_platform_data rm31080ts_macallan_data = {
	.gpio_reset = TOUCH_GPIO_RST_RAYDIUM_SPI,
	.config = 0,
	.platform_id = RM_PLATFORM_D010,
	.name_of_clock = "clk_out_2",
	.name_of_clock_con = "extern2",
};

static struct tegra_spi_device_controller_data dev_cdata = {
	.rx_clk_tap_delay = 0,
	.tx_clk_tap_delay = 16,
};

struct spi_board_info rm31080a_macallan_spi_board[1] = {
	{
	 .modalias = "rm_ts_spidev",
	 .bus_num = 0,
	 .chip_select = 0,
	 .max_speed_hz = 12 * 1000 * 1000,
	 .mode = SPI_MODE_0,
	 .controller_data = &dev_cdata,
	 .platform_data = &rm31080ts_macallan_data,
	 },
};

static int __init gen3_touch_init(void)
{

	gpio_request(GEN3_TOUCH_RESET, "touch_reset");
	gpio_direction_output(GEN3_TOUCH_RESET, 0);
	mdelay(10);
	gpio_direction_output(GEN3_TOUCH_RESET, 1);
	
	gpio_request(GEN3_TOUCH_IRQ_1, "gen3 touch irq");
	gpio_direction_input(GEN3_TOUCH_IRQ_1);

	gen3_i2c_bus2_touch_info[0].irq = gpio_to_irq(GEN3_TOUCH_IRQ_1);
	i2c_register_board_info(1, gen3_i2c_bus2_touch_info,
		ARRAY_SIZE(gen3_i2c_bus2_touch_info));

	return 0;
}

static int __init gen3_32k_clk_init(void)
{
//* //Johnny : EP5N T40X workaround for wifi and gps clock
	struct clk *clk_cdev3, *clk_out3;
	int res;
	
	clk_cdev3 = clk_get_sys("extern3", NULL);
	clk_out3 = clk_get_sys("clk_out_3", "extern3");
	
	clk_enable(clk_cdev3);
	clk_enable(clk_out3);

	res = clk_get_rate(clk_cdev3);
	res = clk_get_rate(clk_out3);
//*/
	return 0;
}

#define GEN3_CHARGE_LED_R		TEGRA_GPIO_PR5
#define GEN3_CHARGE_LED_G		TEGRA_GPIO_PS0

static void red_led_blinking(struct work_struct *work);
static DECLARE_DELAYED_WORK(red_led_blinking_work, red_led_blinking);

static void red_led_blinking(struct work_struct *work)
{
	static int red_led_state;

	gpio_direction_output(GEN3_CHARGE_LED_R, !red_led_state);
	red_led_state = !red_led_state;
	schedule_delayed_work(&red_led_blinking_work, msecs_to_jiffies(500));
}

void ep5n_a2_set_charge_led(int r_on, int g_on)
{
	static int init;

	if(!init)
	{
		gpio_request(GEN3_CHARGE_LED_R, "charge led red");
		gpio_request(GEN3_CHARGE_LED_G, "charge led green");
		init = 1;
	}
	if(r_on == LED_BLINKING)
	{
		schedule_delayed_work(&red_led_blinking_work, 0);
	}
	else
	{
		cancel_delayed_work_sync(&red_led_blinking_work);
		gpio_direction_output(GEN3_CHARGE_LED_R, r_on);
	}
	gpio_direction_output(GEN3_CHARGE_LED_G, g_on);
}


static void __init tegra_macallan_init(void)
{
	struct board_info board_info;

	macallan_sysedp_init();
	tegra_get_display_board_info(&board_info);
	tegra_clk_init_from_table(macallan_clk_init_table);
	tegra_clk_verify_parents();
	tegra_soc_device_init("macallan");
	tegra_enable_pinmux();
	macallan_pinmux_init();
	macallan_i2c_init();
	macallan_spi_init();
	macallan_usb_init();
	macallan_uart_init();
	macallan_audio_init();
	platform_add_devices(macallan_devices, ARRAY_SIZE(macallan_devices));
	tegra_ram_console_debug_init();
	tegra_io_dpd_init();
	macallan_regulator_init();
	macallan_sdhci_init();
	macallan_suspend_init();
	macallan_emc_init();
	macallan_edp_init();
	gen3_32k_clk_init();
	gen3_touch_init();
	macallan_panel_init();
	macallan_kbc_init();
	macallan_pmon_init();
#if defined CONFIG_TI_ST || defined CONFIG_TI_ST_MODULE
	macallan_bt_st();
	macallan_tegra_setup_st_host_wake();
#endif
#if defined CONFIG_BLUEDROID_PM
       macallan_setup_bluedroid_pm();
#endif
	tegra_release_bootloader_fb();
#ifdef CONFIG_TEGRA_WDT_RECOVERY
	tegra_wdt_recovery_init();
#endif
	tegra_serial_debug_init(TEGRA_UARTD_BASE, INT_WDT_CPU, NULL, -1, -1);
	macallan_sensors_init();
	macallan_soctherm_init();
	tegra_register_fuse();
	macallan_sysedp_core_init();
	macallan_sysedp_psydepl_init();
	tegra_vibrator_init();
}

static void __init macallan_ramconsole_reserve(unsigned long size)
{
	tegra_ram_console_debug_reserve(SZ_1M);
}

static void __init tegra_macallan_dt_init(void)
{
#ifdef CONFIG_USE_OF
	of_platform_populate(NULL,
		of_default_bus_match_table, NULL, NULL);
#endif

	tegra_macallan_init();
}

static void __init tegra_macallan_reserve(void)
{
#if defined(CONFIG_NVMAP_CONVERT_CARVEOUT_TO_IOVMM)
	/* 2560*1600*4*2 = 32768000 bytes */
	tegra_reserve(0, SZ_32M + SZ_2M, SZ_16M);
#else
	tegra_reserve(SZ_128M, SZ_32M + SZ_2M, SZ_4M);
#endif
	macallan_ramconsole_reserve(SZ_1M);
}

static const char * const macallan_dt_board_compat[] = {
	"nvidia,macallan",
	NULL
};

MACHINE_START(MACALLAN, "macallan")
	.atag_offset	= 0x100,
	.soc		= &tegra_soc_desc,
	.map_io		= tegra_map_common_io,
	.reserve	= tegra_macallan_reserve,
	.init_early	= tegra11x_init_early,
	.init_irq	= tegra_init_irq,
	.handle_irq	= gic_handle_irq,
	.timer		= &tegra_timer,
	.init_machine	= tegra_macallan_dt_init,
	.restart	= tegra_assert_system_reset,
	.dt_compat	= macallan_dt_board_compat,
MACHINE_END
