/*
 * arch/arm/mach-tegra/board-dalmore-sensors.c
 *
 * Copyright (c) 2012-2013 NVIDIA CORPORATION, All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of NVIDIA CORPORATION nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mpu.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/therm_est.h>
#include <linux/nct1008.h>
#include <mach/edp.h>
#include <linux/edp.h>
#include <mach/gpio-tegra.h>
#include <mach/pinmux-t11.h>
#include <mach/pinmux.h>
#include <linux/vibrator.h>
#include <linux/i2c/twl.h>
#include <linux/delay.h>
#include <media/imx091.h>
#include <media/imx132.h>
#include <media/ov9772.h>
#include <media/ov5640.h>
#include <media/a1040.h>
#include <media/as364x.h>
#include <media/lm356x.h>
#include <media/ad5816.h>
#include <media/ov5693.h>
#include <media/ad5823.h>
#include <media/imx135.h>
#include <media/imx132.h>
#include <media/ar0833.h>
#include <media/dw9718.h>
#include <media/max77387.h>
#include <generated/mach-types.h>
#include <linux/power/sbs-battery.h>
#include <linux/cm3218.h>

#include "gpio-names.h"
#include "board.h"
#include "board-common.h"
#include "board-dalmore.h"
#include "cpu-tegra.h"
#include "devices.h"
#include "tegra-board-id.h"
#include "dvfs.h"

static struct nvc_gpio_pdata imx091_gpio_pdata[] = {
	{IMX091_GPIO_RESET, CAM_PRI_RSTN, true, false},
	{IMX091_GPIO_PWDN, CAM_PRI_PWRDWN, true, false},
	{IMX091_GPIO_1V8_EN, CAM_1V8_EN, true, false},
	{IMX091_GPIO_2V8_EN, CAM_2V8_EN, true, false},
};

/********************************Vibrator*****************************************/
#define VIBRATOR_EN_GPIO 	TEGRA_GPIO_PH5 //56

static int vibrator_time(int time)
{
	mdelay(time);
	return 0;
}
static int vibrator_init(void)
{
	gpio_request(VIBRATOR_EN_GPIO, "vibrator_en");
	gpio_direction_output(VIBRATOR_EN_GPIO, 0);
	
	return 0;
}
static int vibrator_enable(int isEnable)
{
	gpio_direction_output(VIBRATOR_EN_GPIO,  isEnable);
	return 0;
}

struct omap_vibrator omapVibratorPlatformData = {
	.init = vibrator_init,
	.enable = vibrator_enable,
	.time = vibrator_time,
};

static struct platform_device vib_data = {
	.name		= "vibrator",
	.id		= -1,
	.dev		= {
		.platform_data	= &omapVibratorPlatformData,
	},	
};

int __init tablet_vibrator_init(void)
{
/*  	gpio_request(VIBRATOR_EN_GPIO, "vibrator_en");
	gpio_direction_output(VIBRATOR_EN_GPIO, 1);  */ 
	platform_device_register(&vib_data);
	return 0;
}
/*********************************************************************************/

#ifdef CONFIG_INV_MPU
#include <linux/mpu.h>
#endif

#ifdef CONFIG_INPUT_CAPELLA_CM3218
//If need to power on/off cm3218
//Implement __capella_cm3218_power function and set cm3218_pdata.power point to this function
//static int __capella_cm3218_power(int on, uint8_t val){
//}
#define CM3218_INT_N TEGRA_GPIO_PX3//187 //pinmux, will init in driver

static int __capella_cm3218_power(int on, uint8_t val)
{
	static struct regulator *light_reg;
	int rc=0;

	light_reg = regulator_get(NULL,"vdd_sys_sensor_3v3");
	if(IS_ERR(light_reg)){
		printk(KERN_ERR "get light vaux3 regulator fail\n");
		rc= PTR_ERR(light_reg);
		return rc;
	}
	
	if(on){
		/*rc = regulator_set_voltage(light_reg, 2800000, 2800000);
		if(rc){
			printk(KERN_ERR "set light vaux3 regulator fail\n");
			return rc;
		} */

		printk(KERN_INFO "%s: enable light sensor regulator\n",__func__);
		regulator_enable(light_reg);
		//gpio_direction_output(OMAP4_CM3217_PWR_GPIO,1);
	}else {
		printk(KERN_INFO "%s: disable light sensor regulator\n",__func__);
		regulator_disable(light_reg);
	}

	return 0;
}

static struct cm3218_platform_data cm3218_pdata = {
	.intr = CM3218_INT_N,
	.levels = { 0x0A, 0xA0, 0xE1, 0x140, 0x280, 0x500,
				0xA28, 0x16A8, 0x1F40, 0x2800},
	.power = __capella_cm3218_power,
	.ALS_slave_address = 0x90 >> 1,
	.check_interrupt_add = CM3218_check_INI,
	.is_cmd = CM3218_ALS_SM_2 | CM3218_ALS_IT_250ms | CM3218_ALS_PERS_1 | CM3218_ALS_RES_1,
};

static struct i2c_board_info dalmore_i2c_board_info_cm3218[] = {
	{
		I2C_BOARD_INFO(CM3218_I2C_NAME, 0x90 >> 1),
		.platform_data = &cm3218_pdata,
		.irq = CM3218_INT_N,
	},
};
#endif

static struct board_info board_info;

static struct throttle_table tj_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 792000 } },
	{ {  739500, 528000, 468000, 372000, 792000 } },
	{ {  714000, 528000, 468000, 336000, 792000 } },
	{ {  688500, 528000, 420000, 336000, 792000 } },
	{ {  663000, 492000, 420000, 336000, 792000 } },
	{ {  637500, 492000, 420000, 336000, 408000 } },
	{ {  612000, 492000, 420000, 300000, 408000 } },
	{ {  586500, 492000, 360000, 336000, 408000 } },
	{ {  561000, 420000, 420000, 300000, 408000 } },
	{ {  535500, 420000, 360000, 228000, 408000 } },
	{ {  510000, 420000, 288000, 228000, 408000 } },
	{ {  484500, 324000, 288000, 228000, 408000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle tj_throttle = {
	.throt_tab_size = ARRAY_SIZE(tj_throttle_table),
	.throt_tab = tj_throttle_table,
};

static int __init dalmore_throttle_init(void)
{
	if (machine_is_dalmore())
		balanced_throttle_register(&tj_throttle, "tegra-balanced");
	return 0;
}
module_init(dalmore_throttle_init);

static struct nct1008_platform_data dalmore_nct1008_pdata = {
	.supported_hwrev = true,
	.ext_range = true,
	.conv_rate = 0x06, /* 4Hz conversion rate */
	.shutdown_ext_limit = 105, /* C */
	.shutdown_local_limit = 120, /* C */

	.num_trips = 1,
	.trips = {
		{
			.cdev_type = "suspend_soctherm",
			.trip_temp = 50000,
			.trip_type = THERMAL_TRIP_ACTIVE,
			.upper = 1,
			.lower = 1,
			.hysteresis = 5000,
		},
	},
};

static struct i2c_board_info dalmore_i2c4_nct1008_board_info[] = {
	{
		I2C_BOARD_INFO("nct1008", 0x4C),
		.platform_data = &dalmore_nct1008_pdata,
		.irq = -1,
	}
};

#define VI_PINMUX(_pingroup, _mux, _pupd, _tri, _io, _lock, _ioreset) \
	{							\
		.pingroup	= TEGRA_PINGROUP_##_pingroup,	\
		.func		= TEGRA_MUX_##_mux,		\
		.pupd		= TEGRA_PUPD_##_pupd,		\
		.tristate	= TEGRA_TRI_##_tri,		\
		.io		= TEGRA_PIN_##_io,		\
		.lock		= TEGRA_PIN_LOCK_##_lock,	\
		.od		= TEGRA_PIN_OD_DEFAULT,		\
		.ioreset	= TEGRA_PIN_IO_RESET_##_ioreset	\
}

static int dalmore_focuser_power_on(struct ad5816_power_rail *pw)
{
	int err;

	pr_info("%s\n", __func__);
	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
		return -EFAULT;

	err = regulator_enable(pw->vdd_i2c);
	if (unlikely(err))
		goto ad5816_vdd_i2c_fail;

	err = regulator_enable(pw->vdd);
	if (unlikely(err))
		goto ad5816_vdd_fail;

	return 0;

ad5816_vdd_fail:
	regulator_disable(pw->vdd_i2c);

ad5816_vdd_i2c_fail:
	pr_err("%s FAILED\n", __func__);

	return -ENODEV;
}

static int dalmore_focuser_power_off(struct ad5816_power_rail *pw)
{
	pr_info("%s\n", __func__);
	if (unlikely(WARN_ON(!pw || !pw->vdd || !pw->vdd_i2c)))
		return -EFAULT;

	regulator_disable(pw->vdd);
	regulator_disable(pw->vdd_i2c);

	return 0;
}

static u16 ad5816_devid;
static u16 max77387_devid;
static struct ad5816_platform_data dalmore_imx091_ad5816_pdata;
static struct ad5816_platform_data dalmore_imx135_ad5816_pdata;

static int dalmore_focuser_detect(void *buf, size_t size)
{
	if (!buf)
		return -EFAULT;
	if (size > sizeof(ad5816_devid))
		return -EINVAL;

	ad5816_devid = *(u16 *)buf;
	/* overwrite platform data if module is imx135 */
	if ((ad5816_devid & 0xff) >= 0x6)
		memcpy(&dalmore_imx091_ad5816_pdata,
			&dalmore_imx135_ad5816_pdata,
			sizeof(dalmore_imx091_ad5816_pdata));

	return 0;
}

static int dalmore_flash_detect(void *buf, size_t size)
{
	if (!buf)
		return -EFAULT;
	if (size > sizeof(max77387_devid))
		return -EINVAL;

	max77387_devid = *(u16 *)buf;

	return 0;
}


static struct tegra_pingroup_config mclk_disable =
	VI_PINMUX(CAM_MCLK, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config mclk_enable =
	VI_PINMUX(CAM_MCLK, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_disable =
	VI_PINMUX(GPIO_PBB0, VI, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

static struct tegra_pingroup_config pbb0_enable =
	VI_PINMUX(GPIO_PBB0, VI_ALT3, NORMAL, NORMAL, OUTPUT, DEFAULT, DEFAULT);

/*
 * As a workaround, dalmore_vcmvdd need to be allocated to activate the
 * sensor devices. This is due to the focuser device(AD5816) will hook up
 * the i2c bus if it is not powered up.
*/
static struct regulator *dalmore_vcmvdd;

static int dalmore_get_vcmvdd(void)
{
	if (!dalmore_vcmvdd) {
		dalmore_vcmvdd = regulator_get(NULL, "vdd_af_cam1");
		if (unlikely(WARN_ON(IS_ERR(dalmore_vcmvdd)))) {
			pr_err("%s: can't get regulator vcmvdd: %ld\n",
				__func__, PTR_ERR(dalmore_vcmvdd));
			dalmore_vcmvdd = NULL;
			return -ENODEV;
		}
	}
	return 0;
}

static int dalmore_imx091_power_on(struct nvc_regulator *vreg)
{
	int err;

	pr_err("kokob3: %s ++\n", __func__);
	if (unlikely(WARN_ON(!vreg)))
		return -EFAULT;

	if (dalmore_get_vcmvdd())
		goto imx091_poweron_fail;

	// kokob3: step A + B, pull down PWRDN before power supplies then pull up, will NOT auto reset
	gpio_set_value(CAM_PRI_PWRDWN, 0); // kokob3: step A
	printk("kokob3: CAM_PRI_PWRDWN: %d\n", gpio_get_value(CAM_PRI_PWRDWN));
	usleep_range(10, 20);

	err = regulator_enable(vreg[IMX091_VREG_AVDD].vreg); // 2v8
	if (err)
		goto imx091_avdd_fail;

	printk("kokob3: CAM_2V8_EN: %d\n", gpio_get_value(CAM_2V8_EN));
	gpio_set_value(CAM_2V8_EN, 1);
	printk("kokob3: CAM_2V8_EN: %d\n", gpio_get_value(CAM_2V8_EN));
	err = regulator_enable(vreg[IMX091_VREG_DVDD].vreg); // 1v8
	if (err)
		goto imx091_dvdd_fail;

	printk("kokob3: CAM_1V8_EN: %d\n", gpio_get_value(CAM_1V8_EN));
	gpio_set_value(CAM_1V8_EN, 1);
	printk("kokob3: CAM_1V8_EN: %d\n", gpio_get_value(CAM_1V8_EN));
	usleep_range(1, 2);
	gpio_set_value(CAM_PRI_PWRDWN, 1); // kokob3: step B
	printk("kokob3: CAM_PRI_PWRDWN: %d\n", gpio_get_value(CAM_PRI_PWRDWN));

	err = regulator_enable(dalmore_vcmvdd);
	if (unlikely(err))
		goto imx091_vcmvdd_fail;

	//tegra_pinmux_config_table(&mclk_enable, 1);
	usleep_range(300, 310);

	pr_info("kokob3: %s --\n", __func__);
	return 1;

imx091_vcmvdd_fail:
	regulator_disable(vreg[IMX091_VREG_DVDD].vreg);
	gpio_set_value(CAM_1V8_EN, 0);

imx091_dvdd_fail:
	regulator_disable(vreg[IMX091_VREG_AVDD].vreg);
	gpio_set_value(CAM_2V8_EN, 0);

imx091_avdd_fail:
	gpio_set_value(CAM_PRI_PWRDWN, 0);

imx091_poweron_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int dalmore_imx091_power_off(struct nvc_regulator *vreg)
{
	if (unlikely(WARN_ON(!vreg)))
		return -EFAULT;

	usleep_range(1, 2);
	//tegra_pinmux_config_table(&mclk_disable, 1);
	gpio_set_value(CAM_PRI_PWRDWN, 0);
	usleep_range(1, 2);

	regulator_disable(dalmore_vcmvdd);
	regulator_disable(vreg[IMX091_VREG_DVDD].vreg);
	gpio_set_value(CAM_1V8_EN, 0);
	regulator_disable(vreg[IMX091_VREG_AVDD].vreg);
	gpio_set_value(CAM_2V8_EN, 0);

	return 1;
}

static struct nvc_imager_cap imx091_cap = {
	.identifier		= "IMX091",
	.sensor_nvc_interface	= 3,
	.pixel_types[0]		= 0x100,
	.orientation		= 0,
	.direction		= 0,
	.initial_clock_rate_khz	= 6000,
	.clock_profiles[0] = {
		.external_clock_khz	= 24000,
		.clock_multiplier	= 850000, /* value / 1,000,000 */
	},
	.clock_profiles[1] = {
		.external_clock_khz	= 0,
		.clock_multiplier	= 0,
	},
	.h_sync_edge		= 0,
	.v_sync_edge		= 0,
	.mclk_on_vgp0		= 0,
	.csi_port		= 0,
	.data_lanes		= 4,
	.virtual_channel_id	= 0,
	.discontinuous_clk_mode	= 1,
	.cil_threshold_settle	= 0x0,
	.min_blank_time_width	= 16,
	.min_blank_time_height	= 16,
	.preferred_mode_index	= 0,
	.focuser_guid		= NVC_FOCUS_GUID(0),
	.torch_guid		= NVC_TORCH_GUID(0),
	.cap_version		= NVC_IMAGER_CAPABILITIES_VERSION2,
};

static struct imx091_platform_data imx091_pdata = {
	.num			= 0,
	.sync			= 0,
	.dev_name		= "camera",
	.gpio_count		= ARRAY_SIZE(imx091_gpio_pdata),
	.gpio			= imx091_gpio_pdata,
	.flash_cap		= {
		.sdo_trigger_enabled = 1,
		.adjustable_flash_timing = 1,
	},
	.cap			= &imx091_cap,
	.power_on		= dalmore_imx091_power_on,
	.power_off		= dalmore_imx091_power_off,
};

static int dalmore_imx132_power_on(struct imx132_power_rail *pw)
{
	int err;

	pr_err("kokob3: %s ++\n", __func__);
	if (unlikely(!pw || !pw->avdd || !pw->dvdd))
		return -EFAULT;

	// kokob3: step A + B, pull down PWRDN before power supplies then pull up, will NOT auto reset
	gpio_set_value(CAM_SEC_PWRDWN, 0); // kokob3: step A
	printk("kokob3: CAM_SEC_PWRDWN: %d\n", gpio_get_value(CAM_SEC_PWRDWN));
	usleep_range(10, 20);


	err = regulator_enable(pw->avdd); // 2v8
	if (unlikely(err))
		goto imx132_avdd_fail;

	gpio_set_value(CAM_2V8_EN, 1);
	printk("kokob3: CAM_2V8_EN: %d\n", gpio_get_value(CAM_2V8_EN));

	err = regulator_enable(pw->dvdd); // 1v8
	if (unlikely(err))
		goto imx132_dvdd_fail;

	gpio_set_value(CAM_1V8_EN, 1);
	printk("kokob3: CAM_1V8_EN: %d\n", gpio_get_value(CAM_1V8_EN));
	usleep_range(1, 2);
	gpio_set_value(CAM_SEC_PWRDWN, 1); // kokob3: step B
	printk("kokob3: CAM_SEC_PWRDWN: %d\n", gpio_get_value(CAM_SEC_PWRDWN));

	//tegra_pinmux_config_table(&pbb0_enable, 1);
	usleep_range(300, 310);

	pr_info("kokob3: %s --\n", __func__);
	return 1;

imx132_dvdd_fail:
	regulator_disable(pw->avdd);

imx132_avdd_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int dalmore_imx132_power_off(struct imx132_power_rail *pw)
{
	if (unlikely(!pw || !pw->avdd || !pw->dvdd))
		return -EFAULT;

	usleep_range(21, 25);
	//tegra_pinmux_config_table(&pbb0_disable, 1);

	gpio_set_value(CAM_SEC_PWRDWN, 0);

	regulator_disable(pw->dvdd);
	regulator_disable(pw->avdd);

	/* return 1 to skip the in-driver power_off sequence */
	return 1;
}

struct imx132_platform_data imx132_data = {
	.power_on = dalmore_imx132_power_on,
	.power_off = dalmore_imx132_power_off,
};

static int dalmore_ov5640_power_on(struct ov5640_power_rail *pw)
{
	int err;
	pr_err("qic-cam: %s ++\n", __func__);

	if (unlikely(!pw || !pw->avdd || !pw->dvdd))
		return -EFAULT;

	//vcm power
	if (dalmore_get_vcmvdd())
		goto ov5640_poweron_fail;

	/* power up sequence */
	//power down pin
	gpio_set_value(CAM_PRI_PWRDWN, 1);

	//reset pin
	gpio_set_value(CAM_PRI_RSTN, 0);
	mdelay(1);

	//vcm power
	err = regulator_enable(dalmore_vcmvdd);
	if (unlikely(err))
		goto ov5640_vcmvdd_fail;

	//dvdd_1v8
	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto ov5640_dvdd_fail;

	gpio_set_value(CAM_1V8_EN, 1);

	//avdd_2v8
	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto ov5640_avdd_fail;

	gpio_set_value(CAM_2V8_EN, 1);

	//mclk
	//tegra_pinmux_config_table(&mclk_enable, 1);
	mdelay(5);

	//power down pin
	gpio_set_value(CAM_PRI_PWRDWN, 0);
	mdelay(1);

	//reset pin
	gpio_set_value(CAM_PRI_RSTN, 1);
	mdelay(20);

	pr_err("qic-cam: %s --\n", __func__);
	return 1;

ov5640_avdd_fail:
	gpio_set_value(CAM_1V8_EN, 0);
	regulator_disable(pw->dvdd);

ov5640_dvdd_fail:
	regulator_disable(dalmore_vcmvdd);

ov5640_vcmvdd_fail:
	gpio_set_value(CAM_PRI_PWRDWN, 0);

ov5640_poweron_fail:
	pr_err("%s FAILED\n", __func__);
	return -ENODEV;
}

static int dalmore_ov5640_power_off(struct ov5640_power_rail *pw)
{
	pr_err("qic-cam: %s ++\n", __func__);

	if (unlikely(!pw || !pw->avdd || !pw->dvdd))
		return -EFAULT;

	gpio_set_value(CAM_PRI_RSTN, 0);

	//tegra_pinmux_config_table(&mclk_disable, 1);

	gpio_set_value(CAM_2V8_EN, 0);
	if (pw->avdd)
		regulator_disable(pw->avdd);

	gpio_set_value(CAM_1V8_EN, 0);
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

	if (dalmore_vcmvdd)
		regulator_disable(dalmore_vcmvdd);

	pr_err("qic-cam: %s --\n", __func__);
	return 1;
}

struct ov5640_platform_data ov5640_pdata = {
	.power_on = dalmore_ov5640_power_on,
	.power_off = dalmore_ov5640_power_off,
};

static int dalmore_a1040_power_on(struct a1040_power_rail *pw)
{
	int err;
	pr_err("qic-cam: %s ++\n", __func__);

	if (unlikely(!pw || !pw->avdd || !pw->dvdd))
		return -EFAULT;

	/* power up sequence */
	//power down and reset pin
	gpio_set_value(CAM_SEC_PWRDWN, 0);

	//dvdd_1v8
	err = regulator_enable(pw->dvdd);
	if (unlikely(err))
		goto a1040_dvdd_fail;

	gpio_set_value(CAM_1V8_EN, 1);

	//avdd_2v8
	err = regulator_enable(pw->avdd);
	if (unlikely(err))
		goto a1040_avdd_fail;

	gpio_set_value(CAM_2V8_EN, 1);

	//mclk
	//tegra_pinmux_config_table(&pbb0_enable, 1);
	mdelay(5);

	//power down and reset pin
	gpio_set_value(CAM_SEC_PWRDWN, 1);
	mdelay(1);

	pr_err("qic-cam: %s --\n", __func__);
	return 1;

a1040_avdd_fail:
	gpio_set_value(CAM_1V8_EN, 0);
	regulator_disable(pw->dvdd);

a1040_dvdd_fail:
	pr_err("%s FAILED\n", __func__);
}

static int dalmore_a1040_power_off(struct a1040_power_rail *pw)
{
	pr_err("qic-cam: %s ++\n", __func__);

	if (unlikely(!pw || !pw->avdd || !pw->dvdd))
		return -EFAULT;

	gpio_set_value(CAM_SEC_PWRDWN, 0);

	//tegra_pinmux_config_table(&pbb0_disable, 1);

	gpio_set_value(CAM_2V8_EN, 0);
	if (pw->avdd)
		regulator_disable(pw->avdd);

	gpio_set_value(CAM_1V8_EN, 0);
	if (pw->dvdd)
		regulator_disable(pw->dvdd);

	pr_err("qic-cam: %s --\n", __func__);
	return 1;
}

struct a1040_platform_data a1040_pdata = {
	.power_on = dalmore_a1040_power_on,
	.power_off = dalmore_a1040_power_off,
};

struct sbs_platform_data sbs_pdata = {
	.poll_retry_count = 100,
	.i2c_retry_count = 2,
};

static int dalmore_lm3560_power_on(struct lm356x_power_rail *pw)
{
	int err;

	err = gpio_request(CAM_FLASH_EN, "cam_flash_en");
	if (err < 0){
		pr_err("%s: gpio_request failed for gpio %s: %d\n",
				__func__, "CAM_FLASH_EN", err);
		return err;
	}

	gpio_direction_output(CAM_FLASH_EN, 1);
	if (gpio_get_value(CAM_FLASH_EN) != 1){
		pr_err("%s gpio_setting failed for gpio %s\n",
			__func__,"CAM_FLASH_EN");
	}

	udelay(20);
	return err;
}

static int dalmore_lm3560_power_off(struct lm356x_power_rail *pw)
{
	gpio_set_value(CAM_FLASH_EN, 0);
	if(gpio_get_value(CAM_FLASH_EN) != 0)
		pr_err("%s gpio_setting failed for gpio %s\n",
			__func__,"CAM_FLASH_EN");

	gpio_free(CAM_FLASH_EN);
	return 0;
}

static struct lm356x_platform_data lm3560_pdata = {
	.config		= {
		.max_total_current_mA = 2300,
		.max_peak_current_mA = 1000,
		.vin_low_v_run_mV = 2900,
		.strobe_type = 1,
        .privacy = LM356X_PRIVACY_MODE_ON | LM356X_PRIVACY_NO_BLINK,
		},
	.pinstate	= {
		.mask	= 0x0008,
		.values	= 0x0008
		},
	.dev_name	= "torch",
	.type		= LM3560,
	.gpio_strobe	= CAM_FLASH_PWR,
	.led_mask	= 3,

	.power_on_callback = dalmore_lm3560_power_on,
	.power_off_callback = dalmore_lm3560_power_off,
};

static struct i2c_board_info dalmore_i2c_board_info_qpadgen3_cam[] = {
	{
		I2C_BOARD_INFO("imx132", 0x36),
		.platform_data = &imx132_data,
	},
	{
		I2C_BOARD_INFO("ov5640", 0x3C),
		.platform_data = &ov5640_pdata,
	},
	{
		I2C_BOARD_INFO("a1040", 0x5D),
		.platform_data = &a1040_pdata,
	},
	{
		I2C_BOARD_INFO("lm3560", 0x53),
		.platform_data = &lm3560_pdata,
	},
};

static int dalmore_camera_init(void)
{
	pr_info("kokob3: %s\n", __func__);

	tegra_pinmux_config_table(&mclk_enable, 1);
	tegra_pinmux_config_table(&pbb0_enable, 1);

	i2c_register_board_info(2, dalmore_i2c_board_info_qpadgen3_cam,
		ARRAY_SIZE(dalmore_i2c_board_info_qpadgen3_cam));
	return 0;
}

/* MPU board file definition	*/
#ifdef CONFIG_INV_MPU
static struct mpu_platform_data gyro_platform_data = {
	.int_config = 0x10,
	.level_shifter = 0,
	.orientation = {	0, -1,  0,
						1,  0,  0,
						0,  0,  1 },
	.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id = COMPASS_ID_AK8963,
	.secondary_i2c_addr = 0x0C,	
	.secondary_orientation = { 0, 1, 0,
							   1, 0, 0,
							  0, 0, 1 },
	.key =
    	{ 
	0xdd, 0x16, 0xcd, 0x7, 0xd9, 0xba, 0x97, 0x37, 
 	0xce, 0xfe, 0x23, 0x90, 0xe1, 0x66, 0x2f, 0x32
    	} 
};
#endif

#define TEGRA_CAMERA_GPIO(_gpio, _label, _value)		\
	{							\
		.gpio = _gpio,					\
		.label = _label,				\
		.value = _value,				\
	}

/* static struct i2c_board_info dalmore_i2c_board_info_cm3218[] = {
	{
		I2C_BOARD_INFO("cm3218", 0x48),
	},
}; */

/* MPU board file definition	*/
static struct mpu_platform_data mpu9150_gyro_data = {
	.int_config	= 0x10,
	.level_shifter	= 0,
	/* Located in board_[platformname].h */
	.orientation	= MPU_GYRO_ORIENTATION,
	.sec_slave_type	= SECONDARY_SLAVE_TYPE_NONE,
	.key		= {0x4E, 0xCC, 0x7E, 0xEB, 0xF6, 0x1E, 0x35, 0x22,
			   0x00, 0x34, 0x0D, 0x65, 0x32, 0xE9, 0x94, 0x89},
};

static struct mpu_platform_data mpu_compass_data = {
	.orientation	= MPU_COMPASS_ORIENTATION,
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct mpu_platform_data bmp180_pdata = {
	.config		= NVI_CONFIG_BOOT_MPU,
};

static struct i2c_board_info __initdata inv_mpu9150_i2c2_board_info[] = {
	{
		I2C_BOARD_INFO(MPU_GYRO_NAME, MPU_GYRO_ADDR),
		.platform_data = &gyro_platform_data,
		//.platform_data = &mpu9150_gyro_data,
	},
	{
		/* The actual BMP180 address is 0x77 but because this conflicts
		 * with another device, this address is hacked so Linux will
		 * call the driver.  The conflict is technically okay since the
		 * BMP180 is behind the MPU.  Also, the BMP180 driver uses a
		 * hard-coded address of 0x77 since it can't be changed anyway.
		 */
		I2C_BOARD_INFO("bmp180", 0x78),
		.platform_data = &bmp180_pdata,
	},
	{
		I2C_BOARD_INFO(MPU_COMPASS_NAME, MPU_COMPASS_ADDR),
		.platform_data = &mpu_compass_data,
	},
};

#ifdef CONFIG_INV_MPU
static void mpuirq_init(void)
{
	int ret = 0;
	unsigned gyro_irq_gpio = MPU_GYRO_IRQ_GPIO;
	unsigned gyro_bus_num = MPU_GYRO_BUS_NUM;
	char *gyro_name = MPU_GYRO_NAME;

	pr_info("*** MPU START *** mpuirq_init...\n");

	ret = gpio_request(gyro_irq_gpio, gyro_name);

	if (ret < 0) {
		pr_err("%s: gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(gyro_irq_gpio);
	if (ret < 0) {
		pr_err("%s: gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(gyro_irq_gpio);
		return;
	}
	pr_info("*** MPU END *** mpuirq_init...\n");

	inv_mpu9150_i2c2_board_info[0].irq = gpio_to_irq(MPU_GYRO_IRQ_GPIO);
	i2c_register_board_info(gyro_bus_num, inv_mpu9150_i2c2_board_info,
		ARRAY_SIZE(inv_mpu9150_i2c2_board_info));
}
#endif

#ifdef CONFIG_INPUT_CAPELLA_CM3218
static void cm3218irq_init(void)
{
	int ret = 0;
	unsigned als_irq_gpio = CM3218_INT_N;
	unsigned als_bus_num = MPU_GYRO_BUS_NUM;
	char *als_name = CM3218_NAME;

	pr_info("*** ALS START *** alsirq_init...\n");

	ret = gpio_request(als_irq_gpio, als_name);

	if (ret < 0) {
		pr_err("%s: als gpio_request failed %d\n", __func__, ret);
		return;
	}

	ret = gpio_direction_input(als_irq_gpio);
	if (ret < 0) {
		pr_err("%s: als gpio_direction_input failed %d\n", __func__, ret);
		gpio_free(als_irq_gpio);
		return;
	}
	pr_info("*** ALS END *** alsirq_init...\n");

	dalmore_i2c_board_info_cm3218[0].irq = gpio_to_irq(CM3218_INT_N);
}
#endif

/* #ifdef CONFIG_INV_MPU
static void mpuirq_init(void)
{
	int res;
	
	pr_info("*** MPU START *** mpuirq_init...\n");
	if ((res = gpio_request(MPUIRQ_GPIO, "mpuirq")))
		printk(KERN_ERR "myirqtest: gpio_request(%d) failed err=%d\n",MPUIRQ_GPIO, res);
	if ((res = gpio_direction_input(MPUIRQ_GPIO)))
		printk(KERN_ERR "gpio_direction_input() failed err=%d\n",res);
	pr_info("*** MPU END *** mpuirq_init...\n");
}
#endif */

static int __init dalmore_nct1008_init(void)
{
	int nct1008_port;
	int ret = 0;

	if ((board_info.board_id == BOARD_E1612) ||
	    (board_info.board_id == BOARD_E1641) ||
	    (board_info.board_id == BOARD_E1613) ||
	    (board_info.board_id == BOARD_P2454))
	{
		/* per email from Matt 9/10/2012 */
		nct1008_port = TEGRA_GPIO_PX6;
	} else if (board_info.board_id == BOARD_E1611) {
		/*
		if (board_info.fab == 0x04)
			nct1008_port = TEGRA_GPIO_PO4;
		else
		*/
			nct1008_port = TEGRA_GPIO_PX6;
	} else {
		nct1008_port = TEGRA_GPIO_PX6;
		pr_err("Warning: nct alert_port assumed TEGRA_GPIO_PX6"
			" for unknown dalmore board id E%d\n",
			board_info.board_id);
	}

	tegra_add_cdev_trips(dalmore_nct1008_pdata.trips,
				&dalmore_nct1008_pdata.num_trips);

	dalmore_i2c4_nct1008_board_info[0].irq = gpio_to_irq(nct1008_port);
	pr_info("%s: dalmore nct1008 irq %d",
			__func__, dalmore_i2c4_nct1008_board_info[0].irq);

	ret = gpio_request(nct1008_port, "temp_alert");
	if (ret < 0)
		return ret;

	ret = gpio_direction_input(nct1008_port);
	if (ret < 0) {
		pr_info("%s: calling gpio_free(nct1008_port)", __func__);
		gpio_free(nct1008_port);
	}

	/* dalmore has thermal sensor on GEN1-I2C i.e. instance 0 */
	i2c_register_board_info(0, dalmore_i2c4_nct1008_board_info,
		ARRAY_SIZE(dalmore_i2c4_nct1008_board_info));

	return ret;
}

static struct i2c_board_info __initdata bq20z45_pdata[] = {
	{
		I2C_BOARD_INFO("sbs-battery", 0x0B),
		.platform_data = &sbs_pdata,
	},
};

#ifdef CONFIG_TEGRA_SKIN_THROTTLE
static struct thermal_trip_info skin_trips[] = {
	{
		.cdev_type = "skin-balanced",
		.trip_temp = 45000,
		.trip_type = THERMAL_TRIP_PASSIVE,
		.upper = THERMAL_NO_LIMIT,
		.lower = THERMAL_NO_LIMIT,
		.hysteresis = 0,
	},
};

static struct therm_est_subdevice skin_devs[] = {
	{
		.dev_data = "Tdiode",
		.coeffs = {
			2, 1, 1, 1,
			1, 1, 1, 1,
			1, 1, 1, 0,
			1, 1, 0, 0,
			0, 0, -1, -7
		},
	},
	{
		.dev_data = "Tboard",
		.coeffs = {
			-11, -7, -5, -3,
			-3, -2, -1, 0,
			0, 0, 1, 1,
			1, 2, 2, 3,
			4, 6, 11, 18
		},
	},
};

static struct therm_est_data skin_data = {
	.num_trips = ARRAY_SIZE(skin_trips),
	.trips = skin_trips,
	.toffset = 9793,
	.polling_period = 1100,
	.passive_delay = 15000,
	.tc1 = 10,
	.tc2 = 1,
	.ndevs = ARRAY_SIZE(skin_devs),
	.devs = skin_devs,
};

static struct throttle_table skin_throttle_table[] = {
	/* CPU_THROT_LOW cannot be used by other than CPU */
	/*      CPU,  C2BUS,  C3BUS,   SCLK,    EMC   */
	{ { 1810500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1785000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1759500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1734000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1708500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1683000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1657500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1632000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1606500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1581000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1555500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1530000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1504500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1479000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1453500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1428000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1402500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1377000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1351500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1326000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1300500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1275000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1249500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1224000, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1198500, NO_CAP, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1173000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1147500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1122000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1096500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1071000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1045500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ { 1020000, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  994500, 636000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  969000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  943500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  918000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  892500, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  867000, 600000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  841500, 564000, NO_CAP, NO_CAP, NO_CAP } },
	{ {  816000, 564000, NO_CAP, NO_CAP, 792000 } },
	{ {  790500, 564000, NO_CAP, 372000, 792000 } },
	{ {  765000, 564000, 468000, 372000, 792000 } },
	{ {  739500, 528000, 468000, 372000, 792000 } },
	{ {  714000, 528000, 468000, 336000, 792000 } },
	{ {  688500, 528000, 420000, 336000, 792000 } },
	{ {  663000, 492000, 420000, 336000, 792000 } },
	{ {  637500, 492000, 420000, 336000, 408000 } },
	{ {  612000, 492000, 420000, 300000, 408000 } },
	{ {  586500, 492000, 360000, 336000, 408000 } },
	{ {  561000, 420000, 420000, 300000, 408000 } },
	{ {  535500, 420000, 360000, 228000, 408000 } },
	{ {  510000, 420000, 288000, 228000, 408000 } },
	{ {  484500, 324000, 288000, 228000, 408000 } },
	{ {  459000, 324000, 288000, 228000, 408000 } },
	{ {  433500, 324000, 288000, 228000, 408000 } },
	{ {  408000, 324000, 288000, 228000, 408000 } },
};

static struct balanced_throttle skin_throttle = {
	.throt_tab_size = ARRAY_SIZE(skin_throttle_table),
	.throt_tab = skin_throttle_table,
};

static int __init dalmore_skin_init(void)
{
	if (machine_is_dalmore()) {
		balanced_throttle_register(&skin_throttle, "skin-balanced");
		tegra_skin_therm_est_device.dev.platform_data = &skin_data;
		platform_device_register(&tegra_skin_therm_est_device);
	}

	return 0;
}
late_initcall(dalmore_skin_init);
#endif

int __init dalmore_sensors_init(void)
{
	int err;

	tegra_get_board_info(&board_info);

	err = dalmore_nct1008_init();
	if (err)
		return err;

	dalmore_camera_init();
	mpuirq_init();
	cm3218irq_init();
	
#if defined(CONFIG_DRV8601_VIB)
	tablet_vibrator_init();
#endif

	i2c_register_board_info(2, dalmore_i2c_board_info_cm3218,
		ARRAY_SIZE(dalmore_i2c_board_info_cm3218));

	i2c_register_board_info(0, bq20z45_pdata,
		ARRAY_SIZE(bq20z45_pdata));

	return 0;
}

static int dalmore_chk_conflict(struct device *dev, void *addrp)
{
	struct i2c_client	*client = i2c_verify_client(dev);
	unsigned short		addr = *(unsigned short *)addrp;

	if (!client)
		return 0;

	pr_info("%s: %s %x - %x\n", __func__, client->name, client->addr, addr);
	if (client->addr == addr)
		i2c_unregister_device(client);
	return 0;
}

int camera_auto_detect(void)
{
#if 0
	struct i2c_adapter *adap = i2c_get_adapter(2);

	pr_info("%s ++ %04x - %04x - %04x\n", __func__,
		ad5816_devid, dw9718_devid, max77387_devid);

	if ((ad5816_devid & 0xff00) == 0x2400) {
		if ((max77387_devid & 0xff) == 0x91) {
			/* IMX135 found */
			i2c_new_device(adap, &dalmore_i2c_board_info_imx135);
			i2c_new_device(adap, &dalmore_i2c_board_info_imx132);
		} else {
			/* remove max77387 from system */
			device_for_each_child(&adap->dev,
				&dalmore_i2c_board_info_e1625[2].addr,
				dalmore_chk_conflict);
			/* IMX091 found*/
			i2c_new_device(adap, &dalmore_i2c_board_info_imx091);
			i2c_new_device(adap, &dalmore_i2c_board_info_ov9772);
			i2c_new_device(adap, &dalmore_i2c_board_info_as3648);
		}
	} else {
		/* remove ad5816 from system */
		device_for_each_child(&adap->dev,
			&dalmore_i2c_board_info_e1625[0].addr,
			dalmore_chk_conflict);
		/* remove max77387 from system */
		device_for_each_child(&adap->dev,
			&dalmore_i2c_board_info_e1625[2].addr,
			dalmore_chk_conflict);
		if (dw9718_devid) {
			/* AR0833 found */
			i2c_new_device(adap, &dalmore_i2c_board_info_ar0833);
			i2c_new_device(adap, &dalmore_i2c_board_info_as3648);
			i2c_new_device(adap, &dalmore_i2c_board_info_ov9772);
		} else { /* default using ov5693 + ad5823 */
			device_for_each_child(&adap->dev,
				&dalmore_i2c_board_info_e1625[1].addr,
				dalmore_chk_conflict);
			i2c_new_device(adap, &dalmore_i2c_board_info_ov5693);
			i2c_new_device(adap, &dalmore_i2c_board_info_ad5823);
			i2c_new_device(adap, &dalmore_i2c_board_info_as3648);
		}
	}

	pr_info("%s --\n", __func__);
#endif
	return 0;
}

int __init dalmore_camera_late_init(void)
{
	if (board_info.board_id != BOARD_E1611) {
		pr_err("%s: Dalmore not found!\n", __func__);
		return 0;
	}

	camera_auto_detect();

	return 0;
}

late_initcall(dalmore_camera_late_init);
