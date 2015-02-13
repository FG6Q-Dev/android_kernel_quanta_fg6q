/*
* LM356X.c - LM356X flash/torch kernel driver
*
* Copyright (c) 2012-2013, NVIDIA CORPORATION.  All rights reserved.

* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.

* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.

* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include<linux/kthread.h>
#include <media/nvc.h>
#include <media/lm356x.h>
#include <linux/max17048_battery.h>

#define lm356x_max_flash_cap_size	(sizeof(u32) \
	+ (sizeof(struct nvc_torch_level_info) \
	* (LM356X_MAX_FLASH_LEVEL)))
#define lm356x_max_torch_cap_size	(sizeof(u32) \
	+ (sizeof(s32) * (LM356X_MAX_TORCH_LEVEL)))

struct lm356x_caps_struct {
	char *name;
	u32 flash_curr_step_uA;
	u32 torch_curr_step_uA;
	u32 num_regs;
	u32 max_peak_curr_mA;
	u32 min_ilimit_mA;
	u32 max_assist_curr_mA;
	u32 max_indicator_curr_mA;
	bool led2_support;
};

struct lm356x_reg_cache {
	u8 enable;
	u8 privacy;
	u8 vin_monitor;
	u8 torch_brightness;
	u8 flash_brightness;
	u8 ftime;
	u8 config_1;
	u8 config_2;
};

struct lm356x_reg {
	u8 addr;
	u8 val;
};

struct lm356x_reg lm3560_reg_table_default[] = {
	{LM356X_REG_ENABLE,             		0x18},
	{LM356X_REG_PRIVACY,            		0x58},
	{LM356X_REG_INDICATOR,         	0x00},
	{LM356X_REG_INDICATOR_BLINK,    	0x00},
	{LM356X_REG_PRIVACY_PWM,        	0xF8},
	{LM356X_REG_GPIO,              		0x80},
	{LM356X_REG_VLED_MONITOR,       	0x80},
	{LM356X_REG_ADC_DELAY,          	0x90},
	{LM356X_REG_VIN_MONITOR,        	0xC0},
	{LM356X_REG_LAST_FLASH,         	0x00},
	{LM356X_REG_TORCH_BRIGHTNESS,  0x52},
	{LM356X_REG_FLASH_BRIGHTNESS,   0xDD},
	{LM356X_REG_FLASH_DURATION,     	0xEF},
	{LM356X_REG_FLAGS,              		0x00},
	{LM356X_REG_CONFIG_1,           	0x6B},
	{LM356X_REG_CONFIG_2,           	0xE0},
};

static const struct lm356x_caps_struct lm356x_caps[] = {
	{"lm3560", 62500, 31250, 16, 1000, 1600, 250, 250, true},
};

/* translated from the default register values after power up */
const struct lm356x_config lm356x_default_cfg = {
	.tx_mask = 0,
	.I_limit_mA = 3000,
	.vin_low_v_run_mV = 3000,
	.vin_low_v_mV = 2900,
	.strobe_type = 2,
	.led_off_when_vin_low = 0,
	.max_peak_current_mA = 1000,
	.max_sustained_current_mA = 0,
	.max_peak_duration_ms = 0,
	.min_current_mA = 0,
};

struct lm356x_info {
	struct i2c_client *i2c_client;
	struct miscdevice miscdev;
	struct dentry *d_lm356x;
	struct list_head list;
	struct lm356x_info *s_info;
	struct mutex mutex;
	struct regulator *v_in;
	struct lm356x_power_rail power;
	struct lm356x_platform_data *pdata;
	struct nvc_torch_flash_capabilities *flash_cap;
	struct nvc_torch_torch_capabilities *torch_cap;
	struct lm356x_caps_struct caps;
	struct lm356x_config config;
	struct lm356x_reg_cache regs;
	atomic_t in_use;
	int flash_cap_size;
	int torch_cap_size;
	int pwr_state;
	u8 s_mode;
	u8 flash_mode;
	u8 led_num;
	u8 led_mask;
	u8 strobe_type;
	bool shutdown_complete;
};

static struct lm356x_platform_data lm356x_default_pdata = {
	.cfg		= 0,
	.num		= 0,
	.sync		= 0,
	.dev_name	= "torch",
	.pinstate	= {0x0000, 0x0000},
	.led_mask	= 3,
};

static const struct i2c_device_id lm356x_id[] = {
	{ "lm3560", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, lm356x_id);

static LIST_HEAD(lm356x_info_list);
static DEFINE_SPINLOCK(lm356x_spinlock);

static const u16 v_in_low[] = {2900, 3000, 3100, 3200};

static int lm356x_debugfs_init(struct lm356x_info *info);
static int lm356x_power_off(struct lm356x_info *info);

static int lm356x_reg_rd(struct lm356x_info *info, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	mutex_lock(&info->mutex);

	if (info && info->shutdown_complete) {
		mutex_unlock(&info->mutex);
		return -EINVAL;
	}

	*val = 0;
	msg[0].addr = info->i2c_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = info->i2c_client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = val;
	if (i2c_transfer(info->i2c_client->adapter, msg, 2) != 2) {
		mutex_unlock(&info->mutex);
		return -EIO;
	}

	mutex_unlock(&info->mutex);
	return 0;
}

static int lm356x_reg_raw_wr(struct lm356x_info *info, u8 *buf, u8 num)
{
	struct i2c_msg msg;
	mutex_lock(&info->mutex);

	if (info && info->shutdown_complete) {
		mutex_unlock(&info->mutex);
		return -EINVAL;
	}

	msg.addr = info->i2c_client->addr;
	msg.flags = 0;
	msg.len = num;
	msg.buf = buf;
	if (i2c_transfer(info->i2c_client->adapter, &msg, 1) != 1) {
		mutex_unlock(&info->mutex);
		return -EIO;
	}

	dev_dbg(&info->i2c_client->dev, "%s %x %x\n", __func__, buf[0], buf[1]);
	mutex_unlock(&info->mutex);
	return 0;
}

static int lm356x_reg_wr(struct lm356x_info *info, u8 reg, u8 val)
{
	u8 buf[2];

	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	buf[0] = reg;
	buf[1] = val;
	return lm356x_reg_raw_wr(info, buf, sizeof(buf));
}

static struct task_struct *strobe_delay_thread = NULL;
static int Torch_on_flag = -1;
static int strobe_on_flag = -1;

static int strobe_delay(void * data)
{
	msleep(2000);
	Torch_on_flag = 1;
	return 0;
}

static int lm356x_set_leds(struct lm356x_info *info, u8 led_mask,
	u8 strobe_type, u8 curr_torch, u8 curr_flash)
{
	int err;
	u8 val_en, val_torch, val_flash, val_flash_duration, val_config_1;

	/*Torch off or Strobe off*/
	err = lm356x_reg_rd(info, LM356X_REG_ENABLE, &val_en);
	if (err) {
		dev_err(&info->i2c_client->dev, "%s: read 0x%x fail!!!\n",
			__func__, LM356X_REG_ENABLE);
		return err;
	}

	if (info->pdata->gpio_strobe)
	{
		err = lm356x_reg_rd(info, LM356X_REG_CONFIG_1, &val_config_1);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s: read 0x%x fail!!!\n",
				__func__, LM356X_REG_CONFIG_1);
			return err;
		}
	}
	if ((info->flash_mode == LM356X_REG_CONTROL_MODE_SHUTDOWN) 
		|| (curr_torch == 0) || (curr_flash == 0)) {
			if (info->pdata->gpio_strobe)
			{
				val_config_1 &= ~(0x04);
				err |= lm356x_reg_wr(info, LM356X_REG_CONFIG_1, val_config_1);
			}
			val_en &= (~0x03);
			err |= lm356x_reg_wr(info, LM356X_REG_ENABLE, val_en);

			return err;
	}

	/*Torch on*/
	if (info->flash_mode == LM356X_REG_CONTROL_MODE_ASSIST)
	{
		if (Torch_on_flag == 1 && max17048_check_voltage_for_flash())
		{
			err = lm356x_reg_rd(info, LM356X_REG_TORCH_BRIGHTNESS, &val_torch);
			if (err)
			{
				dev_err(&info->i2c_client->dev, "%s: read 0x%x fail!!!\n",
					__func__, LM356X_REG_TORCH_BRIGHTNESS);
				return err;
			}
			val_en &= ~(1<<5);
			val_en |= strobe_type == 2 ? (0<<5) : (1<<5);
			val_en &= (~0x03);
			val_en |= info->flash_mode;

			//Check LED1 ON/OFF
			if (led_mask & 1) {
				val_en |= 1<<3;
				val_torch &= ~(0x07);
				val_torch |= (curr_torch-1);
			} else {
				val_en &= ~(1<<3);
				val_torch &= ~(0x07);
			}

			//Check LED2 ON/OFF
			if (led_mask & 2 && info->caps.led2_support) {
				val_en |= 1<<4;
				val_torch &= ~(0x07<<3);
				val_torch |= (curr_torch-1) << 3;
			} else {
				val_en &= ~(1<<4);
				val_torch &= ~(0x07<<3);
			}

			//Set torch brightness
			err |= lm356x_reg_wr(info, LM356X_REG_TORCH_BRIGHTNESS, val_torch);

			//Enable torch on
			err |= lm356x_reg_wr(info, LM356X_REG_ENABLE, val_en);

			strobe_on_flag = 1;
		}
		else
		{
			strobe_on_flag = 0;
		}
	}

	/*Strobe on*/
	if (info->flash_mode == LM356X_REG_CONTROL_MODE_FLASH)
	{
		if (Torch_on_flag == 1 && strobe_on_flag == 1 )
		{
			Torch_on_flag = 0;

			err = lm356x_reg_rd(info, LM356X_REG_FLASH_BRIGHTNESS, &val_flash);
			if (err) {
				dev_err(&info->i2c_client->dev, "%s: read 0x%x fail!!!\n",
					__func__, LM356X_REG_FLASH_BRIGHTNESS);
				return err;
			}
			val_en &= ~(1<<5);
			val_en |= strobe_type == 2 ? (0<<5) : (1<<5);
			val_en &= (~0x03);
			val_en |= info->flash_mode;

			//Check LED1 ON/OFF
			if (led_mask & 1) {
				val_en |= 1<<3;
				val_flash &= ~(0x0F);
				val_flash |= curr_flash-1;
			} else {
				val_en &= ~(1<<3);
				val_flash &= ~(0x0F);
			}

			//Check LED2 ON/OFF
			if (led_mask & 2 && info->caps.led2_support) {
				val_en |= 1<<4;
				val_flash &= ~(0x0F<<4);
				val_flash |= (curr_flash-1) << 4;
			} else {
				val_en &= ~(1<<4);
				val_flash &= ~(0x0F<<4);
			}

			//Set strobe brightness
			err |= lm356x_reg_wr(info, LM356X_REG_FLASH_BRIGHTNESS, val_flash);

			//Set strobe duration
			err |= lm356x_reg_rd(info, LM356X_REG_FLASH_DURATION, &val_flash_duration);
			if (err) {
				dev_err(&info->i2c_client->dev, "%s: read 0x%x fail!!!\n",
					__func__, LM356X_REG_FLASH_DURATION);
				return err;
			}
			val_flash_duration &= ~(0x7F);
			val_flash_duration |= info->regs.ftime;
			err |= lm356x_reg_wr(info, LM356X_REG_FLASH_DURATION, val_flash_duration);

			//Enable strobe on
			if(info->pdata->gpio_strobe)
			{
				val_en &= (~0x03);
				err |= lm356x_reg_wr(info, LM356X_REG_ENABLE, val_en);
			}
			val_config_1 |= (0x04);
			err |= lm356x_reg_wr(info, LM356X_REG_CONFIG_1, val_config_1);

			//Create a thread of strobe delay
			strobe_delay_thread = kthread_create(&strobe_delay,NULL,"strobe_delay_thread");
			if (IS_ERR(strobe_delay_thread))
			{
				printk("Strobe_delay_thread creation error\n");
				strobe_delay_thread = NULL;
			}
			err = wake_up_process(strobe_delay_thread);
		}
	}

	return err;
}

static int lm356x_set_privacy(struct lm356x_info *info)
{
	struct lm356x_config *p_cfg = &info->config;
	int err;
	u8 val;

	err = lm356x_reg_rd(info, LM356X_REG_PRIVACY, &val);
	if (err) {
		dev_err(&info->i2c_client->dev, "%s: read 0x%x fail!!!\n",
			__func__, LM356X_REG_PRIVACY);
		return err;
	}

	val &= ~(LM356X_PRIVACY_MODE_MASK | LM356X_PRIVACY_BLINK_MASK);
	val |= p_cfg->privacy;

	err |= lm356x_reg_wr(info, LM356X_REG_PRIVACY, val);

	return err;
}

#if 0 // Need implement
static int lm356x_set_txmask(struct lm356x_info *info)
{
	struct lm356x_config *p_cfg = &info->config;
	int err;
	u8 val;

	err = lm356x_reg_rd(info, LM356X_REG_GPIO, &val);
	if (err) {
		dev_err(&info->i2c_client->dev, "%s: read 0x%x fail!!!\n",
			__func__, LM356X_REG_GPIO);
		return err;
	}

	/* We use TX1/TX2 as default configuration */
	dev_dbg(&info->i2c_client->dev, "%s: tx_mask %d reg 0x%x is 0x%x\n",
		__func__, p_cfg->tx_mask, LM356X_REG_GPIO, val);

	return err;
}
#endif

static int lm356x_get_vin_index(u16 mV)
{
	int vin;

	for (vin = ARRAY_SIZE(v_in_low) - 1; vin >= 0; vin--) {
		if (mV >= v_in_low[vin])
			break;
	}

	return vin;
}

static void lm356x_config_init(struct lm356x_info *info)
{
	struct lm356x_config *pcfg = &info->config;
	struct lm356x_config *pcfg_cust = &info->pdata->config;

	memcpy(pcfg, &lm356x_default_cfg, sizeof(info->config));

	pcfg->tx_mask = pcfg_cust->tx_mask;
	pcfg->load_balance_on = pcfg_cust->load_balance_on;
	pcfg->led_off_when_vin_low = pcfg_cust->led_off_when_vin_low;
	pcfg->boost_mode = pcfg_cust->boost_mode;

	if (pcfg_cust->strobe_type)
		pcfg->strobe_type = pcfg_cust->strobe_type;

	if (pcfg_cust->vin_low_v_run_mV) {
		if (pcfg_cust->vin_low_v_run_mV == 0xffff)
			pcfg->vin_low_v_run_mV = 0;
		else
			pcfg->vin_low_v_run_mV = pcfg_cust->vin_low_v_run_mV;
	}

	if (pcfg_cust->vin_low_v_mV) {
		if (pcfg_cust->vin_low_v_mV == 0xffff)
			pcfg->vin_low_v_mV = 0;
		else
			pcfg->vin_low_v_mV = pcfg_cust->vin_low_v_mV;
	}

	if (pcfg_cust->I_limit_mA)
		pcfg->I_limit_mA = pcfg_cust->I_limit_mA;

	if (pcfg_cust->max_total_current_mA)
		pcfg->max_total_current_mA = pcfg_cust->max_total_current_mA;

	if (pcfg_cust->max_peak_current_mA)
		pcfg->max_peak_current_mA = pcfg_cust->max_peak_current_mA;

	if (pcfg_cust->max_peak_duration_ms)
		pcfg->max_peak_duration_ms = pcfg_cust->max_peak_duration_ms;

	if (pcfg_cust->max_sustained_current_mA)
		pcfg->max_sustained_current_mA =
		pcfg_cust->max_sustained_current_mA;

	if (pcfg_cust->min_current_mA)
		pcfg->min_current_mA = pcfg_cust->min_current_mA;

}

static int lm356x_update_settings(struct lm356x_info *info)
{
	int err;

	err = lm356x_set_leds(info, info->led_mask, info->strobe_type,
		info->regs.torch_brightness, info->regs.flash_brightness);

	err |= lm356x_set_privacy(info);

	err |= lm356x_reg_wr(info, LM356X_REG_VIN_MONITOR, info->regs.vin_monitor);

	err |= lm356x_reg_wr(info, LM356X_REG_CONFIG_1, info->regs.config_1);

	err |= lm356x_reg_wr(info, LM356X_REG_CONFIG_2, info->regs.config_2);

	return 0;
}

static int lm356x_configure(struct lm356x_info *info, bool update)
{
	struct lm356x_config *pcfg = &info->config;
	struct lm356x_caps_struct *pcap = &info->caps;
	struct nvc_torch_flash_capabilities *pfcap = info->flash_cap;
	struct nvc_torch_torch_capabilities *ptcap = info->torch_cap;
	int val;
	int i;

	if (!pcap->led2_support)
		pcfg->boost_mode = false;

	val = lm356x_get_vin_index(pcfg->vin_low_v_run_mV);
	info->regs.vin_monitor = 0xC0 | (val<<4);

	val = lm356x_get_vin_index(pcfg->vin_low_v_mV);
	info->regs.vin_monitor |= val<<1;

	info->regs.config_1 = 0x6B & ~(1<<5);

	if (pcfg->led_off_when_vin_low)
		info->regs.config_2 = 0xE0 | (1<<3);

	info->led_mask = info->pdata->led_mask;
	info->strobe_type = pcfg->strobe_type;
	info->regs.ftime = DEFAULT_FLASHTIME;

	if (pcfg->max_peak_current_mA > pcap->max_peak_curr_mA ||
		!pcfg->max_peak_current_mA) {
			dev_warn(&info->i2c_client->dev,
				"max_peak_current_mA of %d invalid"
				"changing to %d\n",
				pcfg->max_peak_current_mA,
				pcap->max_peak_curr_mA);
			pcfg->max_peak_current_mA = pcap->max_peak_curr_mA;
	}

	info->led_num = 1;
	if (pcap->led2_support && (info->led_mask & 3) == 3)
		info->led_num = 2;

	val = pcfg->max_peak_current_mA * info->led_num;

	if (!pcfg->max_total_current_mA || pcfg->max_total_current_mA > val)
		pcfg->max_total_current_mA = val;
	pcfg->max_peak_current_mA =
		info->config.max_total_current_mA / info->led_num;

	if (pcfg->max_total_current_mA <= 1600) {
		info->regs.ftime &= ~(0x60);
		info->regs.ftime |= 0x00;
	} else if (pcfg->max_total_current_mA <= 2300) {
		info->regs.ftime &= ~(0x60);
		info->regs.ftime |= 0x20;
	} else if (pcfg->max_total_current_mA <= 3000) {
		info->regs.ftime &= ~(0x60);
		info->regs.ftime |= 0x40;
	} else {
		info->regs.ftime &= ~(0x60);
		info->regs.ftime |= 0x60;
	}

	if (pcfg->max_sustained_current_mA > pcap->max_assist_curr_mA ||
		!pcfg->max_sustained_current_mA) {
			dev_warn(&info->i2c_client->dev,
				"max_sustained_current_mA of %d invalid"
				"changing to %d\n",
				pcfg->max_sustained_current_mA,
				pcap->max_assist_curr_mA);
			pcfg->max_sustained_current_mA =
				pcap->max_assist_curr_mA;
	}

	dev_dbg(&info->i2c_client->dev, "%s: flash cap[%d] = {\n",
		__func__, LM356X_MAX_FLASH_LEVEL);
	val = pcap->flash_curr_step_uA;

	for (i = 0; i < LM356X_MAX_FLASH_LEVEL; i++) {
		pfcap->levels[i].guidenum = val * i / 1000;
		if (pfcap->levels[i].guidenum >
			pcfg->max_peak_current_mA) {
				pfcap->levels[i].guidenum = 0;
				break;
		}
		if (pfcap->levels[i].guidenum > 0) {
			pfcap->levels[i].sustaintime = SUSTAINTIME_DEF;
			pfcap->levels[i].rechargefactor = RECHARGEFACTOR_DEF;
		} else {
			pfcap->levels[i].sustaintime = 0xFFFFFFFF;
			pfcap->levels[i].rechargefactor = 0;
		}
		dev_dbg(&info->i2c_client->dev, "%s:    {%d, %d, %d},\n",
			__func__, pfcap->levels[i].guidenum, pfcap->levels[i].sustaintime,
			pfcap->levels[i].rechargefactor);
	}

	dev_dbg(&info->i2c_client->dev, "%s: }\n", __func__);
	info->flash_cap_size = (sizeof(u32) +
		(sizeof(struct nvc_torch_level_info) * i));
	pfcap->numberoflevels = i;

	dev_dbg(&info->i2c_client->dev, "%s: torch cap[%d] = {\n",
		__func__, LM356X_MAX_TORCH_LEVEL);
	val = pcap->torch_curr_step_uA;
	for (i = 0; i < LM356X_MAX_TORCH_LEVEL; i++) {
		ptcap->guidenum[i] = val * i / 1000;
		if (ptcap->guidenum[i] > pcfg->max_peak_current_mA) {
			ptcap->guidenum[i] = 0;
			break;
		}
		dev_dbg(&info->i2c_client->dev, "%s:   %d,\n", __func__, ptcap->guidenum[i]);
	}

	dev_dbg(&info->i2c_client->dev, "%s: }\n", __func__);
	info->torch_cap_size = (sizeof(u32) + (sizeof(s32) * i));
	ptcap->numberoflevels = i;

	if (update && (info->pwr_state == NVC_PWR_COMM ||
		info->pwr_state == NVC_PWR_ON))
		return lm356x_update_settings(info);

	return 0;
}

static int lm356x_strobe(struct lm356x_info *info, int t_on)
{
	u32 gpio = info->pdata->gpio_strobe & 0xffff;
	u32 lact = (info->pdata->gpio_strobe & 0xffff0000) ? 1 : 0;
	return gpio_direction_output(gpio, lact ^ (t_on & 1));
}

#ifdef CONFIG_PM
static int lm356x_suspend(struct i2c_client *client, pm_message_t msg)
{
	struct lm356x_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "Suspending %s\n", info->caps.name);

	return 0;
}

static int lm356x_resume(struct i2c_client *client)
{
	struct lm356x_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "Resuming %s\n", info->caps.name);

	return 0;
}

static void lm356x_shutdown(struct i2c_client *client)
{
	struct lm356x_info *info = i2c_get_clientdata(client);

	dev_info(&client->dev, "Shutting down %s\n", info->caps.name);

	mutex_lock(&info->mutex);
	/* powier off chip to turn off led */
	if (info->pwr_state != NVC_PWR_OFF)
		lm356x_power_off(info);

	info->shutdown_complete = true;
	mutex_unlock(&info->mutex);
}
#endif

static int lm356x_power_on(struct lm356x_info *info)
{
	//struct lm356x_power_rail *power = &info->power;
	int err = 0;
#if 0
	if (power->v_in) {
		err = regulator_enable(power->v_in);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s v_in err\n",
				__func__);
			return err;
		}
	}

	if (power->v_i2c) {
		err = regulator_enable(power->v_i2c);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s v_i2c err\n",
				__func__);
			regulator_disable(power->v_in);
			return err;
		}
	}
#endif
	if (info->pdata && info->pdata->power_on_callback)
		err = info->pdata->power_on_callback(&info->power);

	return err;
}

static int lm356x_power_off(struct lm356x_info *info)
{
	//struct lm356x_power_rail *power = &info->power;
	int err = 0;

	if (info->pdata && info->pdata->power_off_callback)
		err = info->pdata->power_off_callback(&info->power);
	if (IS_ERR_VALUE(err))
		return err;
#if 0
	if (power->v_in) {
		err = regulator_disable(power->v_in);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s vi_in err\n",
				__func__);
			return err;
		}
	}

	if (power->v_i2c) {
		err = regulator_disable(power->v_i2c);
		if (err) {
			dev_err(&info->i2c_client->dev, "%s vi_i2c err\n",
				__func__);
			return err;
		}
	}
#endif
	return 0;
}

static int lm356x_power(struct lm356x_info *info, int pwr)
{
	int err = 0;

	dev_dbg(&info->i2c_client->dev, "%s %d %d\n",
		__func__, pwr, info->pwr_state);
	if (pwr == info->pwr_state) /* power state no change */
		return 0;

	switch (pwr) {
	case NVC_PWR_OFF:
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			(info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err = lm356x_power_off(info);
		break;
	case NVC_PWR_STDBY_OFF:
		info->flash_mode = LM356X_REG_CONTROL_MODE_SHUTDOWN;
		if ((info->pdata->cfg & NVC_CFG_OFF2STDBY) ||
			(info->pdata->cfg & NVC_CFG_BOOT_INIT))
			pwr = NVC_PWR_STDBY;
		else
			err = lm356x_power_on(info);
		if (!err) {
			usleep_range(100, 120);
			err = lm356x_update_settings(info);
		}
		break;
	case NVC_PWR_STDBY:
		info->flash_mode = LM356X_REG_CONTROL_MODE_SHUTDOWN;
		err = lm356x_power_on(info);
		if (!err) {
			usleep_range(100, 120);
			err = lm356x_update_settings(info);
		}
		break;
	case NVC_PWR_COMM:
	case NVC_PWR_ON:
		err = lm356x_power_on(info);
		if (!err) {
			usleep_range(100, 120);
			err = lm356x_update_settings(info);
		}
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err < 0) {
		dev_err(&info->i2c_client->dev, "%s error\n", __func__);
		pwr = NVC_PWR_ERR;
	}
	info->pwr_state = pwr;
	if (err > 0)
		return 0;

	return err;
}

static int lm356x_power_sync(struct lm356x_info *info, int pwr)
{
	int err1 = 0;
	int err2 = 0;

	if ((info->s_mode == NVC_SYNC_OFF) ||
		(info->s_mode == NVC_SYNC_MASTER) ||
		(info->s_mode == NVC_SYNC_STEREO))
		err1 = lm356x_power(info, pwr);
	if ((info->s_mode == NVC_SYNC_SLAVE) ||
		(info->s_mode == NVC_SYNC_STEREO))
		err2 = lm356x_power(info->s_info, pwr);
	return err1 | err2;
}

static int lm356x_get_dev_id(struct lm356x_info *info)
{
	int i, err = 0;
	u8 val;

	if (NVC_PWR_OFF == info->pwr_state ||
		NVC_PWR_OFF_FORCE == info->pwr_state)
		lm356x_power_on(info);

	for (i = 0; i < info->caps.num_regs; i++) {
		err = lm356x_reg_rd(info, lm3560_reg_table_default[i].addr, &val);
		if (err){
			dev_err(&info->i2c_client->dev, "%s: read setting FAIL!!!\n", __func__);
			err = -EINVAL;
			goto power_off_dev;
		}

		if (val != lm3560_reg_table_default[i].val){
			dev_err(&info->i2c_client->dev, "%s: bad default setting!!! reg 0x%x, val 0x%x\n",
				__func__, lm3560_reg_table_default[i].addr, val);
			err = -EIO;
			goto power_off_dev;
		}
	}

power_off_dev:
	if (NVC_PWR_OFF == info->pwr_state)
		lm356x_power_off(info);

	return err;
}

static int lm356x_user_get_param(struct lm356x_info *info, long arg)
{
	struct nvc_param params;
	struct nvc_torch_pin_state pinstate;
	const void *data_ptr = NULL;
	u32 data_size = 0;
	u8 reg;

	if (copy_from_user(&params,
		(const void __user *)arg,
		sizeof(struct nvc_param))) {
			dev_err(&info->i2c_client->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
			return -EINVAL;
	}

	if (info->s_mode == NVC_SYNC_SLAVE)
		info = info->s_info;
	switch (params.param) {
	case NVC_PARAM_FLASH_CAPS:
		dev_dbg(&info->i2c_client->dev, "%s FLASH_CAPS\n", __func__);
		data_ptr = info->flash_cap;
		data_size = info->flash_cap_size;
		break;
	case NVC_PARAM_FLASH_LEVEL:
		reg = info->regs.flash_brightness;
		data_ptr = &info->flash_cap->levels[reg].guidenum;
		data_size = sizeof(info->flash_cap->levels[reg].guidenum);
		break;
	case NVC_PARAM_TORCH_CAPS:
		dev_dbg(&info->i2c_client->dev, "%s TORCH_CAPS\n", __func__);
		data_ptr = info->torch_cap;
		data_size = info->torch_cap_size;
		break;
	case NVC_PARAM_TORCH_LEVEL:
		reg = info->regs.torch_brightness;
		data_ptr = &info->torch_cap->guidenum[reg];
		data_size = sizeof(info->torch_cap->guidenum[reg]);
		break;
	case NVC_PARAM_FLASH_PIN_STATE:
		/* By default use Active Pin State Setting */
		pinstate = info->pdata->pinstate;
		if ((info->flash_mode != LM356X_REG_CONTROL_MODE_FLASH) ||
			(!info->regs.flash_brightness))
			pinstate.values ^= 0xffff; /* Inactive Pin Setting */

		dev_dbg(&info->i2c_client->dev, "%s FLASH_PIN_STATE: %x&%x\n",
			__func__, pinstate.mask, pinstate.values);
		data_ptr = &pinstate;
		data_size = sizeof(pinstate);
		break;
	case NVC_PARAM_STEREO:
		dev_dbg(&info->i2c_client->dev, "%s STEREO: %d\n",
			__func__, info->s_mode);
		data_ptr = &info->s_mode;
		data_size = sizeof(info->s_mode);
		break;
	default:
		dev_err(&info->i2c_client->dev,
			"%s unsupported parameter: %d\n",
			__func__, params.param);
		return -EINVAL;
	}

	if (params.sizeofvalue < data_size) {
		dev_err(&info->i2c_client->dev,
			"%s data size mismatch %d != %d\n",
			__func__, params.sizeofvalue, data_size);
		return -EINVAL;
	}

	if (copy_to_user((void __user *)params.p_value,
		data_ptr,
		data_size)) {
			dev_err(&info->i2c_client->dev,
				"%s copy_to_user err line %d\n",
				__func__, __LINE__);
			return -EFAULT;
	}

	return 0;
}

static int lm356x_set_param(struct lm356x_info *info,
struct nvc_param *params,
	u8 val)
{
	int err;

	switch (params->param) {
	case NVC_PARAM_FLASH_LEVEL:
		dev_dbg(&info->i2c_client->dev, "%s FLASH_LEVEL: %d\n",
			__func__, val);

		info->flash_mode = LM356X_REG_CONTROL_MODE_FLASH;
		err = lm356x_set_leds(info, info->led_mask, info->strobe_type, val, val);
		if (!val)
			info->flash_mode = LM356X_REG_CONTROL_MODE_ASSIST;
		return err;
	case NVC_PARAM_TORCH_LEVEL:
		dev_dbg(&info->i2c_client->dev, "%s TORCH_LEVEL: %d\n",
			__func__, val);
		info->flash_mode = LM356X_REG_CONTROL_MODE_ASSIST;
		err = lm356x_set_leds(info, info->led_mask, info->strobe_type, val, val);
		return err;
	case NVC_PARAM_FLASH_PIN_STATE:
		dev_dbg(&info->i2c_client->dev, "%s FLASH_PIN_STATE: %d\n",
			__func__, val);
		return lm356x_strobe(info, val);
	default:
		dev_err(&info->i2c_client->dev,
			"%s unsupported parameter: %d\n",
			__func__, params->param);
		return -EINVAL;
	}
}

static int lm356x_user_set_param(struct lm356x_info *info, long arg)
{
	struct nvc_param params;
	u8 val;
	int err = 0;

	if (copy_from_user(&params,
		(const void __user *)arg,
		sizeof(struct nvc_param))) {
			dev_err(&info->i2c_client->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
			return -EINVAL;
	}

	if (copy_from_user(&val, (const void __user *)params.p_value,
		sizeof(val))) {
			dev_err(&info->i2c_client->dev, "%s %d copy_from_user err\n",
				__func__, __LINE__);
			return -EINVAL;
	}

	/* parameters independent of sync mode */
	switch (params.param) {
	case NVC_PARAM_STEREO:
		dev_dbg(&info->i2c_client->dev, "%s STEREO: %d\n",
			__func__, (int)val);
		if (val == info->s_mode)
			return 0;

		switch (val) {
		case NVC_SYNC_OFF:
			info->s_mode = val;
			if (info->s_info != NULL) {
				info->s_info->s_mode = val;
				lm356x_power(info->s_info, NVC_PWR_OFF);
			}
			break;
		case NVC_SYNC_MASTER:
			info->s_mode = val;
			if (info->s_info != NULL)
				info->s_info->s_mode = val;
			break;
		case NVC_SYNC_SLAVE:
		case NVC_SYNC_STEREO:
			if (info->s_info != NULL) {
				/* sync power */
				info->s_info->pwr_state = info->pwr_state;
				err = lm356x_power(info->s_info,
					info->pwr_state);
				if (!err) {
					info->s_mode = val;
					info->s_info->s_mode = val;
				} else {
					lm356x_power(info->s_info,
						NVC_PWR_OFF);
					err = -EIO;
				}
			} else {
				err = -EINVAL;
			}
			break;
		default:
			err = -EINVAL;
		}
		if (info->pdata->cfg & NVC_CFG_NOERR)
			return 0;
		return err;
	default:
		/* parameters dependent on sync mode */
		switch (info->s_mode) {
		case NVC_SYNC_OFF:
		case NVC_SYNC_MASTER:
			return lm356x_set_param(info, &params, val);
		case NVC_SYNC_SLAVE:
			return lm356x_set_param(info->s_info, &params, val);
		case NVC_SYNC_STEREO:
			err = lm356x_set_param(info, &params, val);
			if (!(info->pdata->cfg & NVC_CFG_SYNC_I2C_MUX))
				err |= lm356x_set_param(info->s_info,
				&params, val);
			return err;
		default:
			dev_err(&info->i2c_client->dev, "%s %d internal err\n",
				__func__, __LINE__);
			return -EINVAL;
		}
	}
}

static long lm356x_ioctl(struct file *file,
	unsigned int cmd,
	unsigned long arg)
{
	struct lm356x_info *info = file->private_data;
	int pwr;
	int err;

	switch (cmd) {
	case NVC_IOCTL_PARAM_WR:
		return lm356x_user_set_param(info, arg);
	case NVC_IOCTL_PARAM_RD:
		return lm356x_user_get_param(info, arg);
	case NVC_IOCTL_PWR_WR:
		/* This is a Guaranteed Level of Service (GLOS) call */
		pwr = (int)arg * 2;
		dev_dbg(&info->i2c_client->dev, "%s PWR_WR: %d\n",
			__func__, pwr);
		if (!pwr || (pwr > NVC_PWR_ON)) /* Invalid Power State */
			return 0;

		err = lm356x_power_sync(info, pwr);

		if (info->pdata->cfg & NVC_CFG_NOERR)
			return 0;
		return err;
	case NVC_IOCTL_PWR_RD:
		if (info->s_mode == NVC_SYNC_SLAVE)
			pwr = info->s_info->pwr_state / 2;
		else
			pwr = info->pwr_state / 2;
		dev_dbg(&info->i2c_client->dev, "%s PWR_RD: %d\n",
			__func__, pwr);
		if (copy_to_user((void __user *)arg, (const void *)&pwr,
			sizeof(pwr))) {
				dev_err(&info->i2c_client->dev,
					"%s copy_to_user err line %d\n",
					__func__, __LINE__);
				return -EFAULT;
		}

		return 0;
	default:
		dev_err(&info->i2c_client->dev, "%s unsupported ioctl: %x\n",
			__func__, cmd);
		return -EINVAL;
	}
}

static int lm356x_sync_en(int dev1, int dev2)
{
	struct lm356x_info *sync1 = NULL;
	struct lm356x_info *sync2 = NULL;
	struct lm356x_info *pos = NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(pos, &lm356x_info_list, list) {
		if (pos->pdata->num == dev1) {
			sync1 = pos;
			break;
		}
	}
	pos = NULL;
	list_for_each_entry_rcu(pos, &lm356x_info_list, list) {
		if (pos->pdata->num == dev2) {
			sync2 = pos;
			break;
		}
	}
	rcu_read_unlock();
	if (sync1 != NULL)
		sync1->s_info = NULL;
	if (sync2 != NULL)
		sync2->s_info = NULL;
	if (!dev1 && !dev2)
		return 0; /* no err if default instance 0's used */

	if (dev1 == dev2)
		return -EINVAL; /* err if sync instance is itself */

	if ((sync1 != NULL) && (sync2 != NULL)) {
		sync1->s_info = sync2;
		sync2->s_info = sync1;
	}

	return 0;
}

static int lm356x_sync_dis(struct lm356x_info *info)
{
	if (info->s_info != NULL) {
		info->s_info->s_mode = 0;
		info->s_info->s_info = NULL;
		info->s_mode = 0;
		info->s_info = NULL;
		return 0;
	}

	return -EINVAL;
}

static int lm356x_open(struct inode *inode, struct file *file)
{
	struct lm356x_info *info = NULL;
	struct lm356x_info *pos = NULL;
	int err;

	rcu_read_lock();
	list_for_each_entry_rcu(pos, &lm356x_info_list, list) {
		if (pos->miscdev.minor == iminor(inode)) {
			info = pos;
			break;
		}
	}
	rcu_read_unlock();
	if (!info)
		return -ENODEV;

	err = lm356x_sync_en(info->pdata->num, info->pdata->sync);
	if (err == -EINVAL)
		dev_err(&info->i2c_client->dev,
		"%s err: invalid num (%u) and sync (%u) instance\n",
		__func__, info->pdata->num, info->pdata->sync);
	if (atomic_xchg(&info->in_use, 1))
		return -EBUSY;

	if (info->s_info != NULL) {
		if (atomic_xchg(&info->s_info->in_use, 1))
			return -EBUSY;
	}

	file->private_data = info;
	Torch_on_flag = 1;
	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	return 0;
}

static int lm356x_release(struct inode *inode, struct file *file)
{
	struct lm356x_info *info = file->private_data;

	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);

	lm356x_power_sync(info, NVC_PWR_OFF);
	file->private_data = NULL;
	WARN_ON(!atomic_xchg(&info->in_use, 0));
	if (info->s_info != NULL)
		WARN_ON(!atomic_xchg(&info->s_info->in_use, 0));
	lm356x_sync_dis(info);
	return 0;
}

static int lm356x_power_put(struct lm356x_power_rail *pw)
{
#if 0
	if (likely(pw->v_in))
		regulator_put(pw->v_in);

	if (likely(pw->v_i2c))
		regulator_put(pw->v_i2c);

	pw->v_in = NULL;
	pw->v_i2c = NULL;
#endif
	return 0;
}
#if 0
static int lm356x_regulator_get(struct lm356x_info *info,
struct regulator **vreg, char vreg_name[])
{
	struct regulator *reg = NULL;
	int err = 0;

	if (unlikely(IS_ERR_OR_NULL(reg))) {
		dev_err(&info->i2c_client->dev, "%s %s ERR: %d\n",
			__func__, vreg_name, (int)reg);
		err = PTR_ERR(reg);
		reg = NULL;
	} else
		dev_dbg(&info->i2c_client->dev, "%s: %s\n",
		__func__, vreg_name);

	*vreg = reg;
	return err;
}
#endif
static int lm356x_power_get(struct lm356x_info *info)
{
#if 0
	struct lm356x_power_rail *pw = &info->power;

	lm356x_regulator_get(info, &pw->v_in, "vin"); /* 3.7v */
	lm356x_regulator_get(info, &pw->v_i2c, "vi2c"); /* 1.8v */
#endif
	info->pwr_state = NVC_PWR_OFF;

	return 0;
}

static const struct file_operations lm356x_fileops = {
	.owner = THIS_MODULE,
	.open = lm356x_open,
	.unlocked_ioctl = lm356x_ioctl,
	.release = lm356x_release,
};

static void lm356x_del(struct lm356x_info *info)
{
	lm356x_power_sync(info, NVC_PWR_OFF);
	lm356x_power_put(&info->power);

	lm356x_sync_dis(info);
	spin_lock(&lm356x_spinlock);
	list_del_rcu(&info->list);
	spin_unlock(&lm356x_spinlock);
	synchronize_rcu();
}

static int lm356x_remove(struct i2c_client *client)
{
	struct lm356x_info *info = i2c_get_clientdata(client);

	dev_dbg(&info->i2c_client->dev, "%s\n", __func__);
	misc_deregister(&info->miscdev);
	lm356x_del(info);
	return 0;
}

static int lm356x_probe(
struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct lm356x_info *info;
	char dname[16];
	int err;

	dev_dbg(&client->dev, "%s\n", __func__);
	info = devm_kzalloc(&client->dev, sizeof(*info) +
		lm356x_max_flash_cap_size + lm356x_max_torch_cap_size,
		GFP_KERNEL);
	if (info == NULL) {
		dev_err(&client->dev, "%s: kzalloc error\n", __func__);
		return -ENOMEM;
	}

	info->i2c_client = client;
	if (client->dev.platform_data)
		info->pdata = client->dev.platform_data;
	else {
		info->pdata = &lm356x_default_pdata;
		dev_dbg(&client->dev,
			"%s No platform data.  Using defaults.\n",
			__func__);
	}

	info->flash_cap = (void *)info + sizeof(*info);
	info->torch_cap = (void *)info->flash_cap + lm356x_max_flash_cap_size;
	memcpy(&info->caps, &lm356x_caps[info->pdata->type],
		sizeof(info->caps));

	lm356x_config_init(info);

	info->flash_mode = LM356X_REG_CONTROL_MODE_ASSIST; /* torch mode */

	lm356x_configure(info, false);

	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);
	INIT_LIST_HEAD(&info->list);
	spin_lock(&lm356x_spinlock);
	list_add_rcu(&info->list, &lm356x_info_list);
	spin_unlock(&lm356x_spinlock);

	lm356x_power_get(info);

	err = lm356x_get_dev_id(info);
	if (err < 0) {
		dev_err(&client->dev, "%s device not found\n", __func__);
		if (info->pdata->cfg & NVC_CFG_NODEV) {
			lm356x_del(info);
			return -ENODEV;
		}
	} else
		dev_info(&client->dev, "%s device found\n", __func__);

	if (info->pdata->dev_name != 0)
		strcpy(dname, info->pdata->dev_name);
	else
		strcpy(dname, "lm356x");
	if (info->pdata->num)
		snprintf(dname, sizeof(dname), "%s.%u",
		dname, info->pdata->num);

	info->miscdev.name = dname;
	info->miscdev.fops = &lm356x_fileops;
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	if (misc_register(&info->miscdev)) {
		dev_err(&client->dev, "%s unable to register misc device %s\n",
			__func__, dname);
		lm356x_del(info);
		return -ENODEV;
	}

	info->shutdown_complete = false;
	lm356x_debugfs_init(info);
	return 0;
}

static int lm356x_status_show(struct seq_file *s, void *data)
{
	struct lm356x_info *k_info = s->private;
	struct lm356x_config *pcfg = &k_info->config;

	pr_info("%s\n", __func__);

	seq_printf(s, "lm356x status:\n"
		"    Flash type: %s, bus %d, addr: 0x%02x\n\n"
		"    Led Mask         = %01x\n"
		"    Torch Current    = 0x%02x\n"
		"    Flash Current    = 0x%02x\n"
		"    Flash Mode       = 0x%02x\n"
		"    Flash TimeOut    = 0x%02x\n"
		"    Flash Strobe     = 0x%02x\n"
		"    Max_Peak_Current = 0x%04dmA\n"
		"    Use_TxMask       = 0x%02x\n"
		"    VIN_low_run      = 0x%04dmV\n"
		"    VIN_low          = 0x%04dmV\n"
		"    LedOff_On_VIN_low = %s\n"
		"    PinState Mask    = 0x%04x\n"
		"    PinState Values  = 0x%04x\n"
		,
		(char *)lm356x_id[k_info->pdata->type + 1].name,
		k_info->i2c_client->adapter->nr,
		k_info->i2c_client->addr,
		k_info->led_mask,
		k_info->regs.torch_brightness,
		k_info->regs.flash_brightness,
		k_info->flash_mode, k_info->regs.ftime,
		pcfg->strobe_type,
		pcfg->max_peak_current_mA,
		pcfg->tx_mask,
		pcfg->vin_low_v_run_mV,
		pcfg->vin_low_v_mV,
		pcfg->led_off_when_vin_low ? "TRUE" : "FALSE",
		k_info->pdata->pinstate.mask,
		k_info->pdata->pinstate.values
		);

	return 0;
}

static ssize_t lm356x_attr_set(struct file *s,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct lm356x_info *k_info =
		((struct seq_file *)s->private_data)->private;
	char buf[24];
	int buf_size;
	u32 val = 0;

	pr_info("%s\n", __func__);

	if (!user_buf || count <= 1)
		return -EFAULT;

	memset(buf, 0, sizeof(buf));
	buf_size = min(count, sizeof(buf) - 1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;

	if (sscanf(buf + 1, "0x%x", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "0X%x", &val) == 1)
		goto set_attr;
	if (sscanf(buf + 1, "%d", &val) == 1)
		goto set_attr;

	pr_info("SYNTAX ERROR: %s\n", buf);
	return -EFAULT;

set_attr:
	pr_info("new data = %x\n", val);
	switch (buf[0]) {
	case 'p':
		if (val & 0xffff)
			lm356x_power(k_info, NVC_PWR_ON);
		else
			lm356x_power(k_info, NVC_PWR_OFF);
		break;
	case 'c': /* change torch/flash current settings */
		lm356x_set_leds(k_info, k_info->led_mask, k_info->strobe_type,
			(val >> 4) & 0x07, val & 0x0f);
		break;
	case 'l': /* enable/disable led 1/2 */
		k_info->pdata->led_mask = val;
		lm356x_configure(k_info, false);
		break;
	case 'm': /* change pinstate setting */
		k_info->pdata->pinstate.mask = (val >> 16) & 0xffff;
		k_info->pdata->pinstate.values = val & 0xffff;
		break;
	case 'f': /* modify flash timeout reg */
		k_info->regs.ftime &= ~(0x1f);
		k_info->regs.ftime |= (val & 0x1f);
		lm356x_set_leds(k_info, k_info->led_mask, k_info->strobe_type,
			k_info->regs.torch_brightness,
			k_info->regs.flash_brightness);
		break;
	case 't': /* change txmask/torch settings */
#if 0 // Need implement
		k_info->config.tx_mask = (val >> 4) & 1;
		val = (val >> 8) & 0xffff;
		if (val)
			k_info->config.I_limit_mA = val;

		lm356x_set_txmask(k_info);
#endif
		break;
	case 'v':
		if (val & 0x30) {
			k_info->config.vin_low_v_run_mV &= ~(0x30);
			k_info->config.vin_low_v_run_mV |= (val & 0x30);
		}
		if (val & 0x06) {
			k_info->config.vin_low_v_mV &= ~(0x06);
			k_info->config.vin_low_v_mV |= (val & 0x06);
		}
		lm356x_configure(k_info, true);
		break;
	case 'k':
		if (val <= 1600) {
			k_info->config.max_peak_current_mA = 1600;
			k_info->regs.ftime &= ~(0x30);
			k_info->regs.ftime |= 0x00;
		} else if (val <= 2300) {
			k_info->config.max_peak_current_mA = 2300;
			k_info->regs.ftime &= ~(0x30);
			k_info->regs.ftime |= 0x20;
		} else if (val <= 3000) {
			k_info->config.max_peak_current_mA = 3000;
			k_info->regs.ftime &= ~(0x30);
			k_info->regs.ftime |= 0x40;
		} else {
			k_info->config.max_peak_current_mA = 3600;
			k_info->regs.ftime &= ~(0x30);
			k_info->regs.ftime |= 0x60;
		}
		lm356x_configure(k_info, true);
		break;
	case 'x':
#if 0 // Need implement
		if (val & 0xf)
			k_info->flash_mode = (val & 0xf) - 1;
		if (val & 0xf0)
			k_info->config.strobe_type = ((val & 0xf0) >> 4) - 1;
		if (val & 0xf000)
			k_info->config.led_off_when_vin_low =
			((val & 0xf000) == 0x2000);
		if (val & 0xf0000) {
			val = ((val & 0xf0000) >> 16) - 1;
			if (val >= LM356X_NUM) {
				pr_err("Invalid dev type %x\n", val);
				return -ENODEV;
			}
			k_info->pdata->type = val;
			memcpy(&k_info->caps,
				&lm356x_caps[k_info->pdata->type],
				sizeof(k_info->caps));
		}
		lm356x_configure(k_info, true);
#endif
		break;
	case 'g':
		k_info->pdata->gpio_strobe = val;
		lm356x_strobe(k_info, 1);
		break;
	}

	return count;
}

static int lm356x_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, lm356x_status_show, inode->i_private);
}

static const struct file_operations lm356x_debugfs_fops = {
	.open = lm356x_debugfs_open,
	.read = seq_read,
	.write = lm356x_attr_set,
	.llseek = seq_lseek,
	.release = single_release,
};

static int lm356x_debugfs_init(struct lm356x_info *info)
{
	struct dentry *d;

	info->d_lm356x = debugfs_create_dir(
		info->miscdev.this_device->kobj.name, NULL);
	if (info->d_lm356x == NULL) {
		pr_info("%s: debugfs create dir failed\n", __func__);
		return -ENOMEM;
	}

	d = debugfs_create_file("d", S_IRUGO|S_IWUSR, info->d_lm356x,
		(void *)info, &lm356x_debugfs_fops);
	if (!d) {
		pr_info("%s: debugfs create file failed\n", __func__);
		debugfs_remove_recursive(info->d_lm356x);
		info->d_lm356x = NULL;
	}

	return -EFAULT;
}

static struct i2c_driver lm356x_driver = {
	.driver = {
		.name = "lm356x",
		.owner = THIS_MODULE,
	},
	.id_table = lm356x_id,
	.probe = lm356x_probe,
	.remove = lm356x_remove,
#ifdef CONFIG_PM
	.shutdown = lm356x_shutdown,
	.suspend  = lm356x_suspend,
	.resume   = lm356x_resume,
#endif
};

module_i2c_driver(lm356x_driver);

MODULE_DESCRIPTION("LM356X flash/torch driver");
MODULE_LICENSE("GPL");
