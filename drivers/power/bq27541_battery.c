/*
 * linux/drivers/power/bq27541_battery.c
 *
 * bq27541 battery driver for Linux
 *
 * Copyright (C) 2011 Quanta Computer, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <mach/gpio.h>
#include <linux/delay.h>

#include <linux/usb/otg.h>

#include <linux/i2c/bq27541.h>
#include "../../kernel/power/power.h"
//#include <linux/i2c/bq24191.h>

//#define ENABLE_AC_SUPPLY

/* Const Defns */
#define TEGRA_GPIO_PQ0		128
#define BATTERY_POLL_PERIOD	3000
static unsigned int poll_interval = BATTERY_POLL_PERIOD;

#define I2C1_BATTERY_GAUGE_ADDRESS	0x55
#define DRIVER_BQ27541 "bq27541"

static struct blocking_notifier_head bq27541_notifier_list;

static int g_full_available_capacity=4400;
static int g_batt_present = 1;

/* General structure to hold the driver data */
struct bq27541_device_info {
	struct delayed_work	battery_poll_work;
	struct i2c_client	*client;
	struct bq27541_platform_data *pdata;
	struct otg_transceiver			*otg;
	struct notifier_block			nb;
	struct power_supply		battery;
	struct power_supply		ac;
	struct power_supply		usb;
	//int usb_plug;
	//int irq;

	int ac_online;
	int usb_online;
	int status;
	int present;
	int voltage;
	int capacity;
	int temp;

	int lasttime_voltage;
	int lasttime_capacity;
	int use_usb:1;
	int use_ac:1;
};
struct bq27541_device_info *bq27541_data;

static u8 data[2];
static int ret = 0;
static int batt_capacity = 0;

/*
static enum supply_type {
	SUPPLY_TYPE_BATTERY = 0,
	SUPPLY_TYPE_USB,
	SUPPLY_TYPE_AC,
};

struct workqueue_struct    *bat_work_queue;
*/
static int bq27541_bat_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static int bq27541_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static int bq27541_usb_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static enum power_supply_property bq27541_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
//	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
//	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
//	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
//	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
//	POWER_SUPPLY_PROP_SERIAL_NUMBER
};

static char *power_supplied_to[] = {
	"battery",
};

static enum power_supply_property bq27541_ac_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property bq27541_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};
/*
static struct power_supply bq27541_supply[] = {
	[SUPPLY_TYPE_BATTERY] = {
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= bq27541_battery_properties,
		.num_properties	= ARRAY_SIZE(bq27541_battery_properties),
		.get_property	= bq27541_bat_get_property,
	},
	[SUPPLY_TYPE_USB] = {
		.name		= "usb",
		.type		= POWER_SUPPLY_TYPE_USB,
		.properties	= bq27541_usb_properties,
		.num_properties	= ARRAY_SIZE(bq27541_usb_properties),
		.get_property	= bq27541_usb_get_property,
	},
#ifdef ENABLE_AC_SUPPLY
	[SUPPLY_TYPE_AC] = {
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = power_supplied_to,
		.num_supplicants = ARRAY_SIZE(power_supplied_to),
		.properties = bq27541_ac_properties,
		.num_properties = ARRAY_SIZE(bq27541_ac_properties),
		.get_property = bq27541_ac_get_property,
	},
#endif
};*/
/*----------------------------------------------------------------------------*/
int bq27541_register_notifier(struct notifier_block *nb,
				unsigned int events)
{
	return blocking_notifier_chain_register(&bq27541_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(bq27541_register_notifier);
/*----------------------------------------------------------------------------*/
int bq27541_unregister_notifier(struct notifier_block *nb,
				unsigned int events)
{
	return blocking_notifier_chain_unregister(&bq27541_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(bq27541_unregister_notifier);
/*----------------------------------------------------------------------------*/
int get_battery_mAh()
{
    return g_full_available_capacity;
}
EXPORT_SYMBOL_GPL(get_battery_mAh);
/*----------------------------------------------------------------------------*/
int is_battery_present()
{
    return g_batt_present;
}
EXPORT_SYMBOL_GPL(is_battery_present);
/*-----------------------------------------------------------------------------
 * I2C level read/write related
 *---------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int bq27541_read_byte_data(struct i2c_client *client, u8 reg, u8 *data)
{
	int result;
	result = i2c_smbus_read_byte_data(client, reg);
	*data = result;
	return result;
}
/*----------------------------------------------------------------------------*/
static int bq27541_write_byte_data(struct i2c_client *client, u8 reg, u8 data)
{
	return i2c_smbus_write_byte_data(client, reg, data);
}
/*----------------------------------------------------------------------------*/
#if 0
static int bq27541_usb_notifier_call(struct notifier_block *nb, unsigned long event, void *data)
{
	//printk("bq27541_usb_notifier_call: event = %d \n", event);
	switch (event) {
		case USB_EVENT_LIMIT_100:
		case USB_EVENT_LIMIT_500:
			bq27541_device->usb_plug = 1;
			bq27541_device->pdata->charge_led(1);
			break;
		case USB_EVENT_NONE:
			bq27541_device->usb_plug = 0;
			bq27541_device->pdata->charge_led(0);
			break;
		default:
			return NOTIFY_OK;
	}
/*
#ifdef ENABLE_AC_SUPPLY
	power_supply_changed(&bq27541_supply[SUPPLY_TYPE_AC]);
#endif
	power_supply_changed(&bq27541_supply[SUPPLY_TYPE_BATTERY]);
*/
	return NOTIFY_OK;
}
#endif
/*-----------------------------------------------------------------------------
 * BQ28550 Function related
 *---------------------------------------------------------------------------*/
static int get_battery_info_temperature(struct i2c_client *client, int *bq27541_temperature)
{
	ret = bq27541_read_byte_data(client, REG_TEMPERATURE_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_TEMPERATURE_MSB, &data[1]);
	if (ret < 0)
		return ret;
	*bq27541_temperature = data[1] << 8 | data[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_voltage(struct i2c_client *client, int *bq27541_voltage)
{
	ret = bq27541_read_byte_data(client, REG_VOLTAGE_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_VOLTAGE_MSB, &data[1]);
	if (ret < 0) 
		return ret;
	*bq27541_voltage = data[1] << 8 | data[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_avg_current(struct i2c_client *client, int *bq27541_avg_current)
{
	s16 avgi;
	ret = bq27541_read_byte_data(client, REG_AVGI_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_AVGI_MSB, &data[1]);
	if (ret < 0) 
		return ret;
	avgi = (s16) (data[1] << 8 | data[0]);
	*bq27541_avg_current = (int) avgi;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_current(struct i2c_client *client, int *bq27541_current)
{
	s16 atrate;
	ret = bq27541_read_byte_data(client, REG_ATRATE_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_ATRATE_MSB, &data[1]);
	if (ret < 0) 
		return ret;
	atrate = (s16) (data[1] << 8 | data[0]);
	*bq27541_current = (int) atrate;
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_flags(struct i2c_client *client, int *bq27541_flags)
{
	ret = bq27541_read_byte_data(client, REG_FLAGS_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_FLAGS_MSB, &data[1]);
	if (ret < 0) 
		return ret;
	*bq27541_flags = data[1] << 8 | data[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_fac(struct i2c_client *client, int *bq27541_fac)
{
	ret = bq27541_read_byte_data(client, REG_FAC_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_FAC_MSB, &data[1]);
	if (ret < 0) 
		return ret;
	*bq27541_fac = data[1] << 8 | data[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_capacity(struct i2c_client *client, int *bq27541_capacity)
{
	ret = bq27541_read_byte_data(client, REG_SOC_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	*bq27541_capacity = data[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_rm(struct i2c_client *client, int *bq27541_rm)
{
	ret = bq27541_read_byte_data(client, REG_RM_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_RM_MSB, &data[1]);
	if (ret < 0) 
		return ret;
	*bq27541_rm = data[1] << 8 | data[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_fcc(struct i2c_client *client, int *bq27541_fcc)
{
	ret = bq27541_read_byte_data(client, REG_FCC_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_FCC_MSB, &data[1]);
	if (ret < 0) 
		return ret;
	*bq27541_fcc = data[1] << 8 | data[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_time2empty(struct i2c_client *client, int *bq27541_time2empty)
{
	ret = bq27541_read_byte_data(client, REG_TIME2EMPTY_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_TIME2EMPTY_MSB, &data[1]);
	if (ret < 0) 
		return ret;
	*bq27541_time2empty = data[1] << 8 | data[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static int get_battery_info_time2full(struct i2c_client *client, int *bq27541_time2full)
{
	ret = bq27541_read_byte_data(client, REG_TIME2FULL_LSB, &data[0]);
	if (ret < 0) 
		return ret;
	ret = bq27541_read_byte_data(client, REG_TIME2FULL_MSB, &data[1]);
	if (ret < 0) 
		return ret;
	*bq27541_time2full = data[1] << 8 | data[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
#if 0
static irqreturn_t ac_present_irq(int irq, void *data)
{
	int events;

	//printk(KERN_INFO "%s\n", __func__);

#ifdef ENABLE_AC_SUPPLY
	//power_supply_changed(&bq27541_supply[SUPPLY_TYPE_AC]);
#endif
	//power_supply_changed(&bq27541_supply[SUPPLY_TYPE_BATTERY]);

#if 0
	if(gpio_get_value_cansleep(bq27541_device->pdata->ac_present_gpio)) {
		events = BQ24191_START_CHARGING;
	} else {
		events = BQ24191_STOP_CHARGING;
	}
	blocking_notifier_call_chain(&bq27541_notifier_list, events, NULL);
#endif
	return IRQ_HANDLED;
}
#endif


static int BattChargerStatus(struct bq27541_device_info *di)
{
//	POWER_SUPPLY_STATUS_UNKNOWN
//	POWER_SUPPLY_STATUS_CHARGING
//	POWER_SUPPLY_STATUS_DISCHARGING
#if 0
	printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %s(+)\n", __func__);
	if(gpio_get_value_cansleep(bq27541_device->pdata->ac_present_gpio) && !bq27541_device->usb_plug)
	{
		bq27541_device->pdata->charge_led(0);
		poll_interval = BATTERY_POLL_PERIOD;
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
	else
	{
		poll_interval = CHARGE_FLAG_CHANGE_INTERVAL;
		if(batt_capacity == 100) {
			bq27541_device->pdata->charge_led(0);
			return POWER_SUPPLY_STATUS_FULL;
		} else {
			bq27541_device->pdata->charge_led(1);
			return POWER_SUPPLY_STATUS_CHARGING;
		}
	}
	printk("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ %s(-)\n", __func__);
#endif

	int discharging, flags = 0;
	int status;
	int shunt_voltage;

	if (!g_batt_present || get_battery_info_flags(di->client, &flags))
		status = POWER_SUPPLY_STATUS_UNKNOWN;

	discharging = flags & FLAGS_DSG;

	if(batt_capacity == 100) {
		//printk("POWER_SUPPLY_STATUS_FULL~~~~~~~~~~~~~~~~ \n");
		status = POWER_SUPPLY_STATUS_FULL;
	} else if (discharging) {
		//printk("POWER_SUPPLY_STATUS_DISCHARGING~~~~~~~~~~~~~~~~ \n");
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		//printk("POWER_SUPPLY_STATUS_CHARGING~~~~~~~~~~~~~~~~ \n");
		status = POWER_SUPPLY_STATUS_CHARGING;
	}

	return status;

}

static void battery_poll_work_func(struct work_struct *work)
{
	struct bq27541_device_info *di;
	di = container_of(work, struct bq27541_device_info, battery_poll_work.work);

	int rc = 0;
	int reg_data;

	//printk(KERN_INFO "%s\n", __func__);

	get_battery_info_fac(di->client, &g_full_available_capacity);

	rc = get_battery_info_capacity(di->client, &reg_data);
	if (rc < 0) {
		dev_err(&di->client->dev,
			"%s: no battery found(%d)\n", __func__, rc);
		di->present = -EINVAL;
	} else 
		di->present = 1;

	di->capacity =
		((reg_data > 100) ? 100 : max((s32)(reg_data - 3) * 100 / 97, 0));

	di->status = BattChargerStatus(di);

	rc = get_battery_info_voltage(di->client, &reg_data);
	if (rc < 0) {
		dev_err(&di->client->dev,
		"%s: no battery voltage(%d)\n", __func__, rc);
		di->voltage = -EINVAL;
	} else
		di->voltage = reg_data * 3 * 1000; //in microV unit

	rc = get_battery_info_temperature(di->client, &reg_data);
	if (rc < 0) {
		dev_err(&di->client->dev,
		"%s: no battery temperature(%d)\n", __func__, rc);
		di->temp = -EINVAL;
	} else 
		di->temp = reg_data - 2731;

	if (di->voltage != di->lasttime_voltage ||
		di->capacity != di->lasttime_capacity ) {

		di->lasttime_voltage = di->voltage;
		di->lasttime_capacity = di->capacity;

		power_supply_changed(&di->battery);
	}
	schedule_delayed_work(&di->battery_poll_work, msecs_to_jiffies(poll_interval));
}

void bq27541_battery_status(int status,
				int chrg_type)
{
	if (!bq27541_data)
		return;

	bq27541_data->ac_online = 0;
	bq27541_data->usb_online = 0;

	if (status == progress) {
		bq27541_data->status = POWER_SUPPLY_STATUS_CHARGING;
		if (chrg_type == AC)
			bq27541_data->ac_online = 1;
		else if (chrg_type == USB)
			bq27541_data->usb_online = 1;
	} else
		bq27541_data->status = POWER_SUPPLY_STATUS_DISCHARGING;

	power_supply_changed(&bq27541_data->battery);
	if (bq27541_data->use_usb)
		power_supply_changed(&bq27541_data->usb);
	if (bq27541_data->use_ac)
		power_supply_changed(&bq27541_data->ac);

}
EXPORT_SYMBOL_GPL(bq27541_battery_status);

static int bq27541_bat_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	struct bq27541_device_info *di = container_of(psy, struct bq27541_device_info, battery);

	int rc;
	int reg_data;

	switch (psp) {
	case POWER_SUPPLY_PROP_HEALTH:
		//printk("POWER_SUPPLY_PROP_HEALTH\n");
	case POWER_SUPPLY_PROP_PRESENT:
		//printk("POWER_SUPPLY_PROP_PRESENT\n");
		val->intval = di->present;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		//printk("POWER_SUPPLY_PROP_STATUS\n");
		val->intval = di->status;
		//printk("Charger status=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = di->capacity;
		batt_capacity = val->intval;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		//printk("POWER_SUPPLY_PROP_TECHNOLOGY\n");
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	default:
		dev_err(&di->client->dev,
			"%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int bq27541_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq27541_device_info *di = container_of(psy, struct bq27541_device_info, ac);

	//printk("bq27541_ac_get_property\n");
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		//printk("POWER_SUPPLY_PROP_ONLINE\n");
#if 0
		if (bq27541_device->pdata)
			if (gpio_is_valid(bq27541_device->pdata->ac_present_gpio))
				val->intval = 1 ^ gpio_get_value_cansleep(
						bq27541_device->pdata->ac_present_gpio);
			else
				return -EINVAL;
		else

			val->intval = gpio_get_value_cansleep(
						irq_to_gpio(bq27541_device->irq));
#endif
		val->intval = di->ac_online;
		break;
	default:
		dev_err(&di->client->dev,
			"%s: INVALID property\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int bq27541_usb_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct bq27541_device_info *di = container_of(psy, struct bq27541_device_info, usb);

	if (psp == POWER_SUPPLY_PROP_ONLINE)
		val->intval = di->usb_online;
	else
		return -EINVAL;

	return 0;
}

int bq27541_check_battery()
{
	int ret;

	if (!bq27541_data)
		return -ENODEV;

    ret = i2c_smbus_read_word_data(bq27541_data->client, REG_VOLTAGE_LSB);
	if (ret < 0) {
		dev_err(&bq27541_data->client->dev,
			"%s: no battery present(%d)\n", __func__, ret);

		g_batt_present = 0;

		return -EINVAL;

	}
	return 0;
}
EXPORT_SYMBOL_GPL(bq27541_check_battery);

static int __devinit bq27541_battery_probe(struct i2c_client *client, 
					const struct i2c_device_id *id)
{
	struct bq27541_device_info *di;

	int ret;
	int i;
	int flags;

	printk(KERN_INFO "%s\n", __func__);

	di = kzalloc(sizeof(*di), GFP_KERNEL);
	if (!di) {
	   dev_err(&client->dev, "fail to allocate memory!!!\n");
	   ret = -ENOMEM;
	   goto exit;
	}
	
	di->client = client;
	di->pdata = client->dev.platform_data;
	di->ac_online = 0;
	di->usb_online = 0;
	bq27541_data = di;
	flags = di->client->flags;
	di->client->flags &= ~I2C_M_IGNORE_NAK;
	
	i2c_set_clientdata(client, di);
	
	if (bq27541_check_battery() < 0) {
		ret = -ENODEV;
		goto exit;
	}

	get_battery_info_fac(di->client, &g_full_available_capacity);
	
	di->client->flags = flags;
	/*di->irq = client->irq;
	
	//add AC plug interrupt
	if (di->irq) {
		ret = request_threaded_irq(di->irq, NULL,
			ac_present_irq,
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			"ac_present", di);
		if (ret < 0) {
			dev_err(&di->client->dev,
				"%s: request_irq failed(%d)\n", __func__, ret);
			goto exit;
		}
	}

	ret = register_callback(bq27541_battery_status, di);
	if (ret < 0)
		goto exit;*/

	di->battery.name		= "battery";
	di->battery.type		= POWER_SUPPLY_TYPE_BATTERY;
	di->battery.get_property	= bq27541_bat_get_property;
	di->battery.properties		= bq27541_battery_properties;
	di->battery.num_properties	= ARRAY_SIZE(bq27541_battery_properties);

	ret = power_supply_register(&client->dev, &di->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		goto exit;
	}

    if (di->pdata->use_ac) {
	    di->ac.name		= "bq27541-ac";
	    di->ac.type		= POWER_SUPPLY_TYPE_MAINS;
	    di->ac.get_property	= bq27541_ac_get_property;
	    di->ac.properties	= bq27541_ac_properties;
	    di->ac.num_properties	= ARRAY_SIZE(bq27541_ac_properties);

	    ret = power_supply_register(&client->dev, &di->ac);
	    if (ret) {
		    dev_err(&client->dev, "failed: power supply register\n");
		    goto error1;
	    }
	}

    if (di->pdata->use_usb) {
	    di->usb.name		= "bq27541-usb";
	    di->usb.type		= POWER_SUPPLY_TYPE_USB;
	    di->usb.get_property	= bq27541_usb_get_property;
	    di->usb.properties	= bq27541_usb_properties;
	    di->usb.num_properties	= ARRAY_SIZE(bq27541_usb_properties);

	    ret = power_supply_register(&client->dev, &di->usb);
	    if (ret) {
		    dev_err(&client->dev, "failed: power supply register\n");
		    goto error;
	    }
	}

	INIT_DELAYED_WORK_DEFERRABLE(&di->battery_poll_work, battery_poll_work_func);
	schedule_delayed_work(&di->battery_poll_work, msecs_to_jiffies(poll_interval));

	dev_info(&di->client->dev, "driver registered\n");
	
        return 0;	

error:
	power_supply_unregister(&di->ac);
error1:
	power_supply_unregister(&di->battery);
exit:
        kfree(di);
        return ret;
}

static int __devexit bq27541_battery_remove(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	power_supply_unregister(&di->battery);
	power_supply_unregister(&di->usb);
	power_supply_unregister(&di->ac);
	cancel_delayed_work_sync(&di->battery_poll_work);
	kfree(di);
        
        return 0;
}

#ifdef CONFIG_PM
static int bq27541_battery_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	printk("%s\n",__func__);

	cancel_delayed_work_sync(&di->battery_poll_work);

	return 0;
}

static int bq27541_battery_resume(struct i2c_client *client)
{
	struct bq27541_device_info *di = i2c_get_clientdata(client);

	printk("%s\n",__func__);

	schedule_delayed_work(&di->battery_poll_work, 0);

	return 0;
}

#else

#define bq27541_battery_suspend NULL
#define bq27541_battery_resume NULL

#endif /* CONFIG_PM */


static const struct i2c_device_id bq27541_id[] = {
	{ DRIVER_BQ27541, 0 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
	.driver		= {
		.name	= "bq27541_battery",
	},
	.probe		= bq27541_battery_probe,
	.remove		= bq27541_battery_remove,
	.id_table	= bq27541_id,
    .suspend	= bq27541_battery_suspend,
    .resume		= bq27541_battery_resume,
};

static int __init bq27541_battery_init(void)
{
    int ret = 0;

    ret = i2c_add_driver(&bq27541_battery_driver);
    if (ret) {
        printk(KERN_ERR "bq27541: Could not add driver\n");
        return ret;
    }
    return 0;
}

module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Quanta Inc");
