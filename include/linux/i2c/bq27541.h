/*
 * Copyright (C) 2011 QUANTA, Inc.
 * 
 */

#ifndef _LINUX_BQ27541_I2C_H
#define _LINUX_BQ27541_I2C_H
#include <linux/smb349-charger.h>

/*-----------------------------------------------------------------------------
 * Notifier Definition
 *---------------------------------------------------------------------------*/
#define BQ24191_NOTIFIER_EVENT		3

/*-----------------------------------------------------------------------------
 * Commands Definition
 *---------------------------------------------------------------------------*/
//battery gauge
#define REG_CONTROL_LSB			0x00
#define REG_CONTROL_MSB			0x01
#define REG_ATRATE_LSB			0x02
#define REG_ATRATE_MSB			0x03
#define REG_ATTIME2EMPTY_LSB		0x04
#define REG_ATTIME2EMPTY_MSB		0x05
#define REG_TEMPERATURE_LSB		0x06
#define REG_TEMPERATURE_MSB		0x07
#define REG_VOLTAGE_LSB			0x08
#define REG_VOLTAGE_MSB			0x09
#define REG_FLAGS_LSB			0x0A
#define REG_FLAGS_MSB			0x0B
	#define FLAGS_DSG	(1<<0)
	#define FLAGS_SOCF	(1<<1)
	#define FLAGS_SOC1	(1<<2)
	#define FLAGS_CHG	(1<<8)
	#define FLAGS_FC	(1<<9)
	#define FLAGS_XCHG	(1<,10)
	#define FLAGS_CHG_INH	(1<<11)
	#define FLAGS_OTD	(1<<14)
	#define FLAGS_OTC	(1<<15)
#define REG_NAC_LSB			0x0C
#define REG_NAC_MSB			0x0D
#define REG_FAC_LSB			0x0E
#define REG_FAC_MSB			0x0F
#define REG_RM_LSB			0x10
#define REG_RM_MSB			0x11
#define REG_FCC_LSB			0x12
#define REG_FCC_MSB			0x13
#define REG_AVGI_LSB			0x14
#define REG_AVGI_MSB			0x15
#define REG_TIME2EMPTY_LSB		0x16
#define REG_TIME2EMPTY_MSB		0x17
#define REG_TIME2FULL_LSB		0x18
#define REG_TIME2FULL_MSB		0x19
#define REG_CYCLECOUNT_LSB		0x2A
#define REG_CYCLECOUNT_MSB		0x2B
#define REG_SOC_LSB			0x2C
#define REG_SOC_MSB			0x2D

struct bq27541_platform_data {
	//int (*gpio_init)(void);
	int ac_present_gpio;
	int usb_present_gpio;
	int use_ac;
	int use_usb;
	void (*charge_led)(int);
};

//int bq27541_register_notifier(struct notifier_block *nb, unsigned int events);
//int bq27541_unregister_notifier(struct notifier_block *nb, unsigned int events);
void bq27541_battery_status(int status, int chrg_type);
int bq27541_check_battery(void);

#endif
