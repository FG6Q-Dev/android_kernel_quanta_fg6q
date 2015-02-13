/*
 * Copyright (c) 2012, NVIDIA CORPORATION.  All rights reserved.

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

#ifndef __LM356X_H__
#define __LM356X_H__

#include <media/nvc_torch.h>

#define LM356X_REG_ENABLE           0x10
#define LM356X_REG_PRIVACY          0x11
#define LM356X_REG_INDICATOR        0x12
#define LM356X_REG_INDICATOR_BLINK  0x13
#define LM356X_REG_PRIVACY_PWM      0x14
#define LM356X_REG_GPIO             0x20
#define LM356X_REG_VLED_MONITOR     0x30
#define LM356X_REG_ADC_DELAY        0x31
#define LM356X_REG_VIN_MONITOR      0x80
#define LM356X_REG_LAST_FLASH       0x81
#define LM356X_REG_TORCH_BRIGHTNESS 0xA0
#define LM356X_REG_FLASH_BRIGHTNESS 0xB0
#define LM356X_REG_FLASH_DURATION   0xC0
#define LM356X_REG_FLAGS            0xD0
#define LM356X_REG_CONFIG_1         0xE0
#define LM356X_REG_CONFIG_2         0xF0

#define LM356X_REG_CONTROL_MODE_SHUTDOWN	0x00
#define LM356X_REG_CONTROL_MODE_INDICATOR	0x01
#define LM356X_REG_CONTROL_MODE_ASSIST		0x02
#define LM356X_REG_CONTROL_MODE_FLASH		0x03

#define LM356X_PRIVACY_MODE_MASK    (0x01<<3)
#define LM356X_PRIVACY_MODE_ON      (0x00<<3)
#define LM356X_PRIVACY_MODE_OFF     (0x01<<3)

#define LM356X_PRIVACY_BLINK_MASK   (0x03<<6)
#define LM356X_PRIVACY_NO_BLINK     (0x00<<6)
#define LM356X_PRIVACY_BLINK_128MS  (0x01<<6)
#define LM356X_PRIVACY_BLINK_256MS  (0x02<<6)
#define LM356X_PRIVACY_BLINK_512MS  (0x03<<6)

#define LM356X_MAX_FLASH_LEVEL	12 //687.5mA for single LED
#define LM356X_MAX_TORCH_LEVEL	9//250mA for single LED

#define SUSTAINTIME_DEF		512 // kokob3: extend flash duration from 256ms to 512ms
#define DEFAULT_FLASHTIME	((SUSTAINTIME_DEF/32)-1)
#define RECHARGEFACTOR_DEF	160

enum {
	LM3560,
	LM3561,
	LM356X_NUM,
};

struct lm356x_config {
	bool tx_mask; /* enable TX: 0=disable TX1/TX2, 1= only enable TX1,
                                2=only enable TX2, 3= enable TX1/TX2 */
	u16 I_limit_mA; /* LM3560: 1600, 2300, 3000, 3600 mA for the coil*/
	u16 vin_low_v_run_mV; /* off, 2900, 3000, 3100, 3200 mV
                             battery limit for dynamic flash reduction */
	u16 vin_low_v_mV; /* off, 2900, 3000, 3100, 3200mV
                         battery limit for flash denial */
	u8 strobe_type; /* 1=edge, 2=level */
       u8 privacy;
	/* balance the current sinks for unmatched LED forward valtages */
	bool load_balance_on;
	bool led_off_when_vin_low; /* if 0 txmask current is used */
	bool boost_mode; /* all LED current are increased by 11% */
	/* LED configuration, two identical leds must be connected. */
	u16 max_total_current_mA; /* Both leds' maximum peak current in mA */
	u16 max_peak_current_mA; /* This led's maximum peak current in mA */
	u16 max_peak_duration_ms; /* the maximum duration max_peak_current_mA
				     can be applied */
	u16 max_sustained_current_mA; /* This leds maximum sustained current
					 in mA */
	u16 min_current_mA; /* This leds minimum current in mA, desired
			       values smaller than this will be realised
			       using PWM. */
};

struct lm356x_power_rail {
	struct regulator *v_in;
	struct regulator *v_i2c;
};

struct lm356x_platform_data {
	struct lm356x_config config;
	u32 type; /* flash device type, refer to lm356x_type */
	u32 led_mask; /* which led(s) enabled, 1/2/3 - left/right/both */
	unsigned cfg; /* use the NVC_CFG_ defines */
	unsigned num; /* see implementation notes in driver */
	unsigned sync; /* see implementation notes in driver */
	const char *dev_name; /* see implementation notes in driver */
	struct nvc_torch_pin_state pinstate; /* see notes in driver */
	unsigned gpio_strobe; /* GPIO connected to the ACT signal */
	bool strobe_low_act; /* strobe state active low */

	int (*power_on_callback)(struct lm356x_power_rail *pw);
	int (*power_off_callback)(struct lm356x_power_rail *pw);
};

#endif
/* __LM356X_H__ */
