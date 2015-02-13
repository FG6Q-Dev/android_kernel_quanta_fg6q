/*
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __A1040_H__
#define __A1040_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define A1040_SENSOR_NAME	    "a1040"
#define DEV(x)                  "/dev/"x
#define A1040_SENSOR_PATH     DEV(A1040_SENSOR_NAME)

#define A1040_WAIT_MS       0 /* special number to indicate this is wait time require */
#define A1040_TABLE_END     1 /* special number to indicate this is end of table */
#define A1040_WRITE_REG_DATA8      2 /* special the data width as one byte */
#define A1040_WRITE_REG_DATA16     3 /* special the data width as two byte */
#define A1040_POLL_REG_BIT         4 /* poll the bit set */

#define A1040_MAX_RETRIES   3      /* max counter for retry I2C access */
#define A1040_POLL_RETRIES  1000   /* max poll retry */

/* A1040 Product ID */
#define A1040_MODEL_ID_REG_ADDR   0x0000
#define A1040_MODEL_ID_REG_VALUE  0x2481

#define A1040_SYSMGR_NEXT_STATE_REG   0xDC00
#define A1040_SYSMGR_CURR_STATE_REG   0xDC01
#define A1040_SYS_STATE_STREAMING 0x31
#define A1040_SYS_STATE_STANDBY   0x52

#define A1040_CMD_REG     0x0080
#define A1040_HOST_CMD_1  0x0002
#define A1040_HOST_CMD_OK 0x8000

#define A1040_IOCTL_SET_MODE           _IOW('o', 1, struct a1040_mode)
#define A1040_IOCTL_GET_STATUS         _IOR('o', 2, __u8)
#define A1040_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, __u8)
#define A1040_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, __u8)
#define A1040_IOCTL_SET_EXPOSURE       _IOW('o', 5, int)
#define A1040_IOCTL_CAPTURE_CMD        _IOW('o', 6, __u8)
#define A1040_IOCTL_GET_BRIGHTNESS     _IOR('o', 7, __u16)
#define A1040_IOCTL_SET_3A_LOCK		 _IOW('o', 8, __u32)

enum {
    A1040_IsYUVSensor = 1,
    A1040_Brightness,
} ;

enum {
    A1040_ColorEffect = 0,
    A1040_Whitebalance,
    A1040_SceneMode,
    A1040_Exposure,
    A1040_FlashMode,
    A1040_FocusMode,
};

enum {
    A1040_ColorEffect_Invalid = 0,
    A1040_ColorEffect_Aqua,
    A1040_ColorEffect_Blackboard,
    A1040_ColorEffect_Mono,
    A1040_ColorEffect_Negative,
    A1040_ColorEffect_None,
    A1040_ColorEffect_Posterize,
    A1040_ColorEffect_Sepia,
    A1040_ColorEffect_Solarize,
    A1040_ColorEffect_Whiteboard,
    A1040_ColorEffect_Max
};

enum {
    A1040_WhiteBalance_Sunlight = 0,
    A1040_WhiteBalance_Cloudy,
    A1040_WhiteBalance_Shade,
    A1040_WhiteBalance_Tungsten,
    A1040_WhiteBalance_Incandescent,
    A1040_WhiteBalance_Fluorescent,
    A1040_WhiteBalance_Flash,
    A1040_WhiteBalance_Horizon,
    A1040_WhiteBalance_WarmFluorescent,
    A1040_WhiteBalance_Twilight,

    A1040_WhiteBalance_Auto,
    A1040_WhiteBalance_None
};

enum {
    A1040_Exposure_Negative_3 = 0,
    A1040_Exposure_Negative_2,
    A1040_Exposure_Negative_1,
    A1040_Exposure_0,
    A1040_Exposure_1,
    A1040_Exposure_2,
    A1040_Exposure_3,
    A1040_Exposure_Max
};

struct a1040_mode {
	int xres;
	int yres;
};

#ifdef __KERNEL__
struct a1040_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
};

struct a1040_platform_data {
	int (*power_on)(struct a1040_power_rail *pw);
	int (*power_off)(struct a1040_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /*__A1040_H__ */

