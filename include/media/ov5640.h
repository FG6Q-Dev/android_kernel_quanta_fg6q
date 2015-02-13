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

#ifndef __OV5640_H__
#define __OV5640_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define OV5640_SENSOR_NAME	    "ov5640"
#define DEV(x)          "/dev/"x
#define OV5640_SENSOR_PATH     DEV(OV5640_SENSOR_NAME)

#define OV5640_WAIT_MS       0 /* special number to indicate this is wait time require */
#define OV5640_TABLE_END     1 /* special number to indicate this is end of table */
#define OV5640_WRITE_REG_DATA8      2 /* special the data width as one byte */
#define OV5640_POLL_REG_BIT         3 /* poll the bit set */

#define OV5640_MAX_RETRIES   3      /* max counter for retry I2C access */
#define OV5640_POLL_RETRIES  1000   /* max poll retry */

/* ov5640 Product ID */
#define OV5640_MODEL_ID_REG_ADDR   0x300A
#define OV5640_MODEL_ID_REG_VALUE  0x5640

#define OV5640_AF_CMD_MAIN_REG  0x3022
#define OV5640_AF_CMD_ACK_REG   0x3023
#define OV5640_AF_CMD_PARA0_REG 0x3024
#define OV5640_AF_CMD_PARA1_REG 0x3025
#define OV5640_AF_CMD_PARA2_REG 0x3026
#define OV5640_AF_CMD_PARA3_REG 0x3027
#define OV5640_AF_CMD_PARA4_REG 0x3028
#define OV5640_AF_FW_STATUS_REG 0x3029

#define OV5640_AF_DATA_LAUNCH_TOUCH_FOCUS_ZONE  0x81
#define OV5640_AF_DATA_ENABLE_CUST_FOCUS_ZONE   0x8F
#define OV5640_AF_DATA_CONFIGURE_FOCUS_ZONE_0   0x90
#define OV5640_AF_DATA_LAUNCH_CUST_FOCUS_ZONE   0x9F

#define OV5640_VVF_MAX_X    80
#define OV5640_VVF_MAX_Y    60

#define OV5640_IOCTL_SET_MODE           _IOW('o', 1, struct ov5640_mode)
#define OV5640_IOCTL_GET_STATUS         _IOR('o', 2, __u8)
#define OV5640_IOCTL_SET_COLOR_EFFECT   _IOW('o', 3, __u8)
#define OV5640_IOCTL_SET_WHITE_BALANCE  _IOW('o', 4, __u8)
#define OV5640_IOCTL_SET_SCENE_MODE     _IOW('o', 5, __u8)
#define OV5640_IOCTL_SET_AF_MODE        _IOW('o', 6, __u8)
#define OV5640_IOCTL_GET_AF_STATUS      _IOR('o', 7, __u8)
#define OV5640_IOCTL_SET_EXPOSURE       _IOW('o', 8, int)
#define OV5640_IOCTL_SET_3A_LOCK            _IOW('o', 9,  __u8)
#define OV5640_IOCTL_SET_FLASH_MODE     _IOW('o', 10, __u8)
#define OV5640_IOCTL_GET_BRIGHTNESS     _IOR('o', 11, __u16)
#define OV5640_IOCTL_SET_AF_TRIGGER     _IOW('o', 12, __u8)
#define OV5640_IOCTL_SET_AF_WINDOW_TOUCH_MODE   _IOW('o', 13, struct ov5640_af_window)
#define OV5640_IOCTL_SET_AF_WINDOW_CUST_MODE    _IOW('o', 14, struct ov5640_af_window)
#define OV5640_IOCTL_SET_WHITE_BALANCE_FLASH	_IOW('o', 15, __u8)

enum {
    OV5640_IsYUVSensor = 1,
    OV5640_Brightness,
} ;

enum {
    OV5640_FlashOn = 0,
    OV5640_FlashOff,
    OV5640_FlashAuto,
    OV5640_FlashTorch
};

enum {
    OV5640_ColorEffect = 0,
    OV5640_Whitebalance,
    OV5640_SceneMode,
    OV5640_Exposure,
    OV5640_FlashMode,
    OV5640_FocusMode,
};

enum {
    OV5640_ColorEffect_Invalid = 0,
    OV5640_ColorEffect_Aqua,
    OV5640_ColorEffect_Blackboard,
    OV5640_ColorEffect_Mono,
    OV5640_ColorEffect_Negative,
    OV5640_ColorEffect_None,
    OV5640_ColorEffect_Posterize,
    OV5640_ColorEffect_Sepia,
    OV5640_ColorEffect_Solarize,
    OV5640_ColorEffect_Whiteboard,
    OV5640_ColorEffect_Max
};

enum {
    OV5640_WhiteBalance_Sunlight = 0,
    OV5640_WhiteBalance_Cloudy,
    OV5640_WhiteBalance_Shade,
    OV5640_WhiteBalance_Tungsten,
    OV5640_WhiteBalance_Incandescent,
    OV5640_WhiteBalance_Fluorescent,
    OV5640_WhiteBalance_Flash,
    OV5640_WhiteBalance_Horizon,
    OV5640_WhiteBalance_WarmFluorescent,
    OV5640_WhiteBalance_Twilight,

    OV5640_WhiteBalance_Auto,
    OV5640_WhiteBalance_None

};

enum {
    OV5640_SceneMode_Invalid = 0,
    OV5640_SceneMode_Auto,
    OV5640_SceneMode_Action,
    OV5640_SceneMode_Portrait,
    OV5640_SceneMode_Landscape,
    OV5640_SceneMode_Beach,
    OV5640_SceneMode_Candlelight,
    OV5640_SceneMode_Fireworks,
    OV5640_SceneMode_Night,
    OV5640_SceneMode_NightPortrait,
    OV5640_SceneMode_Party,
    OV5640_SceneMode_Snow,
    OV5640_SceneMode_Sports,
    OV5640_SceneMode_SteadyPhoto,
    OV5640_SceneMode_Sunset,
    OV5640_SceneMode_Theatre,
    OV5640_SceneMode_Barcode,
    OV5640_SceneMode_Backlight,
    OV5640_SceneMode_Max
};

enum {
    OV5640_FocusMode_Invalid = 0,
    OV5640_FocusMode_Auto,
    OV5640_FocusMode_Infinity,
    OV5640_FocusMode_Macro,
    OV5640_FocusMode_Hyperfocal,
    OV5640_FocusMode_Fixed,
    OV5640_FocusMode_Continuous_Video,
    OV5640_FocusMode_Continuous_Picture,
    OV5640_FocusMode_Max
};

enum {
    OV5640_Exposure_Negative_3 = 0,
    OV5640_Exposure_Negative_2,
    OV5640_Exposure_Negative_1,
    OV5640_Exposure_0,
    OV5640_Exposure_1,
    OV5640_Exposure_2,
    OV5640_Exposure_3,
    OV5640_Exposure_Max
};

struct ov5640_mode {
    int xres;
    int yres;
};

struct ov5640_af_window {
    int x_center_pos;
    int y_center_pos;
    int x_size;
    int y_size;
};

typedef struct TapToFocusInfoRec
{
    unsigned int focusmode;
    unsigned int numOfRegions;
    unsigned int left;
    unsigned int top;
    unsigned int AEWindows[4][4];
} TapToFocusInfo;

#ifdef __KERNEL__
struct ov5640_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
};

struct ov5640_platform_data {
	int (*power_on)(struct ov5640_power_rail *pw);
	int (*power_off)(struct ov5640_power_rail *pw);
};
#endif /* __KERNEL__ */

#endif  /* __OV5640_H__ */
