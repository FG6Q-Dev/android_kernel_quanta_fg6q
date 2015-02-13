/*
 * kernel/drivers/media/video/tegra
 *
 * A1040 sensor driver
 *
 * Copyright (C) 2011 NVIDIA Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <media/a1040.h>

struct a1040_reg {
    u8  op;
    u16 addr;
    u16 val;
};

struct a1040_info {
    int mode;
    struct a1040_power_rail	power;
    struct i2c_client *i2c_client;
    struct a1040_platform_data *pdata;
};

static struct a1040_info *info;
//static int initial = 0;
static int pre_wb_mode = A1040_WhiteBalance_None;

static struct a1040_reg mode_1280x960[] = {
// RESET_REGISTER
    {A1040_WRITE_REG_DATA16, 0x301A, 0x0234},
    {A1040_WAIT_MS, 0x0000, 10},

//[Step2-PLL_Timing]
    {A1040_WRITE_REG_DATA16, 0x098E, 0x1000},
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC97E},
    {A1040_WRITE_REG_DATA16, 0x0990, 0x0100},
    {A1040_WRITE_REG_DATA16, 0xC980, 0x0120}, 	// CAM_SYSCTL_PLL_DIVIDER_M_N
    {A1040_WRITE_REG_DATA16, 0xC982, 0x0700}, 	// CAM_SYSCTL_PLL_DIVIDER_P

// LOAD=Timing_settings
    {A1040_WRITE_REG_DATA16, 0xC800, 0x0004}, 	// CAM_SENSOR_CFG_Y_ADDR_START
    {A1040_WRITE_REG_DATA16, 0xC802, 0x0004}, 	// CAM_SENSOR_CFG_X_ADDR_START
    {A1040_WRITE_REG_DATA16, 0xC804, 0x03CB}, 	// CAM_SENSOR_CFG_Y_ADDR_END
    {A1040_WRITE_REG_DATA16, 0xC806, 0x050B}, 	// CAM_SENSOR_CFG_X_ADDR_END
    {A1040_WRITE_REG_DATA16, 0xC80C, 0x0001}, 	// CAM_SENSOR_CFG_ROW_SPEED
    {A1040_WRITE_REG_DATA16, 0xC80E, 0x00DB}, 	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MIN
    {A1040_WRITE_REG_DATA16, 0xC810, 0x05B3}, 	// CAM_SENSOR_CFG_FINE_INTEG_TIME_MAX
    {A1040_WRITE_REG_DATA16, 0xC812, 0x03EE}, 	// CAM_SENSOR_CFG_FRAME_LENGTH_LINES
    {A1040_WRITE_REG_DATA16, 0xC814, 0x0636}, 	// CAM_SENSOR_CFG_LINE_LENGTH_PCK
    {A1040_WRITE_REG_DATA16, 0xC816, 0x0060}, 	// CAM_SENSOR_CFG_FINE_CORRECTION
    {A1040_WRITE_REG_DATA16, 0xC818, 0x03C3}, 	// CAM_SENSOR_CFG_CPIPE_LAST_ROW
    {A1040_WRITE_REG_DATA16, 0xC834, 0x0000}, 	// CAM_SENSOR_CONTROL_READ_MODE
    {A1040_WRITE_REG_DATA16, 0xC854, 0x0000}, 	// CAM_CROP_WINDOW_XOFFSET
    {A1040_WRITE_REG_DATA16, 0xC856, 0x0000}, 	// CAM_CROP_WINDOW_YOFFSET
    {A1040_WRITE_REG_DATA16, 0xC858, 0x0500}, 	// CAM_CROP_WINDOW_WIDTH
    {A1040_WRITE_REG_DATA16, 0xC85A, 0x03C0}, 	// CAM_CROP_WINDOW_HEIGHT
//    {A1040_WRITE_REG_DATA16, 0x098E, 0xC85C},
    {A1040_WRITE_REG_DATA8, 0xC85C, 0x03}, 	// CAM_CROP_CROPMODE
    {A1040_WRITE_REG_DATA16, 0xC868, 0x0500}, 	// CAM_OUTPUT_WIDTH
    {A1040_WRITE_REG_DATA16, 0xC86A, 0x03C0}, 	// CAM_OUTPUT_HEIGHT
    {A1040_WRITE_REG_DATA16, 0xC88C, 0x1E02}, 	// CAM_AET_MAX_FRAME_RATE
    {A1040_WRITE_REG_DATA16, 0xC88E, 0x0F00}, 	// CAM_AET_MIN_FRAME_RATE
    {A1040_WRITE_REG_DATA16, 0xC914, 0x0000}, 	// CAM_STAT_AWB_CLIP_WINDOW_XSTART
    {A1040_WRITE_REG_DATA16, 0xC916, 0x0000}, 	// CAM_STAT_AWB_CLIP_WINDOW_YSTART
    {A1040_WRITE_REG_DATA16, 0xC918, 0x04FF}, 	// CAM_STAT_AWB_CLIP_WINDOW_XEND
    {A1040_WRITE_REG_DATA16, 0xC91A, 0x03BF}, 	// CAM_STAT_AWB_CLIP_WINDOW_YEND
    {A1040_WRITE_REG_DATA16, 0xC91C, 0x0000}, 	// CAM_STAT_AE_INITIAL_WINDOW_XSTART
    {A1040_WRITE_REG_DATA16, 0xC91E, 0x0000}, 	// CAM_STAT_AE_INITIAL_WINDOW_YSTART
    {A1040_WRITE_REG_DATA16, 0xC920, 0x00FF}, 	// CAM_STAT_AE_INITIAL_WINDOW_XEND
    {A1040_WRITE_REG_DATA16, 0xC922, 0x00BF}, 	// CAM_STAT_AE_INITIAL_WINDOW_YEND
//    {A1040_WRITE_REG_DATA16, 0x098E, 0xE801},
    {A1040_WRITE_REG_DATA8, 0xE801, 0x00},		//    {A1040_WRITE_REG_DATA16, 0xE801, 0x00 	// AUTO_BINNING_MODE

// [Step3-Recommended]
// [Sensor optimization]
    {A1040_WRITE_REG_DATA16, 0x316A, 0x8270}, 	// DAC_TXLO_ROW
    {A1040_WRITE_REG_DATA16, 0x316C, 0x8270}, 	// DAC_TXLO
    {A1040_WRITE_REG_DATA16, 0x3ED0, 0x2305}, 	// DAC_LD_4_5
    {A1040_WRITE_REG_DATA16, 0x3ED2, 0x77CF}, 	// DAC_LD_6_7
    {A1040_WRITE_REG_DATA16, 0x316E, 0x8202}, 	// DAC_ECL
    {A1040_WRITE_REG_DATA16, 0x3180, 0x87FF}, 	// DELTA_DK_CONTROL
    {A1040_WRITE_REG_DATA16, 0x30D4, 0x6080}, 	// COLUMN_CORRECTION
    {A1040_WRITE_REG_DATA16, 0xA802, 0x0008}, 	// AE_TRACK_MODE
// LOAD=Errata item
    {A1040_WRITE_REG_DATA16, 0x3E14, 0xFF39}, 	// SAMP_COL_PUP2
// LOAD=Errata item
    {A1040_WRITE_REG_DATA16, 0x301A, 0x8234}, 	// RESET_REGISTER

//[Step4-APGA]
//[TP101_SOC1040_APGA] [APGA Settings 90% 2013/08/07 06:45:57]
    {A1040_WRITE_REG_DATA16, 0x098E, 0x495E}, 	// LOGICAL_ADDRESS_ACCESS [CAM_PGA_PGA_CONTROL]
    {A1040_WRITE_REG_DATA16, 0xC95E, 0x0000}, 	// CAM_PGA_PGA_CONTROL
    {A1040_WRITE_REG_DATA16, 0x3640, 0x02B0 },  //  P_G1_P0Q0
    {A1040_WRITE_REG_DATA16, 0x3642, 0x844A },  //  P_G1_P0Q1
    {A1040_WRITE_REG_DATA16, 0x3644, 0x12D1 }, 	//  P_G1_P0Q2
    {A1040_WRITE_REG_DATA16, 0x3646, 0xA5CB }, 	//  P_G1_P0Q3
    {A1040_WRITE_REG_DATA16, 0x3648, 0xAF8E },  //  P_G1_P0Q4
    {A1040_WRITE_REG_DATA16, 0x364A, 0x00F0 }, 	//  P_R_P0Q0
    {A1040_WRITE_REG_DATA16, 0x364C, 0xB68A }, 	//  P_R_P0Q1
    {A1040_WRITE_REG_DATA16, 0x364E, 0x31D1 }, 	//  P_R_P0Q2
    {A1040_WRITE_REG_DATA16, 0x3650, 0x77EA }, 	//  P_R_P0Q3
    {A1040_WRITE_REG_DATA16, 0x3652, 0xCBAE }, 	//  P_R_P0Q4
    {A1040_WRITE_REG_DATA16, 0x3654, 0x0190 }, 	//  P_B_P0Q0
    {A1040_WRITE_REG_DATA16, 0x3656, 0x782B }, 	//  P_B_P0Q1
    {A1040_WRITE_REG_DATA16, 0x3658, 0x5B10 }, 	//  P_B_P0Q2
    {A1040_WRITE_REG_DATA16, 0x365A, 0xB8AD }, 	//  P_B_P0Q3
    {A1040_WRITE_REG_DATA16, 0x365C, 0xA00C }, 	//  P_B_P0Q4
    {A1040_WRITE_REG_DATA16, 0x365E, 0x00B0 }, 	//  P_G2_P0Q0
    {A1040_WRITE_REG_DATA16, 0x3660, 0xAC8B }, 	//  P_G2_P0Q1
    {A1040_WRITE_REG_DATA16, 0x3662, 0x1CF1 }, 	//  P_G2_P0Q2
    {A1040_WRITE_REG_DATA16, 0x3664, 0xB0A4 }, 	//  P_G2_P0Q3
    {A1040_WRITE_REG_DATA16, 0x3666, 0xAD6F }, 	//  P_G2_P0Q4
    {A1040_WRITE_REG_DATA16, 0x3680, 0xFCEB }, 	//  P_G1_P1Q0
    {A1040_WRITE_REG_DATA16, 0x3682, 0xD7AA }, 	//  P_G1_P1Q1
    {A1040_WRITE_REG_DATA16, 0x3684, 0x1AAF }, 	//  P_G1_P1Q2
    {A1040_WRITE_REG_DATA16, 0x3686, 0x1B8C }, 	//  P_G1_P1Q3
    {A1040_WRITE_REG_DATA16, 0x3688, 0xBAEF }, 	//  P_G1_P1Q4
    {A1040_WRITE_REG_DATA16, 0x368A, 0x914C }, 	//  P_R_P1Q0
    {A1040_WRITE_REG_DATA16, 0x368C, 0x648A }, 	//  P_R_P1Q1
    {A1040_WRITE_REG_DATA16, 0x368E, 0x07CF }, 	//  P_R_P1Q2
    {A1040_WRITE_REG_DATA16, 0x3690, 0x69CC }, 	//  P_R_P1Q3
    {A1040_WRITE_REG_DATA16, 0x3692, 0xF28F }, 	//  P_R_P1Q4
    {A1040_WRITE_REG_DATA16, 0x3694, 0x0BAB }, 	//  P_B_P1Q0
    {A1040_WRITE_REG_DATA16, 0x3696, 0x4F8B }, 	//  P_B_P1Q1
    {A1040_WRITE_REG_DATA16, 0x3698, 0xF6CC }, 	//  P_B_P1Q2
    {A1040_WRITE_REG_DATA16, 0x369A, 0xC5CE }, 	//  P_B_P1Q3
    {A1040_WRITE_REG_DATA16, 0x369C, 0xBCCD }, 	//  P_B_P1Q4
    {A1040_WRITE_REG_DATA16, 0x369E, 0x098A }, 	//  P_G2_P1Q0
    {A1040_WRITE_REG_DATA16, 0x36A0, 0xDCAA }, 	//  P_G2_P1Q1
    {A1040_WRITE_REG_DATA16, 0x36A2, 0xC7CD }, 	//  P_G2_P1Q2
    {A1040_WRITE_REG_DATA16, 0x36A4, 0x71A7 }, 	//  P_G2_P1Q3
    {A1040_WRITE_REG_DATA16, 0x36A6, 0x8A2C }, 	//  P_G2_P1Q4
    {A1040_WRITE_REG_DATA16, 0x36C0, 0x2EF1 }, 	//  P_G1_P2Q0
    {A1040_WRITE_REG_DATA16, 0x36C2, 0xE92E }, 	//  P_G1_P2Q1
    {A1040_WRITE_REG_DATA16, 0x36C4, 0xBD11 }, 	//  P_G1_P2Q2
    {A1040_WRITE_REG_DATA16, 0x36C6, 0x03F1 }, 	//  P_G1_P2Q3
    {A1040_WRITE_REG_DATA16, 0x36C8, 0x2D13 }, 	//  P_G1_P2Q4
    {A1040_WRITE_REG_DATA16, 0x36CA, 0x53B1 }, 	//  P_R_P2Q0
    {A1040_WRITE_REG_DATA16, 0x36CC, 0xA3EE }, 	//  P_R_P2Q1
    {A1040_WRITE_REG_DATA16, 0x36CE, 0x9CB1 }, 	//  P_R_P2Q2
    {A1040_WRITE_REG_DATA16, 0x36D0, 0x5A8F }, 	//  P_R_P2Q3
    {A1040_WRITE_REG_DATA16, 0x36D2, 0x1673 }, 	//  P_R_P2Q4
    {A1040_WRITE_REG_DATA16, 0x36D4, 0x0F51 }, 	//  P_B_P2Q0
    {A1040_WRITE_REG_DATA16, 0x36D6, 0x80EF }, 	//  P_B_P2Q1
    {A1040_WRITE_REG_DATA16, 0x36D8, 0xE411 }, 	//  P_B_P2Q2
    {A1040_WRITE_REG_DATA16, 0x36DA, 0x1211 }, 	//  P_B_P2Q3
    {A1040_WRITE_REG_DATA16, 0x36DC, 0x60F3 }, 	//  P_B_P2Q4
    {A1040_WRITE_REG_DATA16, 0x36DE, 0x3111 }, 	//  P_G2_P2Q0
    {A1040_WRITE_REG_DATA16, 0x36E0, 0xB64D }, 	//  P_G2_P2Q1
    {A1040_WRITE_REG_DATA16, 0x36E2, 0xE4F1 }, 	//  P_G2_P2Q2
    {A1040_WRITE_REG_DATA16, 0x36E4, 0x31B0 }, 	//  P_G2_P2Q3
    {A1040_WRITE_REG_DATA16, 0x36E6, 0x4B53 }, 	//  P_G2_P2Q4
    {A1040_WRITE_REG_DATA16, 0x3700, 0xC38D }, 	//  P_G1_P3Q0
    {A1040_WRITE_REG_DATA16, 0x3702, 0x7BEB }, 	//  P_G1_P3Q1
    {A1040_WRITE_REG_DATA16, 0x3704, 0xE1B0 }, 	//  P_G1_P3Q2
    {A1040_WRITE_REG_DATA16, 0x3706, 0x3F4C }, 	//  P_G1_P3Q3
    {A1040_WRITE_REG_DATA16, 0x3708, 0x65D0 }, 	//  P_G1_P3Q4
    {A1040_WRITE_REG_DATA16, 0x370A, 0xE7CD }, 	//  P_R_P3Q0
    {A1040_WRITE_REG_DATA16, 0x370C, 0xE7AD }, 	//  P_R_P3Q1
    {A1040_WRITE_REG_DATA16, 0x370E, 0xF3B0 }, 	//  P_R_P3Q2
    {A1040_WRITE_REG_DATA16, 0x3710, 0x37EF }, 	//  P_R_P3Q3
    {A1040_WRITE_REG_DATA16, 0x3712, 0x0CD2 }, 	//  P_R_P3Q4
    {A1040_WRITE_REG_DATA16, 0x3714, 0x022C }, 	//  P_B_P3Q0
    {A1040_WRITE_REG_DATA16, 0x3716, 0x87AE }, 	//  P_B_P3Q1
    {A1040_WRITE_REG_DATA16, 0x3718, 0xA0B0 }, 	//  P_B_P3Q2
    {A1040_WRITE_REG_DATA16, 0x371A, 0x75B0 }, 	//  P_B_P3Q3
    {A1040_WRITE_REG_DATA16, 0x371C, 0x6F91 }, 	//  P_B_P3Q4
    {A1040_WRITE_REG_DATA16, 0x371E, 0xD9EC }, 	//  P_G2_P3Q0
    {A1040_WRITE_REG_DATA16, 0x3720, 0x198D }, 	//  P_G2_P3Q1
    {A1040_WRITE_REG_DATA16, 0x3722, 0xE3B0 }, 	//  P_G2_P3Q2
    {A1040_WRITE_REG_DATA16, 0x3724, 0x18CE }, 	//  P_G2_P3Q3
    {A1040_WRITE_REG_DATA16, 0x3726, 0x6B91 }, 	//  P_G2_P3Q4
    {A1040_WRITE_REG_DATA16, 0x3740, 0xE210 }, 	//  P_G1_P4Q0
    {A1040_WRITE_REG_DATA16, 0x3742, 0x29F0 }, 	//  P_G1_P4Q1
    {A1040_WRITE_REG_DATA16, 0x3744, 0x7132 }, 	//  P_G1_P4Q2
    {A1040_WRITE_REG_DATA16, 0x3746, 0xC451 }, 	//  P_G1_P4Q3
    {A1040_WRITE_REG_DATA16, 0x3748, 0xE0B3 }, 	//  P_G1_P4Q4
    {A1040_WRITE_REG_DATA16, 0x374A, 0xBC50 }, 	//  P_R_P4Q0
    {A1040_WRITE_REG_DATA16, 0x374C, 0x508F }, 	//  P_R_P4Q1
    {A1040_WRITE_REG_DATA16, 0x374E, 0x4372 }, 	//  P_R_P4Q2
    {A1040_WRITE_REG_DATA16, 0x3750, 0xF4CF }, 	//  P_R_P4Q3
    {A1040_WRITE_REG_DATA16, 0x3752, 0xA1D4 }, 	//  P_R_P4Q4
    {A1040_WRITE_REG_DATA16, 0x3754, 0xA1D0 }, 	//  P_B_P4Q0
    {A1040_WRITE_REG_DATA16, 0x3756, 0x21B0 }, 	//  P_B_P4Q1
    {A1040_WRITE_REG_DATA16, 0x3758, 0x5793 }, 	//  P_B_P4Q2
    {A1040_WRITE_REG_DATA16, 0x375A, 0x8E52 }, 	//  P_B_P4Q3
    {A1040_WRITE_REG_DATA16, 0x375C, 0xF814 }, 	//  P_B_P4Q4
    {A1040_WRITE_REG_DATA16, 0x375E, 0x8271 }, 	//  P_G2_P4Q0
    {A1040_WRITE_REG_DATA16, 0x3760, 0x27AF }, 	//  P_G2_P4Q1
    {A1040_WRITE_REG_DATA16, 0x3762, 0x1933 }, 	//  P_G2_P4Q2
    {A1040_WRITE_REG_DATA16, 0x3764, 0xD7B0 }, 	//  P_G2_P4Q3
    {A1040_WRITE_REG_DATA16, 0x3766, 0xB5D4 }, 	//  P_G2_P4Q4
    {A1040_WRITE_REG_DATA16, 0x3784, 0x028C }, 	//  CENTER_COLUMN
    {A1040_WRITE_REG_DATA16, 0x3782, 0x01EC }, 	//  CENTER_ROW
    {A1040_WRITE_REG_DATA16, 0x37C0, 0xBF69 }, 	//  P_GR_Q5
    {A1040_WRITE_REG_DATA16, 0x37C2, 0xFBE9 }, 	//  P_RD_Q5
    {A1040_WRITE_REG_DATA16, 0x37C4, 0xCE6A }, 	//  P_BL_Q5
    {A1040_WRITE_REG_DATA16, 0x37C6, 0x8549 }, 	//  P_GB_Q5
    {A1040_WRITE_REG_DATA16, 0x098E, 0x0000 }, 	//  LOGICAL addressing
    {A1040_WRITE_REG_DATA16, 0xC960, 0x0B22 }, 	//  CAM_PGA_L_CONFIG_COLOUR_TEMP
    {A1040_WRITE_REG_DATA16, 0xC962, 0x78A8 }, 	//  CAM_PGA_L_CONFIG_GREEN_RED_Q14
    {A1040_WRITE_REG_DATA16, 0xC964, 0x50B8 }, 	//  CAM_PGA_L_CONFIG_RED_Q14
    {A1040_WRITE_REG_DATA16, 0xC966, 0x771E }, 	//  CAM_PGA_L_CONFIG_GREEN_BLUE_Q14
    {A1040_WRITE_REG_DATA16, 0xC968, 0x7384 }, 	//  CAM_PGA_L_CONFIG_BLUE_Q14
    {A1040_WRITE_REG_DATA16, 0xC96A, 0x1004 }, 	//  CAM_PGA_M_CONFIG_COLOUR_TEMP
    {A1040_WRITE_REG_DATA16, 0xC96C, 0x7F41 }, 	//  CAM_PGA_M_CONFIG_GREEN_RED_Q14
    {A1040_WRITE_REG_DATA16, 0xC96E, 0x7F04 }, 	//  CAM_PGA_M_CONFIG_RED_Q14
    {A1040_WRITE_REG_DATA16, 0xC970, 0x7F7B }, 	//  CAM_PGA_M_CONFIG_GREEN_BLUE_Q14
    {A1040_WRITE_REG_DATA16, 0xC972, 0x7E63 }, 	//  CAM_PGA_M_CONFIG_BLUE_Q14
    {A1040_WRITE_REG_DATA16, 0xC974, 0x1964 }, 	//  CAM_PGA_R_CONFIG_COLOUR_TEMP
    {A1040_WRITE_REG_DATA16, 0xC976, 0x7204 }, 	// CAM_PGA_R_CONFIG_GREEN_RED_Q14
    {A1040_WRITE_REG_DATA16, 0xC978, 0x6B90 }, 	// CAM_PGA_R_CONFIG_RED_Q14
    {A1040_WRITE_REG_DATA16, 0xC97A, 0x7EEC }, 	// CAM_PGA_R_CONFIG_GREEN_BLUE_Q14
    {A1040_WRITE_REG_DATA16, 0xC97C, 0x6B90 }, 	// CAM_PGA_R_CONFIG_BLUE_Q14
    {A1040_WRITE_REG_DATA16, 0xC95E, 0x0003 }, 	//  CAM_PGA_PGA_CONTROL

    // r:1.0, G:1.25, B:1.22 , e:1.424
    {A1040_WRITE_REG_DATA16, 0x300A, 0x03EE}, 	// FRAME_LENGTH_LINES
    {A1040_WRITE_REG_DATA16, 0x3012, 0x0064}, 	// COARSE_INTEGRATION_TIME
    {A1040_WRITE_REG_DATA16, 0x305A, 0x1020}, 	// RED_GAIN
    {A1040_WRITE_REG_DATA16, 0x3056, 0x1028}, 	// GREEN1_GAIN
    {A1040_WRITE_REG_DATA16, 0x305C, 0x1028}, 	// GREEN2_GAIN
    {A1040_WRITE_REG_DATA16, 0x3058, 0x1027}, 	// BLUE_GAIN
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC87A}, 	// LOGICAL_ADDRESS_ACCESS 
    {A1040_WRITE_REG_DATA8,  0xC87A, 0x34}, 	// CAM_AET_TARGET_AVERAGE_LUMA
    {A1040_WRITE_REG_DATA8,  0xC87B, 0x3A}, 	// CAM_AET_TARGET_AVERAGE_LUMA_DARK
    {A1040_WRITE_REG_DATA16, 0xC87C, 0x0010}, 	// CAM_AET_BLACK_CLIPPING_TARGET

// [Step4-APGA]
    {A1040_WRITE_REG_DATA16, 0x098E, 0x0000}, 	// LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0xC95E, 0x0003}, 	// CAM_PGA_PGA_CONTROL

// [Step5-AWB_CCM]1: LOAD=CCM
    {A1040_WRITE_REG_DATA16, 0x098E, 0x4892 }, 	// LOGICAL_ADDRESS_ACCESS [CAM_AWB_CCM_L_0]
    {A1040_WRITE_REG_DATA16, 0xC892, 0x01F2 }, 	// CAM_AWB_CCM_L_0
    {A1040_WRITE_REG_DATA16, 0xC8A4, 0x016F }, 	// CAM_AWB_CCM_M_0
    {A1040_WRITE_REG_DATA16, 0xC8B6, 0x01D9 }, 	// CAM_AWB_CCM_R_0
    {A1040_WRITE_REG_DATA16, 0xC8DA, 0x004D }, 	// CAM_AWB_LL_CCM_0
    {A1040_WRITE_REG_DATA16, 0xC894, 0xFF6C }, 	// CAM_AWB_CCM_L_1
    {A1040_WRITE_REG_DATA16, 0xC8A6, 0xFF59 }, 	// CAM_AWB_CCM_M_1
    {A1040_WRITE_REG_DATA16, 0xC8B8, 0xFF26 }, 	// CAM_AWB_CCM_R_1
    {A1040_WRITE_REG_DATA16, 0xC8DC, 0x0096 }, 	// CAM_AWB_LL_CCM_1
    {A1040_WRITE_REG_DATA16, 0xC896, 0xFFCA }, 	// CAM_AWB_CCM_L_2
    {A1040_WRITE_REG_DATA16, 0xC8A8, 0xFFFC }, 	// CAM_AWB_CCM_M_2
    {A1040_WRITE_REG_DATA16, 0xC8BA, 0xFFF3 }, 	// CAM_AWB_CCM_R_2
    {A1040_WRITE_REG_DATA16, 0xC8DE, 0x001D }, 	// CAM_AWB_LL_CCM_2
    {A1040_WRITE_REG_DATA16, 0xC898, 0xFFAC }, 	// CAM_AWB_CCM_L_3
    {A1040_WRITE_REG_DATA16, 0xC8AA, 0xFFA3 }, 	// CAM_AWB_CCM_M_3
    {A1040_WRITE_REG_DATA16, 0xC8BC, 0xFFB3 }, 	// CAM_AWB_CCM_R_3
    {A1040_WRITE_REG_DATA16, 0xC8E0, 0x004D }, 	// CAM_AWB_LL_CCM_3
    {A1040_WRITE_REG_DATA16, 0xC89A, 0x0138 }, 	// CAM_AWB_CCM_L_4
    {A1040_WRITE_REG_DATA16, 0xC8AC, 0x0160 }, 	// CAM_AWB_CCM_M_4
    {A1040_WRITE_REG_DATA16, 0xC8BE, 0x0132 }, 	// CAM_AWB_CCM_R_4
    {A1040_WRITE_REG_DATA16, 0xC8E2, 0x0096 }, 	// CAM_AWB_LL_CCM_4
    {A1040_WRITE_REG_DATA16, 0xC89C, 0x0009 }, 	// CAM_AWB_CCM_L_5
    {A1040_WRITE_REG_DATA16, 0xC8AE, 0x0004 }, 	// CAM_AWB_CCM_M_5
    {A1040_WRITE_REG_DATA16, 0xC8C0, 0xFFE8 }, 	// CAM_AWB_CCM_R_5
    {A1040_WRITE_REG_DATA16, 0xC8E4, 0x001D }, 	// CAM_AWB_LL_CCM_5
    {A1040_WRITE_REG_DATA16, 0xC89E, 0xFFC1 }, 	// CAM_AWB_CCM_L_6
    {A1040_WRITE_REG_DATA16, 0xC8B0, 0xFFC0 }, 	// CAM_AWB_CCM_M_6
    {A1040_WRITE_REG_DATA16, 0xC8C2, 0xFFDA }, 	// CAM_AWB_CCM_R_6
    {A1040_WRITE_REG_DATA16, 0xC8E6, 0x004D }, 	// CAM_AWB_LL_CCM_6
    {A1040_WRITE_REG_DATA16, 0xC8A0, 0xFF1D }, 	// CAM_AWB_CCM_L_7
    {A1040_WRITE_REG_DATA16, 0xC8B2, 0xFF45 }, 	// CAM_AWB_CCM_M_7
    {A1040_WRITE_REG_DATA16, 0xC8C4, 0xFECD }, 	// CAM_AWB_CCM_R_7
    {A1040_WRITE_REG_DATA16, 0xC8E8, 0x0096 }, 	// CAM_AWB_LL_CCM_7
    {A1040_WRITE_REG_DATA16, 0xC8A2, 0x01D4 }, 	// CAM_AWB_CCM_L_8
    {A1040_WRITE_REG_DATA16, 0xC8B4, 0x021B }, 	// CAM_AWB_CCM_M_8
    {A1040_WRITE_REG_DATA16, 0xC8C6, 0x02C2 }, 	// CAM_AWB_CCM_R_8
    {A1040_WRITE_REG_DATA16, 0xC8EA, 0x001D }, 	// CAM_AWB_LL_CCM_8
    {A1040_WRITE_REG_DATA16, 0xC8C8, 0x0075 }, 	// CAM_AWB_CCM_L_RG_GAIN
    {A1040_WRITE_REG_DATA16, 0xC8CA, 0x011C }, 	// CAM_AWB_CCM_L_BG_GAIN
    {A1040_WRITE_REG_DATA16, 0xC8CC, 0x009A }, 	// CAM_AWB_CCM_M_RG_GAIN
    {A1040_WRITE_REG_DATA16, 0xC8CE, 0x0105 }, 	// CAM_AWB_CCM_M_BG_GAIN
    {A1040_WRITE_REG_DATA16, 0xC8D0, 0x00A4 }, 	// CAM_AWB_CCM_R_RG_GAIN
    {A1040_WRITE_REG_DATA16, 0xC8D2, 0x00AC }, 	// CAM_AWB_CCM_R_BG_GAIN
    {A1040_WRITE_REG_DATA16, 0xC8D4, 0x0A8C }, 	// CAM_AWB_CCM_L_CTEMP
    {A1040_WRITE_REG_DATA16, 0xC8D6, 0x0F0A }, 	// CAM_AWB_CCM_M_CTEMP
    {A1040_WRITE_REG_DATA16, 0xC8D8, 0x1964 }, 	// CAM_AWB_CCM_R_CTEMP

// LOAD=AWB
    {A1040_WRITE_REG_DATA16, 0xC914, 0x0000 }, 	// CAM_STAT_AWB_CLIP_WINDOW_XSTART
    {A1040_WRITE_REG_DATA16, 0xC916, 0x0000 }, 	// CAM_STAT_AWB_CLIP_WINDOW_YSTART
    {A1040_WRITE_REG_DATA16, 0xC918, 0x04FF }, 	// CAM_STAT_AWB_CLIP_WINDOW_XEND
    {A1040_WRITE_REG_DATA16, 0xC91A, 0x02CF }, 	// CAM_STAT_AWB_CLIP_WINDOW_YEND
    {A1040_WRITE_REG_DATA16, 0xC904, 0x0033 }, 	// CAM_AWB_AWB_XSHIFT_PRE_ADJ
    {A1040_WRITE_REG_DATA16, 0xC906, 0x0040 }, 	// CAM_AWB_AWB_YSHIFT_PRE_ADJ
    {A1040_WRITE_REG_DATA8, 0xC8F2, 0x04},	 // CAM_AWB_AWB_XSCALE
    {A1040_WRITE_REG_DATA8, 0xC8F3, 0x02}, // CAM_AWB_AWB_YSCALE
    {A1040_WRITE_REG_DATA16, 0xC8F4, 0x0000 }, 	// CAM_AWB_AWB_WEIGHTS_0
    {A1040_WRITE_REG_DATA16, 0xC8F6, 0x0000 }, 	// CAM_AWB_AWB_WEIGHTS_1
    {A1040_WRITE_REG_DATA16, 0xC8F8, 0x0000 }, 	// CAM_AWB_AWB_WEIGHTS_2
    {A1040_WRITE_REG_DATA16, 0xC8FA, 0x47A4 }, 	// CAM_AWB_AWB_WEIGHTS_3
    {A1040_WRITE_REG_DATA16, 0xC8FC, 0x1EB4 }, 	// CAM_AWB_AWB_WEIGHTS_4
    {A1040_WRITE_REG_DATA16, 0xC8FE, 0x2045 }, 	// CAM_AWB_AWB_WEIGHTS_5
    {A1040_WRITE_REG_DATA16, 0xC900, 0x01AC }, 	// CAM_AWB_AWB_WEIGHTS_6
    {A1040_WRITE_REG_DATA16, 0xC902, 0x007C }, 	// CAM_AWB_AWB_WEIGHTS_7
    {A1040_WRITE_REG_DATA16, 0xC926, 0x0020 }, 	// CAM_LL_START_BRIGHTNESS
    {A1040_WRITE_REG_DATA16, 0xC928, 0x009A }, 	// CAM_LL_STOP_BRIGHTNESS
    {A1040_WRITE_REG_DATA16, 0xC946, 0x0096 }, 	// CAM_LL_START_GAIN_METRIC
    {A1040_WRITE_REG_DATA16, 0xC948, 0x00F5 }, 	// CAM_LL_STOP_GAIN_METRIC
    {A1040_WRITE_REG_DATA16, 0xC952, 0x0020 }, 	// CAM_LL_START_TARGET_LUMA_BM
    {A1040_WRITE_REG_DATA16, 0xC954, 0x009A }, 	// CAM_LL_STOP_TARGET_LUMA_BM
    {A1040_WRITE_REG_DATA16, 0xC926, 0x0020 }, 	// CAM_LL_START_BRIGHTNESS
    {A1040_WRITE_REG_DATA16, 0xC928, 0x009A }, 	// CAM_LL_STOP_BRIGHTNESS
    {A1040_WRITE_REG_DATA8, 0xC92A, 0xCA}, 	// CAM_LL_START_SATURATION
    {A1040_WRITE_REG_DATA8, 0xC92B, 0xB5 }, 	// CAM_LL_END_SATURATION
    {A1040_WRITE_REG_DATA8, 0xC92C, 0x00 }, 	 // CAM_LL_START_DESATURATION
    {A1040_WRITE_REG_DATA8, 0xC92D, 0xFF }, 	// CAM_LL_END_DESATURATION
    {A1040_WRITE_REG_DATA8, 0xC92E, 0x1E }, 	// CAM_LL_START_DEMOSAIC
    {A1040_WRITE_REG_DATA8, 0xC92F, 0x03 }, 	// CAM_LL_START_AP_GAIN
    {A1040_WRITE_REG_DATA8, 0xC930, 0x06 }, 	// CAM_LL_START_AP_THRESH
    {A1040_WRITE_REG_DATA8, 0xC931, 0x78 }, 	// CAM_LL_STOP_DEMOSAIC
    {A1040_WRITE_REG_DATA8, 0xC932, 0x01 }, 	// CAM_LL_STOP_AP_GAIN
    {A1040_WRITE_REG_DATA8, 0xC933, 0x0C }, 	// CAM_LL_STOP_AP_THRESH
    {A1040_WRITE_REG_DATA8, 0xC934, 0x64  }, 	// CAM_LL_START_NR_RED
    {A1040_WRITE_REG_DATA8, 0xC935, 0x64  }, 	// CAM_LL_START_NR_GREEN
    {A1040_WRITE_REG_DATA8, 0xC936, 0x64  }, 	// CAM_LL_START_NR_BLUE
    {A1040_WRITE_REG_DATA8, 0xC937, 0x0F  }, 	// CAM_LL_START_NR_THRESH
    {A1040_WRITE_REG_DATA8, 0xC938, 0x64  }, 	// CAM_LL_STOP_NR_RED
    {A1040_WRITE_REG_DATA8, 0xC939, 0x64  }, 	// CAM_LL_STOP_NR_GREEN
    {A1040_WRITE_REG_DATA8, 0xC93A, 0x64  }, 	// CAM_LL_STOP_NR_BLUE
    {A1040_WRITE_REG_DATA8, 0xC93B, 0x32  }, 	// CAM_LL_STOP_NR_THRESH
    {A1040_WRITE_REG_DATA16, 0xC93C, 0x0020 }, 	// CAM_LL_START_CONTRAST_BM
    {A1040_WRITE_REG_DATA16, 0xC93E, 0x009A }, 	// CAM_LL_STOP_CONTRAST_BM
    {A1040_WRITE_REG_DATA16, 0xC940, 0x00FC }, 	// CAM_LL_GAMMA
    {A1040_WRITE_REG_DATA8, 0xC942, 0x3E  }, 	// CAM_LL_START_CONTRAST_GRADIENT
    {A1040_WRITE_REG_DATA8, 0xC943, 0x26  }, 	// CAM_LL_STOP_CONTRAST_GRADIENT
    {A1040_WRITE_REG_DATA8, 0xC944, 0x20  }, 	// CAM_LL_START_CONTRAST_LUMA_PERCENTAGE
    {A1040_WRITE_REG_DATA8, 0xC945, 0x19  }, 	// CAM_LL_STOP_CONTRAST_LUMA_PERCENTAGE
    {A1040_WRITE_REG_DATA16, 0xC946, 0x0096 }, 	// CAM_LL_START_GAIN_METRIC
    {A1040_WRITE_REG_DATA16, 0xC948, 0x00F3 }, 	// CAM_LL_STOP_GAIN_METRIC
    {A1040_WRITE_REG_DATA16, 0xC94A, 0x0370 }, // CAM_LL_START_FADE_TO_BLACK_LUMA
    {A1040_WRITE_REG_DATA16, 0xC94C, 0x00F0 }, 	// CAM_LL_STOP_FADE_TO_BLACK_LUMA
    {A1040_WRITE_REG_DATA16, 0xC94E, 0x00A6 }, 	// CAM_LL_CLUSTER_DC_TH_BM
    {A1040_WRITE_REG_DATA8, 0xC950, 0x05 }, 	// CAM_LL_CLUSTER_DC_GATE_PERCENTAGE
    {A1040_WRITE_REG_DATA8, 0xC951, 0x40 }, 	// CAM_LL_SUMMING_SENSITIVITY_FACTOR
    {A1040_WRITE_REG_DATA16, 0xC952, 0x0020 }, 	// CAM_LL_START_TARGET_LUMA_BM
    {A1040_WRITE_REG_DATA16, 0xC954, 0x009A }, 	// CAM_LL_STOP_TARGET_LUMA_BM
    {A1040_WRITE_REG_DATA8, 0xC90D, 0x90 }, 	// CAM_AWB_K_G_L
    {A1040_WRITE_REG_DATA8, 0xC90E, 0x90 }, 	// CAM_AWB_K_B_L
    {A1040_WRITE_REG_DATA8, 0xC90F, 0x88 }, 	// CAM_AWB_K_R_R
    {A1040_WRITE_REG_DATA8, 0xC910, 0x88 }, 	// CAM_AWB_K_G_R
    {A1040_WRITE_REG_DATA8, 0xC911, 0x80 }, 	// CAM_AWB_K_B_R

// LOAD=Step8-Features
    {A1040_WRITE_REG_DATA16, 0x098E, 0x0000}, 	// LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0xC984, 0x8040}, 	// CAM_PORT_OUTPUT_CONTROL
    {A1040_WRITE_REG_DATA16, 0x001E, 0x0777}, 	// PAD_SLEW
    {A1040_WRITE_REG_DATA16, 0xC984, 0x8041},	 	// CAM_PORT_OUTPUT_CONTROL
    {A1040_WRITE_REG_DATA16, 0xC988, 0x0F00}, 	// CAM_PORT_MIPI_TIMING_T_HS_ZERO
    {A1040_WRITE_REG_DATA16, 0xC98A, 0x0B07}, 	// CAM_PORT_MIPI_TIMING_T_HS_EXIT_HS_TRAIL
    {A1040_WRITE_REG_DATA16, 0xC98C, 0x0D01}, 	// CAM_PORT_MIPI_TIMING_T_CLK_POST_CLK_PRE
    {A1040_WRITE_REG_DATA16, 0xC98E, 0x071D}, 	// CAM_PORT_MIPI_TIMING_T_CLK_TRAIL_CLK_ZERO
    {A1040_WRITE_REG_DATA16, 0xC990, 0x0006}, 	// CAM_PORT_MIPI_TIMING_T_LPX
    {A1040_WRITE_REG_DATA16, 0xC992, 0x0A0C}, 	// CAM_PORT_MIPI_TIMING_INIT_TIMING
    {A1040_WRITE_REG_DATA16, 0x3C5A, 0x0009}, 	// MIPI_DELAY_TRIM

// LOAD=Dual Lane Mipi Receiver Init
// LOAD=Enter Suspend
    //{A1040_WRITE_REG_DATA16, 0x098E, 0xDC00},
    //{A1040_WRITE_REG_DATA16, 0x0990, 0x4000},		//
    {A1040_WRITE_REG_DATA8, 0xDC00, 0x40},    	// SYSMGR_NEXT_STATE
    {A1040_WRITE_REG_DATA16, 0x0080, 0x8002}, 	// COMMAND_REGISTER
    {A1040_WAIT_MS, 0x0000, 50},
    //{A1040_WRITE_REG_DATA16, 0x098E, 0xDC00},
    //{A1040_WRITE_REG_DATA16, 0x0990, 0x3400},		//
    {A1040_WRITE_REG_DATA8, 0xDC00, 0x34}, 	// SYSMGR_NEXT_STATE
    {A1040_WRITE_REG_DATA16, 0x0080, 0x8002}, 	// COMMAND_REGISTER
    {A1040_WAIT_MS, 0x0000, 50},

//LOAD=Leave Suspend
//   {A1040_WRITE_REG_DATA16, 0x098E, 0xDC00},
//   {A1040_WRITE_REG_DATA16, 0x0990, 0x3400},		//


    {A1040_WRITE_REG_DATA8, 0xDC00, 0x28}, 	// SYSMGR_NEXT_STATE
    {A1040_WRITE_REG_DATA16, 0x0080, 0x8002}, 	// COMMAND_REGISTER
    {A1040_WAIT_MS, 0x0000, 50},

//Config MIPI clock to discontinuous
    //{A1040_WRITE_REG_DATA16, 0x3C40, 0x783A},

    //{A1040_WAIT_MS, 0x0000, 10},
    {A1040_TABLE_END, 0x0000, 0x0000}
};

#if 0
static struct a1040_reg mode_640x480[] = {

    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg ColorEffect_None[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg ColorEffect_Aqua[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg ColorEffect_Mono[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg ColorEffect_Sepia[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg ColorEffect_Negative[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg ColorEffect_Solarize[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg ColorEffect_Posterize[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg ColorEffect_Blackboard[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg ColorEffect_Whiteboard[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};
#endif

static struct a1040_reg WhiteBalance_Sunlight[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC909}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0x0990, 0x0100}, // CAM_AWB_AWBMODE
    {A1040_WRITE_REG_DATA16, 0x098E, 0x48F0}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0xC8F0, 0x1964}, // CAM_AWB_COLOR_TEMPERATURE
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg WhiteBalance_Cloudy[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC909}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0x0990, 0x0100}, // CAM_AWB_AWBMODE
    {A1040_WRITE_REG_DATA16, 0x098E, 0x48F0}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0xC8F0, 0x1D4C}, // CAM_AWB_COLOR_TEMPERATURE
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg WhiteBalance_Incandescent[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC909}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0x0990, 0x0100}, // CAM_AWB_AWBMODE
    {A1040_WRITE_REG_DATA16, 0x098E, 0x48F0}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0xC8F0, 0x0B22}, // CAM_AWB_COLOR_TEMPERATURE
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg WhiteBalance_Fluorescent[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC909}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0x0990, 0x0100}, // CAM_AWB_AWBMODE
    {A1040_WRITE_REG_DATA16, 0x098E, 0x48F0}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0xC8F0, 0x1194}, // CAM_AWB_COLOR_TEMPERATURE
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg WhiteBalance_Auto[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC909}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA16, 0x0990, 0x0300}, // CAM_AWB_AWBMODE
    {A1040_WRITE_REG_DATA16, 0x098E, 0x48C8}, // LOGICAL_ADDRESS_ACCESS
    {A1040_TABLE_END, 0x0000, 0x0000}
};
    
static struct a1040_reg WhiteBalance_Shade[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg WhiteBalance_Tungsten[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg WhiteBalance_Flash[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg WhiteBalance_Horizon[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg WhiteBalance_WarmFluorescent[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg WhiteBalance_Twilight[] = {
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg exp_negative3[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC87A}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA8,  0xC87A, 0x1C}, // CAM_AET_TARGET_AVERAGE_LUMA
    {A1040_WRITE_REG_DATA8,  0xC87B, 0x22}, // CAM_AET_TARGET_AVERAGE_LUMA_DARK
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg exp_negative2[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC87A}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA8,  0xC87A, 0x24}, // CAM_AET_TARGET_AVERAGE_LUMA
    {A1040_WRITE_REG_DATA8,  0xC87B, 0x2A}, // CAM_AET_TARGET_AVERAGE_LUMA_DARK
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg exp_negative1[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC87A}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA8,  0xC87A, 0x2C}, // CAM_AET_TARGET_AVERAGE_LUMA
    {A1040_WRITE_REG_DATA8,  0xC87B, 0x32}, // CAM_AET_TARGET_AVERAGE_LUMA_DARK
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg exp_zero[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC87A}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA8,  0xC87A, 0x34}, // CAM_AET_TARGET_AVERAGE_LUMA
    {A1040_WRITE_REG_DATA8,  0xC87B, 0x3A}, // CAM_AET_TARGET_AVERAGE_LUMA_DARK
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg exp_one[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC87A}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA8,  0xC87A, 0x3C}, // CAM_AET_TARGET_AVERAGE_LUMA
    {A1040_WRITE_REG_DATA8,  0xC87B, 0x42}, // CAM_AET_TARGET_AVERAGE_LUMA_DARK
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg exp_two[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC87A}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA8,  0xC87A, 0x44}, // CAM_AET_TARGET_AVERAGE_LUMA
    {A1040_WRITE_REG_DATA8,  0xC87B, 0x4A}, // CAM_AET_TARGET_AVERAGE_LUMA_DARK
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg exp_three[] = {
    {A1040_WRITE_REG_DATA16, 0x098E, 0xC87A}, // LOGICAL_ADDRESS_ACCESS
    {A1040_WRITE_REG_DATA8,  0xC87A, 0x4C}, // CAM_AET_TARGET_AVERAGE_LUMA
    {A1040_WRITE_REG_DATA8,  0xC87B, 0x52}, // CAM_AET_TARGET_AVERAGE_LUMA_DARK
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg lock_3A[] = {
    {A1040_WRITE_REG_DATA8,  0xCC00, 0x01},//AE
    {A1040_WRITE_REG_DATA8,  0xCC01, 0x00},//AWB
    {A1040_TABLE_END, 0x0000, 0x0000}
};

static struct a1040_reg unlock_3A[] = {
    {A1040_WRITE_REG_DATA8,  0xCC00, 0x02},//AE
    {A1040_WRITE_REG_DATA8,  0xCC01, 0x01},//AWB
    {A1040_TABLE_END, 0x0000, 0x0000}
};

enum {
    A1040_SENSOR_MODE_1280x960,
    //A1040_SENSOR_MODE_1280x720,
    //A1040_SENSOR_MODE_640x480,
};

static struct a1040_reg *mode_table[] = {
    [A1040_SENSOR_MODE_1280x960]   = mode_1280x960,
    //[A1040_SENSOR_MODE_1280x720]   = mode_1280x720,
    //[A1040_SENSOR_MODE_640x480]   = mode_640x480,
};

static struct a1040_reg *wb_mode_table[] = {
    [A1040_WhiteBalance_Sunlight] = WhiteBalance_Sunlight,
    [A1040_WhiteBalance_Cloudy] = WhiteBalance_Cloudy,
    [A1040_WhiteBalance_Shade] = WhiteBalance_Shade,
    [A1040_WhiteBalance_Tungsten] = WhiteBalance_Tungsten,
    [A1040_WhiteBalance_Incandescent] = WhiteBalance_Incandescent,
    [A1040_WhiteBalance_Fluorescent] = WhiteBalance_Fluorescent,
    [A1040_WhiteBalance_Flash] = WhiteBalance_Flash,
    [A1040_WhiteBalance_Horizon] = WhiteBalance_Horizon,
    [A1040_WhiteBalance_WarmFluorescent] = WhiteBalance_WarmFluorescent,
    [A1040_WhiteBalance_Twilight] = WhiteBalance_Twilight,
    [A1040_WhiteBalance_Auto] = WhiteBalance_Auto,
};

static struct a1040_reg *exposure_table[] = {
    [A1040_Exposure_Negative_3] = exp_negative3,
    [A1040_Exposure_Negative_2] = exp_negative2,
    [A1040_Exposure_Negative_1] = exp_negative1,
    [A1040_Exposure_0] = exp_zero,
    [A1040_Exposure_1] = exp_one,
    [A1040_Exposure_2] = exp_two,
    [A1040_Exposure_3] = exp_three,
};
#if 0
static struct a1040_reg *color_effect_mode_table[] = {
    [A1040_ColorEffect_Aqua] = ColorEffect_Aqua,
    [A1040_ColorEffect_Blackboard] = ColorEffect_Blackboard,
    [A1040_ColorEffect_Mono] = ColorEffect_Mono,
    [A1040_ColorEffect_Negative] = ColorEffect_Negative,
    [A1040_ColorEffect_None] = ColorEffect_None,
    [A1040_ColorEffect_Posterize] = ColorEffect_Posterize,
    [A1040_ColorEffect_Sepia] = ColorEffect_Sepia,
    [A1040_ColorEffect_Solarize] = ColorEffect_Solarize,
    [A1040_ColorEffect_Whiteboard] = ColorEffect_Whiteboard,
};
#endif

static int a1040_read_reg16(struct i2c_client *client, u16 addr, u16 *val)
{
    int err;
    struct i2c_msg msg[2];
    unsigned char data[4];

    if (!client->adapter) return -ENODEV;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 2;
    msg[0].buf = data;

    /* high byte goes out first */
    data[0] = (u8) (addr >> 8);;
    data[1] = (u8) (addr & 0xff);

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 2;
    msg[1].buf = data;

    err = i2c_transfer(client->adapter, msg, 2);

    if (err != 2) return -EINVAL;

    *val = (data[0]<<8) | data[1];

    return 0;
}

static int a1040_write_reg8(struct i2c_client *client, u16 addr, u8 val)
{
    int err;
    struct i2c_msg msg;
    unsigned char data[3];
    int retry = 0;

    if (!client->adapter) return -ENODEV;

    data[0] = (u8) (addr >> 8);
    data[1] = (u8) (addr & 0xff);
    data[2] = (u8) (val & 0xff);

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 3;
    msg.buf = data;
    do {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1) return 0;
        retry++;
        pr_err("a1040 : i2c transfer failed, retrying %x %x\n",
                addr, val);
        msleep(3);
    } while (retry <= A1040_MAX_RETRIES);

    return err;
}

static int a1040_write_reg16(struct i2c_client *client, u16 addr, u16 val)
{
    int err;
    struct i2c_msg msg;
    unsigned char data[4];
    int retry = 0;

    if (!client->adapter) return -ENODEV;

    data[0] = (u8) (addr >> 8);
    data[1] = (u8) (addr & 0xff);
    data[2] = (u8) (val >> 8);
    data[3] = (u8) (val & 0xff);

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 4;
    msg.buf = data;
    do {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1) return 0;
        retry++;
        pr_err("a1040 : i2c transfer failed, retrying %x %x\n",
                addr, val);
        msleep(3);
    } while (retry <= A1040_MAX_RETRIES);

    return err;
}

#if 0
static int a1040_write_buf(struct i2c_client *client, u16 addr, u8 *buf, int len)
{
    int err = 0;
    struct i2c_msg msg;
    u8 *data;
    int retry = 0;

    if (!client->adapter) return -ENODEV;

    data = kmalloc(len+2, GFP_KERNEL);
    if (!data) {
        pr_err("%s: Cannot allocate memory for data!!!\n", __func__);
        return -ENOMEM;
    }

    memset(data, 0, len+2);

    data[0] = (u8) (addr >> 8);
    data[1] = (u8) (addr & 0xff);

    memcpy(&data[2], buf, len);

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = len+2;
    msg.buf = data;
    do {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1) return 0;
        retry++;
        pr_err("a1040 : i2c transfer failed, retrying 0x%x\n",
                addr);
        msleep(3);
    } while (retry <= A1040_MAX_RETRIES);

    kfree(data);

    return err;
}
#endif
static int a1040_poll(struct i2c_client *client, u16 addr, u16 value,
            u16 mask)
{
    u16 data;
    int try_times, err;
    int try_error;

    try_error = 0;
    for (try_times = 0; try_times < A1040_POLL_RETRIES; try_times++) {
        err = a1040_read_reg16(client, addr, &data);

        if (err) {
            if ((try_error++)<10) {
                msleep(100);
                continue;
            }else
                return err;
        }

        pr_info("poll reg 0x%x: 0x%x == 0x%x & 0x%x %d times\n", addr, value, data, mask, try_times);
        if (value == (data & mask)) return 0;
    }
    pr_err("%s: poll for %d times 0x%x == ([0x%x]=0x%x) & 0x%x failed\n", __func__,
            try_times, value, addr, data, mask);

    return -EIO;
}

static int a1040_poll_bit_set(struct i2c_client *client, u16 addr, u16 bit)
{
    return a1040_poll(client, addr, bit, bit);
}

static int a1040_write_table(struct i2c_client *client,
            const struct a1040_reg table[])
{
    int err;
    const struct a1040_reg *next;

    for (next = table; next->op != A1040_TABLE_END; next++) {
        switch (next->op) {
            case A1040_WRITE_REG_DATA8:
                err = a1040_write_reg8(client, next->addr, next->val);
                if (err) return err;
                break;
            case A1040_WRITE_REG_DATA16:
                err = a1040_write_reg16(client, next->addr, next->val);
                if (err) return err;
                break;
            case A1040_POLL_REG_BIT:
                err = a1040_poll_bit_set(client, next->addr, next->val);
                if (err) return err;
                break;
            case A1040_WAIT_MS:
                msleep(next->val);
                break;
            default:
                pr_err("%s: Invalid operation 0x%x!!!\n", __func__, next->op);
                break;
        }
    }

    return 0;
}

static int a1040_get_status(struct a1040_info *info, u16 *status)
{
    int err;
    *status = 0;

    err = a1040_read_reg16(info->i2c_client, A1040_MODEL_ID_REG_ADDR, status);
    if (err) return err;

    if (*status == A1040_MODEL_ID_REG_VALUE)
        printk("a1040: module_id= 0x%x\n", *status);
    else
        printk(KERN_ERR "%s: FAILED to get module_id, err: %x\n", __func__, err);

    return 0;
}

static int a1040_set_wb_mode(struct a1040_info *info, u8 mode)
{
    int err;

    pr_info("%s: wb mode %d\n", __func__, mode);
    if (mode < A1040_WhiteBalance_None)
    {
        err = a1040_write_table(info->i2c_client, wb_mode_table[mode]);
        if (err) return err;
    }
    else
        pr_err("%s: unsupport wb mode!!!\n", __func__);

    pre_wb_mode = mode;
    return 0;
}

static int a1040_set_exposure(struct a1040_info *info, u8 exposure)
{
    int err;

    pr_info("%s: exposure enum %d\n", __func__, exposure);
    if (exposure < A1040_Exposure_Max)
    {
        err = a1040_write_table(info->i2c_client, exposure_table[exposure]);
        if (err) return err;
    }
    else
        pr_err("%s: unsupport exposure setting!!!\n", __func__);

    return 0;
}

static int a1040_set_color_effect(struct a1040_info *info, u8 color_effect)
{
#if 0
    int err;

    pr_info("%s: color effect %d\n", __func__, color_effect);

    err = a1040_write_table(info->i2c_client, color_effect_mode_table[color_effect]);
    if (err) return err;
#endif
    return 0;
}

static int a1040_set_mode(struct a1040_info *info, struct a1040_mode *mode)
{
    int sensor_table;
    int err;

    pr_info("%s: xres %u yres %u\n",__func__, mode->xres, mode->yres);

    if (mode->xres == 1280 && mode->yres == 960)
        sensor_table = A1040_SENSOR_MODE_1280x960;
    //else if (mode->xres == 640 && mode->yres == 480)
        //sensor_table = A1040_SENSOR_MODE_640x480;
    else {
        pr_err("%s: invalid resolution supplied to set mode %d %d\n",
                __func__, mode->xres, mode->yres);
        return -EINVAL;
    }

    //if (!initial) {
        err = a1040_write_table(info->i2c_client, mode_table[sensor_table]);
        if (err) return err;

       // initial = 1;
    //}

    info->mode = sensor_table;

    return 0;
}

static long a1040_ioctl(struct file *file,
            unsigned int cmd, unsigned long arg)
{
    struct a1040_info *info = file->private_data;
    int err = 0;

    switch (cmd) {
        case A1040_IOCTL_SET_MODE:
        {
            struct a1040_mode mode;
            if (copy_from_user(&mode, (const void __user *)arg, sizeof(struct a1040_mode)))
                return -EFAULT;

            return a1040_set_mode(info, &mode);
        }
        case A1040_IOCTL_GET_STATUS:
        {
            u16 status;

            err = a1040_get_status(info, &status);
            if (err) return err;

            if (copy_to_user((void __user *)arg, &status, 2)) return -EFAULT;

            return 0;
        }
        case A1040_IOCTL_GET_BRIGHTNESS:
        {
            return 0;
        }
        case A1040_IOCTL_SET_COLOR_EFFECT:
        {
            u8 coloreffect;

            if (copy_from_user(&coloreffect, (const void __user *)arg, sizeof(coloreffect)))
                return -EFAULT;
            pr_info("a1040 SET_COLOR_EFFECT %d\n", coloreffect);
            err = a1040_set_color_effect(info, coloreffect);
            if (err) return err;

            return 0;
        }
        case A1040_IOCTL_SET_WHITE_BALANCE:
        {
            u8 whitebalance;

            if (copy_from_user(&whitebalance, (const void __user *)arg, sizeof(whitebalance))) {
                return -EFAULT;
            }
            pr_info("a1040 SET_WHITE_BALANCE %d\n", whitebalance);
            err = a1040_set_wb_mode(info, whitebalance);
            if (err) return err;

            return 0;
        }
        case A1040_IOCTL_SET_EXPOSURE:
        {
            u8 exposure = (u8)(((int)arg));
            pr_info("a1040 SET_EXPOSURE %d\n", (int)arg);
            err = a1040_set_exposure(info, exposure);
            if (err) return err;

            return 0;
        }
        case A1040_IOCTL_SET_3A_LOCK:
        {
            u8 lock;

            if (copy_from_user(&lock, (const void __user *)arg, sizeof(lock)))
                return -EFAULT;
            pr_info("a1040 LOCK_3A %d\n", lock);
            if (lock == 0)
            {
		  err = a1040_write_table(info->i2c_client, unlock_3A);
                if (err) return err;

		  msleep(50);

		  pr_info("%s: re-config wb mode %d\n", __func__, pre_wb_mode);
		  err = a1040_set_wb_mode(info, pre_wb_mode);
		  if (err) return err;
            }
            else
            {
                err = a1040_write_table(info->i2c_client, lock_3A);
                if (err) return err;
            }
            return 0;
        }
        default:
            return -EINVAL;
    }
    return 0;
}

static int a1040_suspend(struct a1040_info *info)
{
    int err = 0, cnt = 0;
    u16 read_val = 0x0000;

    pr_info("%s\n", __func__);

    /* Enter standby mode */
    err = a1040_write_reg8(info->i2c_client, A1040_SYSMGR_NEXT_STATE_REG, 0x50);
    if (err) return err;

    // (Optional) First check that the FW is ready to accept a new command
    err = a1040_read_reg16(info->i2c_client, A1040_CMD_REG, &read_val);
    if (err) return err;

    read_val &= A1040_HOST_CMD_1;
    if (read_val != 0) {
        pr_err("%s: Set State cmd bit is already set!!!\n", __func__);
        return err;
    }

    // (Mandatory) Issue the Set State command
    // We set the 'OK' bit so we can detect if the command fails
    err = a1040_write_reg16(info->i2c_client, A1040_CMD_REG, (A1040_HOST_CMD_OK | A1040_HOST_CMD_1));
    if (err) return err;

    // Wait for the FW to complete the command (clear the HOST_COMMAND_1 bit)
    do {
        err = a1040_read_reg16(info->i2c_client, A1040_CMD_REG, &read_val);
        if (err) return err;

        read_val &= A1040_HOST_CMD_1;
        mdelay(10);
        cnt++;
    } while((read_val != 0) && (cnt < 10));

    // Check the 'OK' bit to see if the command was successful
    err = a1040_read_reg16(info->i2c_client, A1040_CMD_REG, &read_val);
    if (err) return err;

    read_val &= A1040_HOST_CMD_OK;
    if (read_val != 0x8000) {
        pr_err("%s: Set State cmd failed!!!\n", __func__);
        return err;
    }

    // Wait for the FW to fully-enter standby (SYSMGR_CURRENT_STATE=0x52)
    err = a1040_read_reg16(info->i2c_client, A1040_SYSMGR_CURR_STATE_REG, &read_val);
    if (err) return err;

    read_val = read_val >> 8;
    if (read_val != A1040_SYS_STATE_STANDBY) {
        pr_err("%s: System state is not STANDBY!!! read_val = 0x%x\n", __func__, read_val);
        return err;
    }

    return 0;
}

static int a1040_resume(struct a1040_info *info)
{
    int err = 0, cnt = 0;
    u16 read_val = 0x0000;

    pr_info("%s\n", __func__);

    /* Delay 50ms for wait EXTCLK stable */
    msleep(50);

    /* Exit standby mode */
    err = a1040_write_reg8(info->i2c_client, A1040_SYSMGR_NEXT_STATE_REG, 0x54);
    if (err) return err;

    // (Optional) First check that the FW is ready to accept a new command
    err = a1040_read_reg16(info->i2c_client, A1040_CMD_REG, &read_val);
    if (err) return err;

    read_val &= A1040_HOST_CMD_1;
    if (read_val != 0) {
        pr_err("%s: Set State cmd bit is already set!!!\n", __func__);
        return err;
    }

    // (Mandatory) Issue the Set State command
    // We set the 'OK' bit so we can detect if the command fails
    err = a1040_write_reg16(info->i2c_client, A1040_CMD_REG, (A1040_HOST_CMD_OK | A1040_HOST_CMD_1));
    if (err) return err;

    // Wait for the FW to complete the command (clear the HOST_COMMAND_1 bit)
    do {
        err = a1040_read_reg16(info->i2c_client, A1040_CMD_REG, &read_val);
        if (err) return err;

        read_val &= A1040_HOST_CMD_1;
        mdelay(10);
        cnt++;
    } while((read_val != 0) && (cnt < 10));

    // Check the 'OK' bit to see if the command was successful
    err = a1040_read_reg16(info->i2c_client, A1040_CMD_REG, &read_val);
    if (err) return err;

    read_val &= A1040_HOST_CMD_OK;
    if (read_val != 0x8000) {
        pr_err("%s: Set State cmd failed!!!\n", __func__);
        return err;
    }

    err = a1040_read_reg16(info->i2c_client, A1040_SYSMGR_CURR_STATE_REG, &read_val);
    if (err) return err;

    read_val = read_val >> 8;
    if (read_val != A1040_SYS_STATE_STREAMING) {
        pr_err("%s: System state is not STREAMING!!! read_val = 0x%x\n", __func__, read_val);
        return err;
    }

    return 0;
}

static int a1040_open(struct inode *inode, struct file *file)
{
    int err;
    u16 status;
    pr_info("%s++\n", __func__);

    file->private_data = info;
    if (info->pdata && info->pdata->power_on)
        info->pdata->power_on(&info->power);

    err = a1040_get_status(info, &status);
    if (err) return err;

    err = a1040_resume(info);
    if (err) return err;

    pr_info("%s--\n", __func__);
    return 0;
}

int a1040_release(struct inode *inode, struct file *file)
{
    int err;
    pr_info("%s++\n", __func__);

    err = a1040_suspend(info);
    if (err) return err;

    if (info->pdata && info->pdata->power_off)
        info->pdata->power_off(&info->power);
    file->private_data = NULL;

    pr_info("%s--\n", __func__);
    return 0;
}

static int a1040_power_put(struct a1040_power_rail *pw)
{
    if (likely(pw->dvdd))
        regulator_put(pw->dvdd);

    if (likely(pw->avdd))
        regulator_put(pw->avdd);

    pw->dvdd = NULL;
    pw->avdd = NULL;

    return 0;
}

static int a1040_regulator_get(struct a1040_info *info,
    struct regulator **vreg, char vreg_name[])
{
    struct regulator *reg = NULL;
    int err = 0;

    reg = regulator_get(&info->i2c_client->dev, vreg_name);
    if (unlikely(IS_ERR(reg))) {
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

static int a1040_power_get(struct a1040_info *info)
{
    struct a1040_power_rail *pw = &info->power;

    a1040_regulator_get(info, &pw->dvdd, "sec_cam_1v8"); // digital 1.1v
    a1040_regulator_get(info, &pw->avdd, "sec_cam_2v8"); // analog 2.8v 

    return 0;
}

static const struct file_operations a1040_fileops = {
    .owner = THIS_MODULE,
    .open = a1040_open,
    .unlocked_ioctl = a1040_ioctl,
    .release = a1040_release,
};

static struct miscdevice a1040_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = A1040_SENSOR_NAME,
    .fops = &a1040_fileops,
};

static int a1040_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
    int err;
    pr_info("%s++\n", __func__);

    info = kzalloc(sizeof(struct a1040_info), GFP_KERNEL);
    if (!info) {
		pr_err("a1040: Unable to allocate memory!\n");
		return -ENOMEM;
    }
    info->pdata = client->dev.platform_data;
    info->i2c_client = client;
    info->mode = -1;

    i2c_set_clientdata(client, info);
	
    a1040_power_get(info);

    err = misc_register(&a1040_device);
    if (err) {
        a1040_power_put(&info->power);
        pr_err("a1040: Unable to register misc device!\n");
        kfree(info);
        return err;
    }

    pr_info("%s--\n", __func__);
    return 0;
}

static int a1040_remove(struct i2c_client *client)
{
    struct a1040_info *info;
    info = i2c_get_clientdata(client);
    a1040_power_put(&info->power);
    misc_deregister(&a1040_device);
    kfree(info);
    return 0;
}

static const struct i2c_device_id a1040_id[] = {
    { A1040_SENSOR_NAME, 0 },
    { },
};

MODULE_DEVICE_TABLE(i2c, a1040_id);

static struct i2c_driver a1040_i2c_driver = {
    .driver = {
        .name = A1040_SENSOR_NAME,
        .owner = THIS_MODULE,
    },
    .probe = a1040_probe,
    .remove = a1040_remove,
    .id_table = a1040_id,
};

static int __init a1040_init(void)
{
    return i2c_add_driver(&a1040_i2c_driver);
}

static void __exit a1040_exit(void)
{
    i2c_del_driver(&a1040_i2c_driver);
}

module_init(a1040_init);
module_exit(a1040_exit);

