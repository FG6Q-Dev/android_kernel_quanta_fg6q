/*
 * drivers/video/tegra/dc/sn65dsi8x_dsi2edp.h
 *
 * Copyright (c) 2013, QUANTA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __DRIVERS_VIDEO_TEGRA_DC_SN65DSI8X_DSI2EDP_H
#define __DRIVERS_VIDEO_TEGRA_DC_SN65DSI8X_DSI2EDP_H

struct tegra_dc_sn65dsi8x_dsi2edp_data {
	struct tegra_dc_dsi_data *dsi;
	struct i2c_client *client_i2c;

	bool dsi2edp_enabled;

	struct mutex lock;
};

#endif
