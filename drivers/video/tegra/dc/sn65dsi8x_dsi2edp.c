/*
 * drivers/video/tegra/dc/sn65dsi8x_dsi2edp.c
 *
 * Copyright (c) 2013, QUANTA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/delay.h>

#include <mach/dc.h>

#include "dc_priv.h"
#include "sn65dsi8x_dsi2edp.h"
#include "dsi.h"

static struct tegra_dc_sn65dsi8x_dsi2edp_data *dsi2edp;

enum i2c_transfer_type {
	I2C_WRITE,
	I2C_READ,
};

#define DSI2EDP_TEGRA_I2C_BUS	0
#define DSI2EDP_REG_VAL(addr, val)	{(addr), (val)}

static u8 dsi2edp_config_clk[][2] = {
	DSI2EDP_REG_VAL(0x0d, 0x00), /* pLL disable */
	DSI2EDP_REG_VAL(0x0a, 0x02), /* REFCLK is 19.2MHz */
};

static u8 dsi2edp_config_dsi[][2] = {
	DSI2EDP_REG_VAL(0x10, 0x26),
	DSI2EDP_REG_VAL(0x12, 0x53), /* channel A clk range */
	DSI2EDP_REG_VAL(0x13, 0x53), /* channel B clk range */
};

static u8 dsi2edp_config_dp[][2] = {
	DSI2EDP_REG_VAL(0x5b, 0x00), /* 24bpp RGB */
};

static u8 dsi2edp_config_link[][2] = {
#if 0	/* From TI */
	DSI2EDP_REG_VAL(0x93, 0x24), /* 2 DP Lanes, SSC_SPREAD, SSC_ENABLE = 0 */
	DSI2EDP_REG_VAL(0x94, 0x80), /* HBR DP Daterate, DP_ERC */
#endif
	DSI2EDP_REG_VAL(0x93, 0x20), /* 2 DP Lanes */
	DSI2EDP_REG_VAL(0x94, 0x84), /* DP Daterate, DP_ERC */

	DSI2EDP_REG_VAL(0x0d, 0x01), /* pLL enable */
};

static u8 dsi2edp_config_link_training[][2] = {
#if 0	/* From TI */
        DSI2EDP_REG_VAL(0x5a, 0x04), /* Enh Framing */
#endif
	DSI2EDP_REG_VAL(0x96, 0x0A), /* Semi-Auto Link Training */
};

static u8 dsi2edp_config_video[][2] = {
	DSI2EDP_REG_VAL(0x20, 0x80), /* horizontal pixels on dsi channel A */
	DSI2EDP_REG_VAL(0x21, 0x07),
	DSI2EDP_REG_VAL(0x22, 0x00), /* horizontal pixels on dsi channel B */
	DSI2EDP_REG_VAL(0x23, 0x00),
	DSI2EDP_REG_VAL(0x24, 0x38), /* vertical pixels on lvds channel A */
	DSI2EDP_REG_VAL(0x25, 0x04),

	DSI2EDP_REG_VAL(0x2c, 0x20), /* HSYNC Width */
	DSI2EDP_REG_VAL(0x2d, 0x80), /* HSYNC Polarity */

	DSI2EDP_REG_VAL(0x30, 0x05), /* VSYNC Width */
	DSI2EDP_REG_VAL(0x31, 0x80), /* VSYNC Polarity */

	DSI2EDP_REG_VAL(0x34, 0x50), /* h back porch channel A */
	DSI2EDP_REG_VAL(0x36, 0x18), /* v back porch channel A */
	DSI2EDP_REG_VAL(0x38, 0x30), /* h front porch channel A */
	DSI2EDP_REG_VAL(0x3a, 0x03), /* v front porch channel A */

	DSI2EDP_REG_VAL(0x3c, 0x00), /* color bar test pattern */
	DSI2EDP_REG_VAL(0x3d, 0x00), /* Right Crop */
	DSI2EDP_REG_VAL(0x3e, 0x00), /* Left Crop */
};

static u8 dsi2edp_config_vs[][2] = {
	DSI2EDP_REG_VAL(0x5a, 0x08), /* VSTREAM_ENABLE */
};

static u8 dsi2edp_soft_reset[][2] = {
	DSI2EDP_REG_VAL(0x09, 0x01),
};

static struct i2c_driver sn65dsi8x_dsi2edp_i2c_slave_driver = {
	.driver = {
		.name = "dsi2edp_bridge",
	},
};

static struct i2c_client *sn65dsi8x_init_i2c_slave(struct tegra_dc_dsi_data *dsi)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info p_data = {
		.type = "dsi2edp_bridge",
		.addr = 0x2C,
	};
	int bus = DSI2EDP_TEGRA_I2C_BUS;
	int err = 0;

	adapter = i2c_get_adapter(bus);
	if (!adapter) {
		dev_err(&dsi->dc->ndev->dev,
			"dsi2edp: can't get adpater for bus %d\n", bus);
		err = -EBUSY;
		goto err;
	}

	client = i2c_new_device(adapter, &p_data);
	i2c_put_adapter(adapter);
	if (!client) {
		dev_err(&dsi->dc->ndev->dev,
			"dsi2edp: can't add i2c slave device\n");
		err = -EBUSY;
		goto err;
	}

	err = i2c_add_driver(&sn65dsi8x_dsi2edp_i2c_slave_driver);
	if (err) {
		dev_err(&dsi->dc->ndev->dev,
			"dsi2edp: can't add i2c slave driver\n");
		goto err_free;
	}

	return client;
err:
	return ERR_PTR(err);
err_free:
	i2c_unregister_device(client);
	return ERR_PTR(err);
}

static int sn65dsi8x_dsi2edp_init(struct tegra_dc_dsi_data *dsi)
{
	int err = 0;

	if (dsi2edp) {
		tegra_dsi_set_outdata(dsi, dsi2edp);
		return err;
	}

	dsi2edp = devm_kzalloc(&dsi->dc->ndev->dev, sizeof(*dsi2edp), GFP_KERNEL);
	if (!dsi2edp)
		return -ENOMEM;

	dsi2edp->client_i2c = sn65dsi8x_init_i2c_slave(dsi);
	if (IS_ERR_OR_NULL(dsi2edp->client_i2c)) {
		dev_err(&dsi->dc->ndev->dev,
			"dsi2edp: i2c slave setup failure\n");
	}

	dsi2edp->dsi = dsi;

	tegra_dsi_set_outdata(dsi, dsi2edp);

	mutex_init(&dsi2edp->lock);

	return err;
}

static void sn65dsi8x_dsi2edp_destroy(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_sn65dsi8x_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);

	if (!dsi2edp)
		return;

	mutex_lock(&dsi2edp->lock);
	i2c_del_driver(&sn65dsi8x_dsi2edp_i2c_slave_driver);
	i2c_unregister_device(dsi2edp->client_i2c);
	mutex_unlock(&dsi2edp->lock);
	mutex_destroy(&dsi2edp->lock);
	kfree(dsi2edp);
}

static int sn65dsi8x_dsi2edp_i2c_transfer(struct tegra_dc_sn65dsi8x_dsi2edp_data *dsi2edp,
					u8 transfers[][2], u32 no_of_tranfers,
					enum i2c_transfer_type type)
{
	struct i2c_msg *i2c_msg_transfer;
	struct i2c_client *client = dsi2edp->client_i2c;
	int err = 0;
	u32 cnt = 0;

	i2c_msg_transfer = kzalloc
		(no_of_tranfers * sizeof(*i2c_msg_transfer), GFP_KERNEL);
	if (!i2c_msg_transfer)
		return -ENOMEM;

	for (cnt = 0; cnt < no_of_tranfers; cnt++) {
		i2c_msg_transfer[cnt].addr = client->addr;
		i2c_msg_transfer[cnt].flags = type;
		i2c_msg_transfer[cnt].len = 2;
		i2c_msg_transfer[cnt].buf = transfers[cnt];
	}

	for (cnt = 0; cnt < no_of_tranfers; cnt++) {
		err = i2c_transfer(client->adapter, &i2c_msg_transfer[cnt], 1);
		if (err < 0) {
			dev_err(&dsi2edp->dsi->dc->ndev->dev,
				"dsi2edp: i2c write failed\n");
			break;
		}
		msleep(10);
	}

	kfree(i2c_msg_transfer);
	return err;
}
static void sn65dsi8x_dsi2edp_enable(struct tegra_dc_dsi_data *dsi)
{
	struct tegra_dc_sn65dsi8x_dsi2edp_data *dsi2edp = tegra_dsi_get_outdata(dsi);
	int err = 0;

	if (dsi2edp && dsi2edp->dsi2edp_enabled)
		return;

	mutex_lock(&dsi2edp->lock);

	err = sn65dsi8x_dsi2edp_i2c_transfer(dsi2edp, dsi2edp_soft_reset,
			ARRAY_SIZE(dsi2edp_soft_reset), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Soft reset failed\n");
		goto fail;
	}

	err = sn65dsi8x_dsi2edp_i2c_transfer(dsi2edp, dsi2edp_config_clk,
			ARRAY_SIZE(dsi2edp_config_clk), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Init clk failed\n");
		goto fail;
	}

	err = sn65dsi8x_dsi2edp_i2c_transfer(dsi2edp, dsi2edp_config_dsi,
			ARRAY_SIZE(dsi2edp_config_dsi), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Init dsi failed\n");
		goto fail;
	}

	err = sn65dsi8x_dsi2edp_i2c_transfer(dsi2edp, dsi2edp_config_dp,
			ARRAY_SIZE(dsi2edp_config_dp), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Init displayport failed\n");
		goto fail;
	}

	err = sn65dsi8x_dsi2edp_i2c_transfer(dsi2edp, dsi2edp_config_link,
			ARRAY_SIZE(dsi2edp_config_link), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Init link failed\n");
		goto fail;
	}

	msleep(2);
	err = sn65dsi8x_dsi2edp_i2c_transfer(dsi2edp, dsi2edp_config_link_training,
			ARRAY_SIZE(dsi2edp_config_link_training), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Init link training failed\n");
		goto fail;
	}

	msleep(10);
	err = sn65dsi8x_dsi2edp_i2c_transfer(dsi2edp, dsi2edp_config_video,
			ARRAY_SIZE(dsi2edp_config_video), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: Init video failed\n");
		goto fail;
	}

	err = sn65dsi8x_dsi2edp_i2c_transfer(dsi2edp, dsi2edp_config_vs,
			ARRAY_SIZE(dsi2edp_config_vs), I2C_WRITE);
	if (err < 0) {
		dev_err(&dsi->dc->ndev->dev, "dsi2edp: VSTREAM_ENABLE failed\n");
		goto fail;
	}

	dsi2edp->dsi2edp_enabled = true;
fail:
	mutex_unlock(&dsi2edp->lock);
}

static void sn65dsi8x_dsi2edp_disable(struct tegra_dc_dsi_data *dsi)
{
	dsi2edp->dsi2edp_enabled = false;

	/* To be done */
}

#ifdef CONFIG_PM
static void sn65dsi8x_dsi2edp_suspend(struct tegra_dc_dsi_data *dsi)
{
	dsi2edp->dsi2edp_enabled = false;

	/* To be done */
}

static void sn65dsi8x_dsi2edp_resume(struct tegra_dc_dsi_data *dsi)
{
	/* To be done */
}
#endif

struct tegra_dsi_out_ops sn65dsi8x_dsi2edp_ops = {
	.init = sn65dsi8x_dsi2edp_init,
	.destroy = sn65dsi8x_dsi2edp_destroy,
	.enable = sn65dsi8x_dsi2edp_enable,
	.disable = sn65dsi8x_dsi2edp_disable,
#ifdef CONFIG_PM
	.suspend = sn65dsi8x_dsi2edp_suspend,
	.resume = sn65dsi8x_dsi2edp_resume,
#endif
};
