// SPDX-License-Identifier: GPL-2.0-only
/*
 * Renesas Multi-Protocol PHY device driver
 *
 * Copyright (C) 2025-2026 Renesas Electronics Corporation
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/reset.h>
#include <linux/types.h>

#include <dt-bindings/phy/phy.h>

#define MPPHY_NUM_CHANNELS		4

/* Common registers */
#define MPPHY_CMNCNT1			0x80000
#define MPPHY_CMNCNT2			0x80004
#define MPPHY_PCS0REG1			0x85000
#define MPPHY_PCS0REG5			0x85010

/* Channel register base and offsets */
#define MPPHY_CHAN_BASE(ch)		(0x81000 + (ch) * 0x1000)
#define MPPHY_PXCNTXT1(ch)		(MPPHY_CHAN_BASE(ch) + 0x4)
#define MPPHY_PXCNTXT2(ch)		(MPPHY_CHAN_BASE(ch) + 0x8)
#define MPPHY_PXTEST(ch)		(MPPHY_CHAN_BASE(ch) + 0xc)
#define MPPHY_PXREFCLK(ch)		(MPPHY_CHAN_BASE(ch) + 0x14)
#define MPPHY_PXRXREQ1(ch)		(MPPHY_CHAN_BASE(ch) + 0x24)
#define MPPHY_PXRXCNT(ch)		(MPPHY_CHAN_BASE(ch) + 0x38)
#define MPPHY_PXSRAMCNT(ch)		(MPPHY_CHAN_BASE(ch) + 0x40)
#define MPPHY_PXTXREQ(ch)		(MPPHY_CHAN_BASE(ch) + 0x44)

/* PCS0REG1 register bits */
#define MPPHY_PCS0REG1_VAL		0x10000

/* PXTEST register bit */
#define MPPHY_PXTEST_BIT		0x1

/* PXRXCNT register reset value */
#define MPPHY_PXRXCNT_RESET_VAL		0x202

/* PXSRAMCNT register bits */
#define SRAM_EXT_LD_DONE		0x10

/* TCA (Type-C Adapter) Register Offsets within MP-PHY base */
#define MPPHY_USB_BASE(ch)		(0x90000 + (ch) * 0x10000)

/* Channel specific registers */
#define TCA_INTR_OFFSET(ch)		(MPPHY_USB_BASE(ch) + 0x4)
#define TCA_INTR_STS_OFFSET(ch)		(MPPHY_USB_BASE(ch) + 0x8)
#define TCA_TCPC_OFFSET(ch)		(MPPHY_USB_BASE(ch) + 0x14)
#define TCA_VBUS_CTRL_OFFSET(ch)	(MPPHY_USB_BASE(ch) + 0x40)
#define PSTATE_1_OFFSET(ch)		(MPPHY_USB_BASE(ch) + 0x54)

/* PXREFCLK register value */
#define MPPHY_PXREFCLK_VAL_ETH		0x55

/* Context settings */
#define HIGH_SPEED			0
#define SUPER_SPEED_PLUS		1

/* Firmware update */
#define MPPHY_FW_BASE			0x10000
#define MPPHY_FW_CH_OFFSET		0x20000
#define MPPHY_FW_NAME			"rcar_gen5_mp_phy.bin"

struct mp_phy_chan_priv {
	struct phy *phy;
	unsigned int lane_id;
	unsigned int protocol_id;
	enum phy_mode current_protocol;
	bool initialized;
};

struct mp_phy_priv {
	void __iomem *base;
	struct device *dev;
	struct dev_pm_domain_list *pd_list;
	struct reset_control_bulk_data resets[MPPHY_NUM_CHANNELS + 1];
	struct clk_bulk_data *clks;
	int num_clks;
	const struct firmware *fw;
	struct mp_phy_chan_priv chan[MPPHY_NUM_CHANNELS];
	u32 num_lanes[MPPHY_NUM_CHANNELS];
	u32 write_cntxt1;
	u32 cmncnt[2];
	u8 sramcnt[MPPHY_NUM_CHANNELS];
};

static void mp_phy_write(struct mp_phy_priv *priv, u32 offset, u32 value)
{
	writel(value, priv->base + offset);
}

static void mp_phy_update_bits(struct mp_phy_priv *priv, u32 offset, u32 mask, u32 value)
{
	u32 tmp;

	tmp = readl(priv->base + offset);
	tmp &= ~mask;
	tmp |= value;
	writel(tmp, priv->base + offset);
}

static int mp_phy_reg_wait(struct mp_phy_priv *priv, u32 offs, u32 mask, u32 expected)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(priv->base + offs, val,
					(val & mask) == expected, 1, 10000000);
	if (ret) {
		dev_err(priv->dev,
			"Timeout waiting for offset: 0x%x, mask: 0x%x, expected: 0x%x\n",
			offs, mask, expected);
	}

	return ret;
}

static void mp_phy_update_firmware(struct mp_phy_priv *priv, u32 channel_id)
{
	u32 offset = MPPHY_FW_BASE + MPPHY_FW_CH_OFFSET * channel_id;
	int i;

	for (i = 0; i < priv->fw->size; i += 2) {
		writew(priv->fw->data[i] | (priv->fw->data[i + 1] << 8),
		       priv->base + offset + i);
	}
}

static int mp_phy_exit(struct phy *phy)
{
	struct mp_phy_priv *priv = phy_get_drvdata(phy);
	struct mp_phy_chan_priv *chan = &priv->chan[phy->id];

	if (!chan->initialized)
		return 0;

	chan->initialized = false;
	chan->current_protocol = PHY_MODE_INVALID;

	pm_runtime_put_sync(priv->pd_list->pd_devs[phy->id]);

	return 0;
}

static int mp_phy_init_ethernet(struct mp_phy_priv *priv, u32 channel_id)
{
	mp_phy_update_firmware(priv, channel_id);

	mp_phy_write(priv, MPPHY_PXRXCNT(channel_id), MPPHY_PXRXCNT_RESET_VAL);
	mp_phy_update_bits(priv, MPPHY_PXREFCLK(channel_id),
			   MPPHY_PXREFCLK_VAL_ETH, MPPHY_PXREFCLK_VAL_ETH);
	mp_phy_update_bits(priv, MPPHY_PXRXCNT(channel_id),
			   BIT(9) | BIT(1), 0);
	mp_phy_update_bits(priv, MPPHY_PXTXREQ(channel_id),
			   BIT(19) | BIT(3), BIT(19) | BIT(3));

	return 0;
}

static int mp_phy_init_pcie4(struct mp_phy_priv *priv, u32 channel_id)
{
	if (priv->num_lanes[channel_id] == 1 || priv->num_lanes[channel_id] == 2) {
		if (channel_id == 0) {
			mp_phy_update_bits(priv, MPPHY_PXCNTXT1(0), 0x2010002, 0x2010002);
			mp_phy_update_bits(priv, MPPHY_PXCNTXT2(0), 0x2020201, 0x2020201);
			mp_phy_update_bits(priv, MPPHY_PXTXREQ(0), 0x80004, 0x80004);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(0), 0x30, 0x30);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(0), 0x4, 0x4);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(0), 0x1, 0x1);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(0), 0x202, 0x202);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(0), 0x202, 0);
		} else if (channel_id == 1) {
			mp_phy_update_bits(priv, MPPHY_PXCNTXT1(2), 0x2010002, 0x2010002);
			mp_phy_update_bits(priv, MPPHY_PXCNTXT2(2), 0x2020202, 0x2020202);
			mp_phy_update_bits(priv, MPPHY_PXTXREQ(2), 0x8, 0x8);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(2), 0x30, 0x30);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(2), 0x4, 0x4);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(2), 0x1, 0x1);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(2), 0x202, 0x202);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(2), 0x202, 0);
		}
	} else if (priv->num_lanes[channel_id] == 4) {
		if (channel_id == 0) {
			mp_phy_update_bits(priv, MPPHY_PXCNTXT1(0), 0x2010002, 0x2010002);
			mp_phy_update_bits(priv, MPPHY_PXCNTXT2(0), 0x2020201, 0x2020201);
			mp_phy_update_bits(priv, MPPHY_PXTXREQ(0), 0x8, 0x8);
			mp_phy_update_bits(priv, MPPHY_PXCNTXT1(1), 0x2010002, 0x2010002);
			mp_phy_update_bits(priv, MPPHY_PXCNTXT2(1), 0x2020201, 0x2020202);
			mp_phy_update_bits(priv, MPPHY_PXTXREQ(1), 0x8, 0x8);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(0), 0x30, 0x30);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(0), 0x4, 0x4);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(0), 0x1, 0x1);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(1), 0x30, 0x30);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(1), 0x4, 0x4);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(1), 0x1, 0x1);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(0), 0x202, 0x202);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(1), 0x202, 0x202);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(0), 0x202, 0);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(1), 0x202, 0);
		} else if (channel_id == 1) {
			mp_phy_update_bits(priv, MPPHY_PXCNTXT1(2), 0x2010002, 0x2010002);
			mp_phy_update_bits(priv, MPPHY_PXCNTXT2(2), 0x2020202, 0x2020202);
			mp_phy_update_bits(priv, MPPHY_PXTXREQ(2), 0x8, 0x8);
			mp_phy_update_bits(priv, MPPHY_PXCNTXT1(3), 0x2010002, 0x2010002);
			mp_phy_update_bits(priv, MPPHY_PXCNTXT2(3), 0x2020202, 0x2020202);
			mp_phy_update_bits(priv, MPPHY_PXTXREQ(3), 0x8, 0x8);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(2), 0x30, 0x30);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(2), 0x4, 0x4);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(2), 0x1, 0x1);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(3), 0x30, 0x30);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(3), 0x4, 0x4);
			mp_phy_update_bits(priv, MPPHY_PXREFCLK(3), 0x1, 0x1);

			mp_phy_update_bits(priv, MPPHY_PXRXCNT(2), 0x202, 0x202);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(3), 0x202, 0x202);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(2), 0x202, 0);
			mp_phy_update_bits(priv, MPPHY_PXRXCNT(3), 0x202, 0);
		}
	} else if (priv->num_lanes[channel_id] == 8) {
		mp_phy_write(priv, MPPHY_PXCNTXT1(0), 0x2010002);
		mp_phy_write(priv, MPPHY_PXCNTXT2(0), 0x2020201);
		mp_phy_write(priv, MPPHY_PXTXREQ(0), 0x8);
		mp_phy_write(priv, MPPHY_PXCNTXT1(1), 0x2010002);
		mp_phy_write(priv, MPPHY_PXCNTXT2(1), 0x2020202);
		mp_phy_write(priv, MPPHY_PXTXREQ(1), 0x8);
		mp_phy_write(priv, MPPHY_PXCNTXT1(2), 0x2010002);
		mp_phy_write(priv, MPPHY_PXCNTXT2(2), 0x2020202);
		mp_phy_write(priv, MPPHY_PXTXREQ(2), 0x8);
		mp_phy_write(priv, MPPHY_PXCNTXT1(3), 0x2010002);
		mp_phy_write(priv, MPPHY_PXCNTXT2(3), 0x2020202);
		mp_phy_write(priv, MPPHY_PXTXREQ(3), 0x8);

		mp_phy_update_bits(priv, MPPHY_PXRXCNT(0), 0x202, 0x202);
		mp_phy_update_bits(priv, MPPHY_PXRXCNT(1), 0x202, 0x202);
		mp_phy_update_bits(priv, MPPHY_PXRXCNT(2), 0x202, 0x202);
		mp_phy_update_bits(priv, MPPHY_PXRXCNT(3), 0x202, 0x202);

		mp_phy_update_bits(priv, MPPHY_PXRXCNT(0), 0x202, 0x202);
		mp_phy_update_bits(priv, MPPHY_PXRXCNT(1), 0x202, 0x202);
		mp_phy_update_bits(priv, MPPHY_PXRXCNT(2), 0x202, 0x202);
		mp_phy_update_bits(priv, MPPHY_PXRXCNT(3), 0x202, 0x202);

		mp_phy_update_bits(priv, MPPHY_PXRXCNT(0), 0x202, 0);
		mp_phy_update_bits(priv, MPPHY_PXRXCNT(1), 0x202, 0);
		mp_phy_update_bits(priv, MPPHY_PXRXCNT(2), 0x202, 0);
		mp_phy_update_bits(priv, MPPHY_PXRXCNT(3), 0x202, 0);
	}

	return 0;
}

static int mp_phy_init_usb(struct mp_phy_priv *priv, u32 channel_id)
{
	int ret;

	ret = mp_phy_reg_wait(priv, MPPHY_PXSRAMCNT(channel_id), 0x20, 0x20);
	if (ret)
		return ret;

	mp_phy_update_firmware(priv, channel_id);
	mp_phy_update_bits(priv, MPPHY_PXSRAMCNT(channel_id),
			   SRAM_EXT_LD_DONE, SRAM_EXT_LD_DONE);

	ret = mp_phy_reg_wait(priv, MPPHY_PXRXREQ1(channel_id), 0x2, 0);
	if (ret)
		return ret;

	return 0;
}

static int mp_phy_init(struct phy *phy)
{
	struct mp_phy_priv *priv = phy_get_drvdata(phy);
	struct mp_phy_chan_priv *chan = &priv->chan[phy->id];
	int ret;

	/*
	 * Note: Current source code support for Ethernet, PCIe
	 * initialization is based on the bare metal code shared
	 * by the board team.
	 */
	ret = pm_runtime_get_sync(priv->pd_list->pd_devs[phy->id]);
	if (ret < 0) {
		dev_err(priv->dev,
			"Failed to power on domain for channel %d: %d\n",
			phy->id, ret);
		return ret;
	}

	/* Check if initialized with same protocol then skip */
	if (chan->initialized && chan->current_protocol == phy->attrs.mode)
		return 0;

	if (phy->attrs.mode == PHY_MODE_PCIE)
		ret = mp_phy_init_pcie4(priv, phy->id);
	else if (phy->attrs.mode == PHY_MODE_ETHERNET)
		ret = mp_phy_init_ethernet(priv, phy->id);
	else
		ret = mp_phy_init_usb(priv, phy->id);
	if (ret)
		return ret;

	chan->initialized = true;
	chan->current_protocol = phy->attrs.mode;
	dev_info(priv->dev,
		 "Channel %d successfully initialized for protocol %d\n",
		 phy->id, phy->attrs.mode);

	return 0;
}

static int mp_phy_power_on(struct phy *phy)
{
	struct mp_phy_priv *priv = phy_get_drvdata(phy);
	struct mp_phy_chan_priv *chan = &priv->chan[phy->id];
	int ret;

	if (!chan->initialized) {
		dev_err(priv->dev, "Channel %d not initialized\n", phy->id);
		return -EINVAL;
	}

	/*
	 * The datasheet describes initialization procedure without full
	 * information about the registers. Therefore, the source code is
	 * based on the bare metal code shared by the board team.
	 */
	if (chan->protocol_id == PHY_MODE_PCIE) {
		if (priv->num_lanes[phy->id] == 1 || priv->num_lanes[phy->id] == 2) {
			mp_phy_reg_wait(priv, MPPHY_PXSRAMCNT(phy->id), 0x20, 0x20);
			mp_phy_reg_wait(priv, MPPHY_PXRXREQ1(phy->id), 0x2, 0);
		} else if (priv->num_lanes[phy->id] == 4) {
			mp_phy_reg_wait(priv, MPPHY_PXSRAMCNT(2 * phy->id), 0x20, 0x20);
			mp_phy_reg_wait(priv, MPPHY_PXSRAMCNT((2 * phy->id) + 1), 0x20, 0x20);
			mp_phy_reg_wait(priv, MPPHY_PXRXREQ1(2 * phy->id), 0x2, 0);
			mp_phy_reg_wait(priv, MPPHY_PXRXREQ1((2 * phy->id) + 1), 0x2, 0);
		} else if (priv->num_lanes[phy->id] == 8) {
			mp_phy_reg_wait(priv, MPPHY_PXSRAMCNT(0), 0x20, 0x20);
			mp_phy_reg_wait(priv, MPPHY_PXSRAMCNT(1), 0x20, 0x20);
			mp_phy_reg_wait(priv, MPPHY_PXSRAMCNT(2), 0x20, 0x20);
			mp_phy_reg_wait(priv, MPPHY_PXSRAMCNT(3), 0x20, 0x20);
			mp_phy_reg_wait(priv, MPPHY_PXRXREQ1(0), 0x2, 0);
			mp_phy_reg_wait(priv, MPPHY_PXRXREQ1(1), 0x2, 0);
			mp_phy_reg_wait(priv, MPPHY_PXRXREQ1(2), 0x2, 0);
			mp_phy_reg_wait(priv, MPPHY_PXRXREQ1(3), 0x2, 0);
		}
	} else if (chan->protocol_id == PHY_MODE_ETHERNET) {
		mp_phy_update_bits(priv, MPPHY_PXSRAMCNT(phy->id),
				   SRAM_EXT_LD_DONE, SRAM_EXT_LD_DONE);
		ret = mp_phy_reg_wait(priv, MPPHY_PXRXREQ1(phy->id), 0x2, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static bool mp_phy_proto_is_usb(enum phy_mode protocol)
{
	return protocol == PHY_MODE_USB_HOST ||
	       protocol == PHY_MODE_USB_DEVICE ||
	       protocol == PHY_MODE_USB_OTG;
}

static int mp_phy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct mp_phy_priv *priv = phy_get_drvdata(phy);
	struct mp_phy_chan_priv *chan = &priv->chan[phy->id];
	enum phy_mode new_protocol;

	switch (mode) {
	case PHY_MODE_PCIE:
	case PHY_MODE_ETHERNET:
	case PHY_MODE_USB_HOST:
	case PHY_MODE_USB_DEVICE:
	case PHY_MODE_USB_OTG:
		new_protocol = mode;
		break;
	default:
		dev_err(&phy->dev, "Unsupported PHY mode: %d\n", mode);
		return -EOPNOTSUPP;
	}

	if (chan->initialized && chan->current_protocol != new_protocol) {
		if (!mp_phy_proto_is_usb(new_protocol)) {
			dev_err(&phy->dev,
				"Protocol conflict on channel %d: current=%d, requested=%d\n",
				phy->id, chan->current_protocol, new_protocol);
			return -EINVAL;
		}
	}

	if (chan->initialized) {
		/* Skip if same protocol */
		if (chan->current_protocol == new_protocol)
			return 0;

		if (mp_phy_proto_is_usb(chan->current_protocol) &&
		    mp_phy_proto_is_usb(new_protocol)) {
			return 0;
		}
	}

	chan->current_protocol = new_protocol;
	chan->protocol_id = new_protocol;

	return 0;
}

static int mp_phy_config_usb(struct phy *phy, int speed)
{
	struct mp_phy_priv *priv = phy_get_drvdata(phy);
	struct mp_phy_chan_priv *chan = &priv->chan[phy->id];
	int ret;

	if (speed == HIGH_SPEED) {
		mp_phy_write(priv, TCA_VBUS_CTRL_OFFSET(chan->lane_id), 0x3E);
		return 0;
	}

	if (speed != SUPER_SPEED_PLUS)
		return 0;

	/* SUPER_SPEED_PLUS */
	ret = mp_phy_reg_wait(priv, PSTATE_1_OFFSET(chan->lane_id), 0x3, 0x3);
	if (ret)
		return ret;

	mp_phy_update_bits(priv, TCA_INTR_OFFSET(chan->lane_id), 0x3, 0x3);

	mp_phy_update_bits(priv, TCA_TCPC_OFFSET(chan->lane_id), 0x10, 0x10);
	ret = mp_phy_reg_wait(priv, TCA_INTR_STS_OFFSET(chan->lane_id), 0x1, 0x1);
	if (ret)
		return ret;

	mp_phy_update_bits(priv, TCA_INTR_STS_OFFSET(chan->lane_id), 0x1503, 0x1503);
	mp_phy_update_bits(priv, TCA_TCPC_OFFSET(chan->lane_id), 0x11, 0x11);
	ret = mp_phy_reg_wait(priv, TCA_INTR_STS_OFFSET(chan->lane_id), 0x1, 0x1);
	if (ret)
		return ret;

	mp_phy_update_bits(priv, TCA_INTR_STS_OFFSET(phy->id), 0x1503, 0x1503);

	return 0;
}

static const struct phy_ops mp_phy_ops = {
	.init		= mp_phy_init,
	.exit		= mp_phy_exit,
	.power_on	= mp_phy_power_on,
	.set_mode	= mp_phy_set_mode,
	.set_speed	= mp_phy_config_usb,
	.owner		= THIS_MODULE,
};

static struct phy *mp_phy_xlate(struct device *dev,
				const struct of_phandle_args *args)
{
	struct mp_phy_priv *priv = dev_get_drvdata(dev);
	struct mp_phy_chan_priv *chan;
	int id;

	if (args->args_count > 2) {
		dev_err(dev, "Invalid args_count: %d\n", args->args_count);
		return ERR_PTR(-EINVAL);
	}

	if (args->args_count >= 1)
		id = args->args[0];
	else
		id = 0;

	chan = &priv->chan[id];

	if (args->args_count >= 2)
		chan->lane_id = args->args[1];
	else
		chan->lane_id = 0;

	return priv->chan[id].phy;
}

static int mp_phy_parse_dt(struct platform_device *pdev, struct mp_phy_priv *priv)
{
	struct device *dev = &pdev->dev;
	bool need_fw = false;
	u32 ext_ref_clk[4];
	u32 out_ref_clk[4];
	u32 phy_type[8];
	int i, ret;

	ret = device_property_read_u32_array(dev, "renesas,phy-type", phy_type, 8);
	if (ret < 0)
		return dev_err_probe(dev, -EINVAL, "Failed to read PHY configuration\n");

	ret = device_property_read_u32_array(dev, "renesas,external-ref-clock",
					     ext_ref_clk, 4);
	if (ret < 0)
		return dev_err_probe(dev, -EINVAL, "Failed to read PHY clock configuration\n");

	ret = device_property_read_u32_array(dev, "renesas,output-repeat-ref-clock",
					     out_ref_clk, 4);
	if (ret < 0)
		return dev_err_probe(dev, -EINVAL, "Failed to read PHY clock out configuration\n");

	ret = device_property_read_u32_array(dev, "renesas,num-lanes", priv->num_lanes, 4);
	if (ret < 0)
		return dev_err_probe(dev, -EINVAL, "Failed to read PHY lane configuration\n");

	/* Port 0 can be either PCIe4 channel 0 lanes 0,1 or Ethernet 0-1 */
	if (!((phy_type[0] == PHY_TYPE_PCIE && phy_type[1] == 0) ||
	      (phy_type[0] == PHY_TYPE_XPCS && phy_type[1] == 0))) {
		return dev_err_probe(dev, -EINVAL, "Incorrect PHY port 0 configuration\n");
	}

	/* Port 1 can be either PCIe4 channel 0 lanes 2,3 or Ethernet 2-3 */
	if (!((phy_type[2] == PHY_TYPE_PCIE && phy_type[3] == 0) ||
	      (phy_type[2] == PHY_TYPE_XPCS && phy_type[3] == 1))) {
		return dev_err_probe(dev, -EINVAL, "Incorrect PHY port 1 configuration\n");
	}

	/*
	 * Port 2 can be either PCIe4 channel 0 lanes 4,5 or Ethernet 4-5 or
	 * PCIe4 channel 1 lanes 0,1 or USB3.2 channel 0
	 */
	if (!((phy_type[4] == PHY_TYPE_PCIE && phy_type[5] == 0) ||
	      (phy_type[4] == PHY_TYPE_PCIE && phy_type[5] == 1) ||
	      (phy_type[4] == PHY_TYPE_XPCS && phy_type[5] == 2) ||
	      (phy_type[4] == PHY_TYPE_USB3 && phy_type[5] == 0))) {
		return dev_err_probe(dev, -EINVAL, "Incorrect PHY port 2 configuration\n");
	}

	/*
	 * Port 3 can be either PCIe4 channel 0 lanes 6,7 or Ethernet 6-7 or
	 * PCIe4 channel 1 lanes 2,3 or USB3.2 channel 1
	 */
	if (!((phy_type[6] == PHY_TYPE_PCIE && phy_type[7] == 0) ||
	      (phy_type[6] == PHY_TYPE_PCIE && phy_type[7] == 1) ||
	      (phy_type[6] == PHY_TYPE_XPCS && phy_type[7] == 3) ||
	      (phy_type[6] == PHY_TYPE_USB3 && phy_type[7] == 1))) {
		return dev_err_probe(dev, -EINVAL, "Incorrect PHY port 3 configuration\n");
	}

	priv->cmncnt[0] = 0;
	priv->cmncnt[1] = 0x33330000;	/* All res_{ack,req}_in_sel are 1 */

	for (i = 0; i < MPPHY_NUM_CHANNELS; i++) {
		priv->chan[i].initialized = false;
		priv->chan[i].current_protocol = PHY_MODE_INVALID;
		priv->chan[i].protocol_id = PHY_MODE_INVALID;

		if (phy_type[2 * i] == PHY_TYPE_PCIE) {
			priv->sramcnt[i] = 0xf;
			if (phy_type[(2 * i) + 1] == 0)	/* Channel 0 */
				priv->cmncnt[0] |= 0x0 << (i * 8);
			else				/* Channel 1 */
				priv->cmncnt[0] |= 0x2 << (i * 8);
		} else if (phy_type[2 * i] == PHY_TYPE_XPCS) {
			priv->sramcnt[i] = 0x0;
			priv->write_cntxt1 |= BIT(i);
			priv->cmncnt[0] |= 0x1 << (i * 8);
			need_fw = true;
		} else if (phy_type[2 * i] == PHY_TYPE_USB3) {
			priv->sramcnt[i] = 0x9;
			priv->cmncnt[0] |= 0x3 << (i * 8);
			need_fw = true;
		} else {
			/* Cannot be reached. */
			return -EINVAL;
		}

		if (ext_ref_clk[i])
			priv->cmncnt[1] |= BIT(i * 4);

		if (out_ref_clk[i])
			priv->cmncnt[1] |= BIT((i * 4) + 1);
	}

	if (!need_fw)
		return 0;

	return request_firmware(&priv->fw, MPPHY_FW_NAME, dev);
}

static int mp_phy_probe(struct platform_device *pdev)
{
	static const char *const pd_names[] = { "mpp0", "mpp1", "mpp2", "mpp3" };
	const struct dev_pm_domain_attach_data pd_attach_data = {
		.pd_names = pd_names,
		.num_pd_names = ARRAY_SIZE(pd_names),
		.pd_flags = 0,
	};
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct mp_phy_priv *priv;
	int i, ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	ret = mp_phy_parse_dt(pdev, priv);
	if (ret)
		return ret;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return dev_err_probe(dev, PTR_ERR(priv->base), "Failed to map PHY registers\n");

	priv->num_clks = devm_clk_bulk_get_all(dev, &priv->clks);
	if (priv->num_clks < 0)
		return dev_err_probe(dev, priv->num_clks, "Failed to get PHY clocks\n");
	if (priv->num_clks != 5)
		return dev_err_probe(dev, -ENODEV, "Failed to get all PHY clocks\n");

	/*
	 * The reset ID order here does matters, reset_control_bulk_assert()
	 * asserts these resets in this order, with mpphy02 reset being
	 * asserted first, reset_control_bulk_deassert() deasserts these
	 * resets in reverse order, with mpphy02 being reset being
	 * deasserted last. This is the behavior the hardware expects.
	 */
	priv->resets[0].id = "mpphy02";
	priv->resets[1].id = "mpphy01";
	priv->resets[2].id = "mpphy11";
	priv->resets[3].id = "mpphy21";
	priv->resets[4].id = "mpphy31";
	ret = devm_reset_control_bulk_get_exclusive(&pdev->dev, ARRAY_SIZE(priv->resets),
						    priv->resets);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get PHY resets\n");

	platform_set_drvdata(pdev, priv);

	ret = dev_pm_domain_attach_list(dev, &pd_attach_data, &priv->pd_list);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to attach power domains\n");

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable PHY runtime PM\n");

	provider = devm_of_phy_provider_register(dev, mp_phy_xlate);
	if (IS_ERR(provider))
		return dev_err_probe(dev, PTR_ERR(provider), "Failed to register PHY provider\n");

	for (i = 0; i < MPPHY_NUM_CHANNELS; i++) {
		priv->chan[i].phy = devm_phy_create(dev, NULL, &mp_phy_ops);
		if (IS_ERR(priv->chan[i].phy)) {
			return dev_err_probe(dev, PTR_ERR(priv->chan[i].phy),
					     "Failed to create PHY %d\n", i);
		}

		priv->chan[i].phy->id = i;
		phy_set_drvdata(priv->chan[i].phy, priv);
	}

	return pm_runtime_resume_and_get(dev);
}

static void mp_phy_remove(struct platform_device *pdev)
{
	struct mp_phy_priv *priv = dev_get_drvdata(&pdev->dev);
	struct device *dev = &pdev->dev;

	pm_runtime_put(dev);

	dev_pm_domain_detach_list(priv->pd_list);

	pm_runtime_disable(&pdev->dev);

	if (priv->fw)
		release_firmware(priv->fw);

	platform_set_drvdata(pdev, NULL);
}

static int mp_phy_suspend(struct device *dev)
{
	struct mp_phy_priv *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < MPPHY_NUM_CHANNELS; i++)
		priv->chan[i].initialized = false;

	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);

	dev_info(dev, "Multi-Protocol PHY suspended\n");

	return 0;
}

static int mp_phy_resume(struct device *dev)
{
	struct mp_phy_priv *priv = dev_get_drvdata(dev);
	int i, ret;

	ret = reset_control_bulk_assert(ARRAY_SIZE(priv->resets), priv->resets);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to assert PHY resets\n");

	ret = reset_control_bulk_deassert(ARRAY_SIZE(priv->resets), priv->resets);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to deassert PHY resets\n");

	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable PHY clocks\n");

	/* Reload configuration */
	mp_phy_update_bits(priv, MPPHY_CMNCNT1, priv->cmncnt[0], priv->cmncnt[0]);
	mp_phy_write(priv, MPPHY_CMNCNT2, priv->cmncnt[1]);

	for (i = 0; i < MPPHY_NUM_CHANNELS; i++) {
		mp_phy_update_bits(priv, MPPHY_PXTEST(i),
				   MPPHY_PXTEST_BIT, MPPHY_PXTEST_BIT);
		mp_phy_write(priv, MPPHY_PXSRAMCNT(i), priv->sramcnt[i]);

		if (priv->write_cntxt1 & BIT(i)) {
			mp_phy_write(priv, MPPHY_CHAN_BASE(i) + 0x10c, 0xff0ff);
			mp_phy_write(priv, MPPHY_PXCNTXT1(i), 0x180023);
		}

		mp_phy_update_bits(priv, MPPHY_PXTEST(i), MPPHY_PXTEST_BIT, 0);
	}

	mp_phy_update_bits(priv, MPPHY_PCS0REG1, MPPHY_PCS0REG1_VAL, 0);
	mp_phy_update_bits(priv, MPPHY_PCS0REG5, 0xff000000, 0);

	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(mp_phy_pm_ops, mp_phy_suspend, mp_phy_resume, NULL);

static const struct of_device_id mp_phy_of_match[] = {
	{ .compatible = "renesas,rcar-gen5-mpphy" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mp_phy_of_match);

static struct platform_driver mp_phy_driver = {
	.probe	= mp_phy_probe,
	.remove	= mp_phy_remove,
	.driver	= {
		.name		= "renesas-mpphy",
		.of_match_table	= mp_phy_of_match,
		.pm		= pm_ptr(&mp_phy_pm_ops),
		.probe_type	= PROBE_PREFER_ASYNCHRONOUS,
	},
};

module_platform_driver(mp_phy_driver);

MODULE_AUTHOR("Thanh Quan");
MODULE_DESCRIPTION("Renesas Multi-Protocol PHY driver");
MODULE_LICENSE("GPL v2");
MODULE_FIRMWARE(MPPHY_FW_NAME);
