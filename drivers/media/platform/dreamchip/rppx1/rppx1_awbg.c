// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define AWB_GAIN_VERSION_REG		0x0000

#define AWB_ENABLE_REG			0x0004
#define AWB_ENABLE_AWB_GAIN_EN		BIT(0)

#define AWB_GAIN_GR_REG			0x0008
#define AWB_GAIN_GB_REG			0x000c
#define AWB_GAIN_R_REG			0x0010
#define AWB_GAIN_B_REG			0x0014

static int rppx1_awbg_probe(struct rpp_module *mod)
{
	/* Version check. */
	if (rpp_module_read(mod, AWB_GAIN_VERSION_REG) != 3)
		return -EINVAL;

	return 0;
}

static int
rppx1_awbg_param_rkisp1(struct rpp_module *mod,
			const union rppx1_params_rkisp1_config *block,
			rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_awb_gain_config *cfg = &block->awbg;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + AWB_ENABLE_REG, 0);
		return 0;
	}

	/*
	 * RkISP1 gains are 10-bit with 8 bit fractional part and 0x100 = 1.0,
	 * giving a possible range of 0.0 to 4.0.
	 *
	 * RPP gains are 18-bit with 12 bit fractional part and 0x1000 = 1.0,
	 * giving a possible range of 0.0 to 64.0. NOTE: RPP documentation is
	 * contradictory this is the register definition, the function
	 * description states 0x400 = 1.0 AND 18-bit with 12 fractional bits,
	 * which is not possible...
	 *
	 * Map the RkISP1 value range (0.0 - 4.0) by left shifting by 4.
	 */

	write(priv, mod->base + AWB_GAIN_GR_REG, cfg->config.gain_green_r << 4);
	write(priv, mod->base + AWB_GAIN_GB_REG, cfg->config.gain_green_b << 4);
	write(priv, mod->base + AWB_GAIN_R_REG, cfg->config.gain_red << 4);
	write(priv, mod->base + AWB_GAIN_B_REG, cfg->config.gain_blue << 4);

	write(priv, mod->base + AWB_ENABLE_REG, AWB_ENABLE_AWB_GAIN_EN);

	return 0;
}

const struct rpp_module_ops rppx1_awbg_ops = {
	.probe = rppx1_awbg_probe,
	.param_rkisp1 = rppx1_awbg_param_rkisp1,
};
