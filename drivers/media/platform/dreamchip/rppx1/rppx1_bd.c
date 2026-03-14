// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define DPF_VERSION_REG			0x0000

#define DPF_MODE_REG			0x0004
#define DPF_MODE_USE_NF_GAIN		BIT(9)
#define DPF_MODE_LSC_GAIN_COMP		BIT(8)
#define DPF_MODE_NLL_SEGMENTATION	BIT(6)
#define DPF_MODE_RB_FILTER_SIZE		BIT(5)
#define DPF_MODE_R_FILTER_OFF		BIT(4)
#define DPF_MODE_GR_FILTER_OFF		BIT(3)
#define DPF_MODE_GB_FILTER_OFF		BIT(2)
#define DPF_MODE_B_FILTER_OFF		BIT(1)
#define DPF_MODE_DPF_ENABLE		BIT(0)

#define DPF_STRENGTH_R_REG		0x0008
#define DPF_STRENGTH_G_REG		0x000c
#define DPF_STRENGTH_B_REG		0x0010
#define DPF_S_WEIGHT_G_1_4_REG		0x0014
#define DPF_S_WEIGHT_G_5_6_REG		0x0018
#define DPF_S_WEIGHT_RB_1_4_REG		0x001c
#define DPF_S_WEIGHT_RB_5_6_REG		0x0020

#define DPF_NLL_G_COEFF_REG_NUM		17
#define DPF_NLL_G_COEFF_REG(n)		(0x0024 + (4 * (n)))

#define DPF_NLL_RB_COEFF_REG_NUM	17
#define DPF_NLL_RB_COEFF_REG(n)		(0x0068 + (4 * (n)))

#define DPF_NF_GAIN_R_REG		0x00ac
#define DPF_NF_GAIN_GR_REG		0x00b0
#define DPF_NF_GAIN_GB_REG		0x00b4
#define DPF_NF_GAIN_B_REG		0x00b8

static int rppx1_bd_probe(struct rpp_module *mod)
{
	/* Version check. */
	if (rpp_module_read(mod, DPF_VERSION_REG) != 5)
		return -EINVAL;

	return 0;
}

static int
rppx1_bd_param_rkisp1_main(struct rpp_module *mod,
			   const union rppx1_params_rkisp1_config *block,
			   rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_dpf_config *cfg = &block->dpf;
	unsigned int isp_dpf_mode, spatial_coeff;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + DPF_MODE_REG, 0);
		return 0;
	}

	/*
	 * RkISP1 have an extra hardware flag AWB_GAIN_COMP which was removed
	 * in RPP DB module version 4 and later. If the bit is set the
	 * programmed gains will be processed, if it's not set a default value
	 * of 1 (0x100) will be used. From the RPP documentation for DB version
	 * 4 changelog.
	 *
	 *   Removed RPP_DPF_MODE::awb_gain_comp. Always use programmed
	 *   nf-gains for gain compensation.
	 *
	 * We can emulate this behavior if we keep track of when the RkISP1 do
	 * set the flag.
	 */
	bool awb_gain_comp = false;

	switch (cfg->config.gain.mode) {
	case RKISP1_CIF_ISP_DPF_GAIN_USAGE_NF_GAINS:
		awb_gain_comp = true;
		isp_dpf_mode = DPF_MODE_USE_NF_GAIN;
		break;
	case RKISP1_CIF_ISP_DPF_GAIN_USAGE_LSC_GAINS:
		isp_dpf_mode = DPF_MODE_LSC_GAIN_COMP;
		break;
	case RKISP1_CIF_ISP_DPF_GAIN_USAGE_NF_LSC_GAINS:
		awb_gain_comp = true;
		isp_dpf_mode = DPF_MODE_USE_NF_GAIN | DPF_MODE_LSC_GAIN_COMP;
		break;
	case RKISP1_CIF_ISP_DPF_GAIN_USAGE_AWB_GAINS:
		awb_gain_comp = true;
		isp_dpf_mode = 0;
		break;
	case RKISP1_CIF_ISP_DPF_GAIN_USAGE_AWB_LSC_GAINS:
		awb_gain_comp = true;
		isp_dpf_mode = DPF_MODE_LSC_GAIN_COMP;
		break;
	case RKISP1_CIF_ISP_DPF_GAIN_USAGE_DISABLED:
	default:
		isp_dpf_mode = 0;
		break;
	}

	/* NOTE: Hardware bit for scale_mode is inverted compared to RkISP1. */
	if (cfg->config.nll.scale_mode == RKISP1_CIF_ISP_NLL_SCALE_LINEAR)
		isp_dpf_mode |= DPF_MODE_NLL_SEGMENTATION;
	if (cfg->config.rb_flt.fltsize == RKISP1_CIF_ISP_DPF_RB_FILTERSIZE_9x9)
		isp_dpf_mode |= DPF_MODE_RB_FILTER_SIZE;
	if (!cfg->config.rb_flt.r_enable)
		isp_dpf_mode |= DPF_MODE_R_FILTER_OFF;
	if (!cfg->config.rb_flt.b_enable)
		isp_dpf_mode |= DPF_MODE_B_FILTER_OFF;
	if (!cfg->config.g_flt.gb_enable)
		isp_dpf_mode |= DPF_MODE_GB_FILTER_OFF;
	if (!cfg->config.g_flt.gr_enable)
		isp_dpf_mode |= DPF_MODE_GR_FILTER_OFF;

	isp_dpf_mode |= DPF_MODE_DPF_ENABLE;

	if (awb_gain_comp) {
		write(priv, mod->base + DPF_NF_GAIN_B_REG, cfg->config.gain.nf_b_gain);
		write(priv, mod->base + DPF_NF_GAIN_R_REG, cfg->config.gain.nf_r_gain);
		write(priv, mod->base + DPF_NF_GAIN_GB_REG, cfg->config.gain.nf_gb_gain);
		write(priv, mod->base + DPF_NF_GAIN_GR_REG, cfg->config.gain.nf_gr_gain);
	} else {
		write(priv, mod->base + DPF_NF_GAIN_B_REG, 0x100);
		write(priv, mod->base + DPF_NF_GAIN_R_REG, 0x100);
		write(priv, mod->base + DPF_NF_GAIN_GB_REG, 0x100);
		write(priv, mod->base + DPF_NF_GAIN_GR_REG, 0x100);
	}

	/* The RkISP1 hardware have a single register for all components. */
	for (unsigned int i = 0; i < RKISP1_CIF_ISP_DPF_MAX_NLF_COEFFS; i++) {
		write(priv, mod->base + DPF_NLL_G_COEFF_REG(i), cfg->config.nll.coeff[i]);
		write(priv, mod->base + DPF_NLL_RB_COEFF_REG(i), cfg->config.nll.coeff[i]);
	}

	spatial_coeff = cfg->config.g_flt.spatial_coeff[0] |
			(cfg->config.g_flt.spatial_coeff[1] << 8) |
			(cfg->config.g_flt.spatial_coeff[2] << 16) |
			(cfg->config.g_flt.spatial_coeff[3] << 24);
	write(priv, mod->base + DPF_S_WEIGHT_G_1_4_REG, spatial_coeff);

	spatial_coeff = cfg->config.g_flt.spatial_coeff[4] |
			(cfg->config.g_flt.spatial_coeff[5] << 8);
	write(priv, mod->base + DPF_S_WEIGHT_G_5_6_REG, spatial_coeff);

	spatial_coeff = cfg->config.rb_flt.spatial_coeff[0] |
			(cfg->config.rb_flt.spatial_coeff[1] << 8) |
			(cfg->config.rb_flt.spatial_coeff[2] << 16) |
			(cfg->config.rb_flt.spatial_coeff[3] << 24);
	write(priv, mod->base + DPF_S_WEIGHT_RB_1_4_REG, spatial_coeff);

	spatial_coeff = cfg->config.rb_flt.spatial_coeff[4] |
			(cfg->config.rb_flt.spatial_coeff[5] << 8);
	write(priv, mod->base + DPF_S_WEIGHT_RB_5_6_REG, spatial_coeff);

	/*
	 * Bilateral Denoising does not react on RPP_HDR_UPD::regs_gen_cfg_upd
	 * (see Table 25). A change in configuration needs write of 1 to
	 * RPP_HDR_UPD::regs_cfg_upd.
	 */
	write(priv, 4, 1);

	write(priv, mod->base + DPF_MODE_REG, isp_dpf_mode);

	return 0;
}

static int
rppx1_bd_param_rkisp1_strength(struct rpp_module *mod,
			       const union rppx1_params_rkisp1_config *block,
			       rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_dpf_strength_config *cfg = &block->dpfs;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + DPF_MODE_REG, 0);
		return 0;
	}

	/* Module version 5 adds shadowing for mode and spatial weights. */
	write(priv, mod->base + DPF_STRENGTH_R_REG, cfg->config.r);
	write(priv, mod->base + DPF_STRENGTH_G_REG, cfg->config.g);
	write(priv, mod->base + DPF_STRENGTH_B_REG, cfg->config.b);

	return 0;
}

static int
rppx1_bd_param_rkisp1(struct rpp_module *mod,
		      const union rppx1_params_rkisp1_config *block,
		      rppx1_reg_write write, void *priv)
{
	switch (block->header.type) {
	case RKISP1_EXT_PARAMS_BLOCK_TYPE_DPF:
		return rppx1_bd_param_rkisp1_main(mod, block, write, priv);
	case RKISP1_EXT_PARAMS_BLOCK_TYPE_DPF_STRENGTH:
		return rppx1_bd_param_rkisp1_strength(mod, block, write, priv);
	}

	return -EINVAL;
}

const struct rpp_module_ops rppx1_bd_ops = {
	.probe = rppx1_bd_probe,
	.param_rkisp1 = rppx1_bd_param_rkisp1,
};
