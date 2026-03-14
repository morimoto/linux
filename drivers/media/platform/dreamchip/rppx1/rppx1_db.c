// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define FILT_VERSION_REG		0x0000

#define DEMOSAIC_REG			0x0004
#define DEMOSAIC_DEMOSAIC_BYPASS	BIT(16)
#define DEMOSAIC_DEMOSAIC_TH_MASK	GENMASK(15, 0)

#define FILT_MODE_REG			0x0008
#define FILT_MODE_FILT_LP_SELECT_MASK	GENMASK(11, 8)
#define FILT_MODE_FILT_CHR_H_MODE_MASK	GENMASK(7, 6)
#define FILT_MODE_FILT_CHR_V_MODE_MASK	GENMASK(5, 4)
#define FILT_MODE_FILT_MODE		BIT(1)
#define FILT_MODE_FILT_ENABLE		BIT(0)

#define FILT_THRESH_BL0_REG		0x000c
#define FILT_THRESH_BL1_REG		0x0010
#define FILT_THRESH_SH0_REG		0x0014
#define FILT_THRESH_SH1_REG		0x0018
#define FILT_LUM_WEIGHT_REG		0x001c
#define FILT_FAC_SH1_REG		0x0020
#define FILT_FAC_SH0_REG		0x0024
#define FILT_FAC_MID_REG		0x0028
#define FILT_FAC_BL0_REG		0x002c
#define FILT_FAC_BL1_REG		0x0030

static int rppx1_db_probe(struct rpp_module *mod)
{
	/* Version check. */
	if (rpp_module_read(mod, FILT_VERSION_REG) != 5)
		return -EINVAL;

	return 0;
}

static int
rppx1_db_param_rkisp1_flt(struct rpp_module *mod,
			  const union rppx1_params_rkisp1_config *block,
			  rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_flt_config *cfg = &block->flt;
	u32 gain, kink, min;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + FILT_MODE_REG, 0);
		return 0;
	}

	/*
	 * RkISP1 values are 10-bit, RPP are 18-bit. Conversion verified with
	 * table in datasheet and libcamera pipeline for rkisp1.
	 */
	write(priv, mod->base + FILT_THRESH_BL0_REG, cfg->config.thresh_bl0 << 8);
	write(priv, mod->base + FILT_THRESH_BL0_REG, cfg->config.thresh_bl1 << 8);
	write(priv, mod->base + FILT_THRESH_SH0_REG, cfg->config.thresh_sh0 << 8);
	write(priv, mod->base + FILT_THRESH_SH1_REG, cfg->config.thresh_sh1 << 8);

	/*
	 * RkISP1 values are 6-bit, RPP are 8-bit. Conversion verified with
	 * table in datasheet and libcamera pipeline for rkisp1.
	 */
	write(priv, mod->base + FILT_FAC_BL0_REG, cfg->config.fac_bl0 << 2);
	write(priv, mod->base + FILT_FAC_BL1_REG, cfg->config.fac_bl1 << 2);
	write(priv, mod->base + FILT_FAC_MID_REG, cfg->config.fac_mid << 2);
	write(priv, mod->base + FILT_FAC_SH0_REG, cfg->config.fac_sh0 << 2);
	write(priv, mod->base + FILT_FAC_SH1_REG, cfg->config.fac_sh1 << 2);

	/*
	 * For unknown reasons the 3 fields of the FILT_LUM_WEIGHT register
	 * have been lumped together in a single field in the configuration
	 * data and written as is to the hardware. For RkISP1 the register
	 * layout is,
	 *
	 * 31:19	unused
	 * 18:16	lum_weight_gain
	 * 15:8		lum_weight_kink
	 *  7:0		lum_weight_min
	 *
	 * For RPP the register layout is similar but kink and gain have higher
	 * precision.
	 *
	 * 31		unused
	 * 30:28	lum_weight_gain
	 * 27:24	unused
	 * 23:12	lum_weight_kink
	 * 11:0		lum_weight_min
	 *
	 * Break apart the RkISP1 format, scale kink and min, and map to RPP.
	 */
	gain = (cfg->config.lum_weight & GENMASK(18, 16)) >> 16;
	kink = (cfg->config.lum_weight & GENMASK(15, 8)) >> 8;
	min = cfg->config.lum_weight & GENMASK(7, 0);

	write(priv, mod->base + FILT_LUM_WEIGHT_REG,
	      (gain << 28) | ((kink << 4) << 12) | (min << 4));

	write(priv, mod->base + FILT_MODE_REG,
	      (cfg->config.chr_v_mode << 4) |
	      (cfg->config.chr_h_mode << 6) |
	      (cfg->config.grn_stage1 << 8) |
	      (cfg->config.mode ? FILT_MODE_FILT_MODE : 0) |
	      FILT_MODE_FILT_ENABLE);

	return 0;
}

static int
rppx1_db_param_rkisp1_bdm(struct rpp_module *mod,
			  const union rppx1_params_rkisp1_config *block,
			  rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_bdm_config *cfg = &block->bdm;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + DEMOSAIC_REG, 0x400);
		return 0;
	}

	/*
	 * Threshold for Bayer demosaicing texture detection.
	 *
	 * RkISP1 threshold are 8-bit, RPP threshold are 16-bit. Map the RkISP1
	 * value range by left shifting by 8.
	 */
	write(priv, mod->base + DEMOSAIC_REG, cfg->config.demosaic_th << 8);

	return 0;
}

static int
rppx1_db_param_rkisp1(struct rpp_module *mod,
		      const union rppx1_params_rkisp1_config *block,
		      rppx1_reg_write write, void *priv)
{
	switch (block->header.type) {
	case RKISP1_EXT_PARAMS_BLOCK_TYPE_FLT:
		return rppx1_db_param_rkisp1_flt(mod, block, write, priv);
	case RKISP1_EXT_PARAMS_BLOCK_TYPE_BDM:
		return rppx1_db_param_rkisp1_bdm(mod, block, write, priv);
	}

	return -EINVAL;
}

const struct rpp_module_ops rppx1_db_ops = {
	.probe = rppx1_db_probe,
	.param_rkisp1 = rppx1_db_param_rkisp1,
};
