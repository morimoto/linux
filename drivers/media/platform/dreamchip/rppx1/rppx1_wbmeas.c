// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define AWB_MEAS_VERSION_REG			0x0000

#define AWB_MEAS_PROP_REG			0x0004
#define AWB_MEAS_PROP_MEAS_MODE_RGB		BIT(16) /* 0: YCbCr 1: RGB */
#define AWB_MEAS_PROP_YMAX			BIT(2)
#define AWB_MEAS_PROP_AWB_MODE_ON		BIT(1)

#define AWB_MEAS_H_OFFS_REG			0x0008
#define AWB_MEAS_V_OFFS_REG			0x000c
#define AWB_MEAS_H_SIZE_REG			0x0010
#define AWB_MEAS_V_SIZE_REG			0x0014
#define AWB_MEAS_FRAMES_REG			0x0018
#define AWB_MEAS_REF_CB_MAX_B_REG		0x001c
#define AWB_MEAS_REF_CR_MAX_R_REG		0x0020
#define AWB_MEAS_MAX_Y_REG			0x0024
#define AWB_MEAS_MIN_Y_MAX_G_REG		0x0028
#define AWB_MEAS_MAX_CSUM_REG			0x002c
#define AWB_MEAS_MIN_C_REG			0x0030
#define AWB_MEAS_WHITE_CNT_REG			0x0034
#define AWB_MEAS_MEAN_Y_G_REG			0x0038
#define AWB_MEAS_MEAN_CB_B_REG			0x003c
#define AWB_MEAS_MEAN_CR_R_REG			0x0040

#define AWB_MEAS_CCOR_COEFF_NUM			9
#define AWB_MEAS_CCOR_COEFF_REG(n)		(0x0044 + (4 * (n)))

#define AWB_MEAS_CCOR_OFFSET_R_REG		0x0068
#define AWB_MEAS_CCOR_OFFSET_G_REG		0x006c
#define AWB_MEAS_CCOR_OFFSET_B_REG		0x0070

static int rppx1_wbmeas_probe(struct rpp_module *mod)
{
	/* Version check. */
	switch (rpp_module_read(mod, AWB_MEAS_VERSION_REG)) {
	case 1:
		mod->info.wbmeas.colorbits = 8;
		break;
	case 2:
		mod->info.wbmeas.colorbits = 20;
		break;
	case 3:
		mod->info.wbmeas.colorbits = 24;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int
rppx1_wbmeas_param_rkisp1(struct rpp_module *mod,
			  const union rppx1_params_rkisp1_config *block,
			  rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_awb_meas_config *cfg = &block->awbm;
	/*
	 * The RkISP params are 8-bit while the RPP can be 8, 20 or 24 bit.
	 * Figure out how much we need to adjust the input parameters.
	 */
	const unsigned int shift = mod->info.wbmeas.colorbits - 8;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + AWB_MEAS_PROP_REG, 0);
		return 0;
	}

	/* Program measurement window. */
	write(priv, mod->base + AWB_MEAS_H_OFFS_REG,
	      cfg->config.awb_wnd.h_offs);
	write(priv, mod->base + AWB_MEAS_V_OFFS_REG,
	      cfg->config.awb_wnd.v_offs);
	write(priv, mod->base + AWB_MEAS_H_SIZE_REG,
	      cfg->config.awb_wnd.h_size);
	write(priv, mod->base + AWB_MEAS_V_SIZE_REG,
	      cfg->config.awb_wnd.v_size);

	/* Set number of frames to sample. */
	write(priv, mod->base + AWB_MEAS_FRAMES_REG, cfg->config.frames);

	if (cfg->config.awb_mode == RKISP1_CIF_ISP_AWB_MODE_YCBCR) {
		write(priv, mod->base + AWB_MEAS_REF_CB_MAX_B_REG,
		      cfg->config.awb_ref_cb << shift);
		write(priv, mod->base + AWB_MEAS_REF_CR_MAX_R_REG,
		      cfg->config.awb_ref_cr << shift);
		write(priv, mod->base + AWB_MEAS_MAX_Y_REG,
		      cfg->config.max_y << shift);
		write(priv, mod->base + AWB_MEAS_MIN_Y_MAX_G_REG,
		      cfg->config.min_y << shift);
		write(priv, mod->base + AWB_MEAS_MAX_CSUM_REG,
		      cfg->config.max_csum << shift);
		write(priv, mod->base + AWB_MEAS_MIN_C_REG,
		      cfg->config.min_c << shift);

		/*
		 * Match RkISP1 conversion, documented as
		 *  Y = 16 + 0.2500 R + 0.5000 G + 0.1094 B
		 *  Cb = 128 - 0.1406 R - 0.2969 G + 0.4375 B
		 *  Cr = 128 + 0.4375 R - 0.3750 G - 0.0625 B
		 *
		 * Note map Y to G. Matrix is GBR, not RGB documented for RPPX1.
		 */
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(0), 0x0800);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(1), 0x01c0);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(2), 0x0400);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(3), 0xfb40);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(4), 0x0700);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(5), 0xfdc0);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(6), 0xfa00);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(7), 0xff00);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(8), 0x0700);

		write(priv, mod->base + AWB_MEAS_CCOR_OFFSET_R_REG, 0x00100000);
		write(priv, mod->base + AWB_MEAS_CCOR_OFFSET_G_REG, 0x00800000);
		write(priv, mod->base + AWB_MEAS_CCOR_OFFSET_B_REG, 0x00800000);

		write(priv, mod->base + AWB_MEAS_PROP_REG,
		      cfg->config.enable_ymax_cmp ? AWB_MEAS_PROP_YMAX : 0 |
		      AWB_MEAS_PROP_AWB_MODE_ON);
	} else {
		/* The RkISP params are oddly named, but do map to RGB. */
		write(priv, mod->base + AWB_MEAS_REF_CB_MAX_B_REG,
		      cfg->config.awb_ref_cb << shift);
		write(priv, mod->base + AWB_MEAS_REF_CR_MAX_R_REG,
		      cfg->config.awb_ref_cr << shift);
		write(priv, mod->base + AWB_MEAS_MIN_Y_MAX_G_REG,
		      cfg->config.min_y << shift);

		/* Values from datasheet to map G to Y, B to Cb and R to Cr. */
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(0), 0x1000);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(1), 0x0000);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(2), 0x0000);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(3), 0x0000);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(4), 0x1000);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(5), 0x0000);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(6), 0x0000);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(7), 0x0000);
		write(priv, mod->base + AWB_MEAS_CCOR_COEFF_REG(8), 0x1000);

		/* Values from datasheet. */
		write(priv, mod->base + AWB_MEAS_CCOR_OFFSET_R_REG, 0x00000000);
		write(priv, mod->base + AWB_MEAS_CCOR_OFFSET_G_REG, 0x00000000);
		write(priv, mod->base + AWB_MEAS_CCOR_OFFSET_B_REG, 0x00000000);

		write(priv, mod->base + AWB_MEAS_PROP_REG,
		      AWB_MEAS_PROP_MEAS_MODE_RGB |
		      AWB_MEAS_PROP_AWB_MODE_ON);
	}

	return 0;
}

static int rppx1_wbmeas_stats_rkisp1(struct rpp_module *mod,
				     struct rkisp1_cif_isp_stat *stats)
{
	struct rkisp1_cif_isp_awb_meas *meas = &stats->awb.awb_mean[0];
	/*
	 * The RkISP YCbCr/RGB mean stats are 8-bit while the RPP can be 8, 20
	 * or 24 bit. Figure out how much we need to adjust the output
	 * statistics.
	 */
	const unsigned int shift = mod->info.wbmeas.colorbits - 8;

	meas->cnt = rpp_module_read(mod, AWB_MEAS_WHITE_CNT_REG);
	meas->mean_y_or_g =
		rpp_module_read(mod, AWB_MEAS_MEAN_Y_G_REG) >> shift;
	meas->mean_cb_or_b =
		rpp_module_read(mod, AWB_MEAS_MEAN_CB_B_REG) >> shift;
	meas->mean_cr_or_r =
		rpp_module_read(mod, AWB_MEAS_MEAN_CR_R_REG) >> shift;

	return 0;
}

const struct rpp_module_ops rppx1_wbmeas_ops = {
	.probe = rppx1_wbmeas_probe,
	.param_rkisp1 = rppx1_wbmeas_param_rkisp1,
	.stats_rkisp1 = rppx1_wbmeas_stats_rkisp1
};
