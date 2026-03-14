// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define HIST_VERSION_REG			0x0000

#define HIST_CTRL_REG				0x0004
#define HIST_CTRL_HIST_UPDATE_ENABLE		BIT(0)

#define HIST_MODE_REG				0x0008
#define HIST_MODE_HIST_MODE_MASK		GENMASK(2, 0)
#define HIST_MODE_HIST_MODE_DISABLE		0
#define HIST_MODE_HIST_MODE_YRGB		1
#define HIST_MODE_HIST_MODE_R			2
#define HIST_MODE_HIST_MODE_GR			3
#define HIST_MODE_HIST_MODE_B			4
#define HIST_MODE_HIST_MODE_GB			5

#define HIST_CHANNEL_SEL_REG			0x000c
#define HIST_CHANNEL_SEL_CHANNEL_SELECT_MASK	GENMASK(2, 0)

#define HIST_LAST_MEAS_LINE_REG			0x0010
#define HIST_SUBSAMPLING_REG			0x0014
#define HIST_SUBSAMPLING_V_STEPSIZE(x)		(((x) & 0x7f) << 24)
#define HIST_SUBSAMPLING_H_STEP_INC(x)		(((x) & 0x1ffff))

#define HIST_COEFF_R_REG			0x0018
#define HIST_COEFF_G_REG			0x001c
#define HIST_COEFF_B_REG			0x0020
#define HIST_H_OFFS_REG				0x0024
#define HIST_V_OFFS_REG				0x0028
#define HIST_H_SIZE_REG				0x002c
#define HIST_V_SIZE_REG				0x0030

#define HIST_SAMPLE_RANGE_REG			0x0034
#define HIST_SAMPLE_RANGE_SAMPLE_SHIFT_MASK	GENMASK(28, 24)
#define HIST_SAMPLE_RANGE_SAMPLE_OFFSET_MASK	GENMASK(23, 0)

#define HIST_WEIGHT_00TO30_REG			0x0038
#define HIST_WEIGHT_40TO21_REG			0x003c
#define HIST_WEIGHT_31TO12_REG			0x0040
#define HIST_WEIGHT_22TO03_REG			0x0044
#define HIST_WEIGHT_13TO43_REG			0x0048
#define HIST_WEIGHT_04TO34_REG			0x004c
#define HIST_WEIGHT_44_REG			0x0050
#define HIST_FORCED_UPD_START_LINE_REG		0x0054
#define HIST_FORCED_UPDATE_REG			0x0058
#define HIST_VSTART_STATUS_REG			0x005c

#define HIST_BIN_REG_NUM			32
#define HIST_BIN_REG(n)				(0x0060 + (4 * (n)))

static int rppx1_hist_probe(struct rpp_module *mod)
{
	/* Version check. */
	switch (rpp_module_read(mod, HIST_VERSION_REG)) {
	case 3:
		mod->info.hist.colorbits = 12;
		break;
	case 4:
		mod->info.hist.colorbits = 20;
		break;
	case 5:
		mod->info.hist.colorbits = 24;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define RPPX1_HIST_WEIGHT(v0, v1, v2, v3) \
	(((v0) & 0x1f) | (((v1) & 0x1f) << 8)  | \
	(((v2) & 0x1f) << 16) | \
	(((v3) & 0x1f) << 24))

static int rppx1_hist_param_rkisp1(struct rpp_module *mod,
				   const union rppx1_params_rkisp1_config *block,
				   rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_hst_config *cfg = &block->hst;
	const struct rkisp1_cif_isp_hst_config *arg = &cfg->config;
	u32 h_offs, v_offs, h_size, v_size;
	u8 mode, coeff[3];

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + HIST_MODE_REG,
		      HIST_MODE_HIST_MODE_DISABLE);
		return 0;
	}

	/* Sample after demosaicing. */
	write(priv, mod->base + HIST_CHANNEL_SEL_REG, 7);

	/*
	 * The RkISP1 histogram_predivider setting controls the pixel spacing
	 * between each sample. On RPPX1 there is greater control as both line
	 * and pixel spacing can be controlled.  The RkISP1 stepsize register is
	 * documented as.
	 *
	 *  0, 1, 3: not allowed
	 *  3: process every third input pixel
	 *  4: process every fourth input pixel
	 *  127: process every 127th pixel
	 *
	 * The output bins are 16 bit (FP16.4) so to not overflow a divider
	 * calculated as would be needed.
	 *
	 *  count = mode == RGB_COMBINED ? 3 : 1
	 *  factor = vsize * hsize  * count / 65536
	 *
	 * However the libcamera user of the RkISP documents the setting as
	 * applying to both h and v direction at the same time and calculates
	 * the divider as,
	 *
	 *  count = mode == RGB_COMBINED ? 3 : 1
	 *  factor = ceil(sqrt(vsize * hsize  * count / 65536))
	 *
	 * Real world usage is better then bad documentation, do the same here
	 * and apply the divider in both directions.
	 *
	 * The RPPX1 h-stepping is also configured differently. Internally
	 * there is a 16-bit counter and for each input pixel h_step_inc is
	 * added to it. Every time it overflows the input pixel is sampled.
	 *
	 *  h_step_inc = 2**16 => sample every pixel
	 *  h_step_inc = 2**15 => sample every other pixel
	 *
	 * Gives us the conversion to RkISP1 parameters of.
	 *
	 *  h_step_inc = 65536 / divider
	 */
	write(priv, mod->base + HIST_SUBSAMPLING_REG,
	      HIST_SUBSAMPLING_V_STEPSIZE(arg->histogram_predivider) |
	      HIST_SUBSAMPLING_H_STEP_INC(0x10000 / arg->histogram_predivider));

	/*
	 * Adjust and set measurement window to hardware limitations,
	 * - Offsets must be even.
	 * - Width and height must be divisible by 10.
	 */
	h_offs = arg->meas_window.h_offs & 0x1ffe;
	v_offs = arg->meas_window.v_offs & 0x1ffe;
	h_size = arg->meas_window.h_size - arg->meas_window.h_size % 10;
	v_size = arg->meas_window.v_size - arg->meas_window.v_size % 10;

	write(priv, mod->base + HIST_H_OFFS_REG, h_offs);
	write(priv, mod->base + HIST_V_OFFS_REG, v_offs);
	write(priv, mod->base + HIST_H_SIZE_REG, h_size / 5);
	write(priv, mod->base + HIST_V_SIZE_REG, v_size / 5);

	/* Set last measurement line for ready interrupt. */
	write(priv, mod->base + HIST_LAST_MEAS_LINE_REG,
	      v_offs + v_size + 1);

	/* NOTE: Keep the default full sample range. */

	/* Set measurement window weights. */
	write(priv, mod->base + HIST_WEIGHT_00TO30_REG,
	      RPPX1_HIST_WEIGHT(arg->hist_weight[0], arg->hist_weight[1],
				arg->hist_weight[2], arg->hist_weight[3]));
	write(priv, mod->base + HIST_WEIGHT_40TO21_REG,
	      RPPX1_HIST_WEIGHT(arg->hist_weight[4], arg->hist_weight[5],
				arg->hist_weight[6], arg->hist_weight[7]));
	write(priv, mod->base + HIST_WEIGHT_31TO12_REG,
	      RPPX1_HIST_WEIGHT(arg->hist_weight[8], arg->hist_weight[9],
				arg->hist_weight[10], arg->hist_weight[11]));
	write(priv, mod->base + HIST_WEIGHT_22TO03_REG,
	      RPPX1_HIST_WEIGHT(arg->hist_weight[12], arg->hist_weight[13],
				arg->hist_weight[14], arg->hist_weight[15]));
	write(priv, mod->base + HIST_WEIGHT_13TO43_REG,
	      RPPX1_HIST_WEIGHT(arg->hist_weight[16], arg->hist_weight[17],
				arg->hist_weight[18], arg->hist_weight[19]));
	write(priv, mod->base + HIST_WEIGHT_04TO34_REG,
	      RPPX1_HIST_WEIGHT(arg->hist_weight[20], arg->hist_weight[21],
				arg->hist_weight[22], arg->hist_weight[23]));
	write(priv, mod->base + HIST_WEIGHT_44_REG,
	      RPPX1_HIST_WEIGHT(arg->hist_weight[24], 0, 0, 0));

	/* Translate RkISP1 modes. */
	mode = HIST_MODE_HIST_MODE_YRGB;
	switch (arg->mode) {
	case RKISP1_CIF_ISP_HISTOGRAM_MODE_RGB_COMBINED:
		/* L = R + G + B */
		coeff[0] = 0x80;
		coeff[1] = 0x80;
		coeff[2] = 0x80;
		break;
	case RKISP1_CIF_ISP_HISTOGRAM_MODE_R_HISTOGRAM:
		/* L = R */
		coeff[0] = 0x80;
		coeff[1] = 0x00;
		coeff[2] = 0x00;
		break;
	case RKISP1_CIF_ISP_HISTOGRAM_MODE_G_HISTOGRAM:
		/* L = G */
		coeff[0] = 0x00;
		coeff[1] = 0x80;
		coeff[2] = 0x00;
		break;
	case RKISP1_CIF_ISP_HISTOGRAM_MODE_B_HISTOGRAM:
		coeff[0] = 0x00;
		coeff[1] = 0x00;
		coeff[2] = 0x80;
		break;
	case RKISP1_CIF_ISP_HISTOGRAM_MODE_Y_HISTOGRAM:
		/* Coefficients for a BT.601 (from datasheet). */
		coeff[0] = 38;
		coeff[1] = 75;
		coeff[2] = 15;
		break;
	default:
		mode = HIST_MODE_HIST_MODE_DISABLE;
		coeff[0] = 0x00;
		coeff[1] = 0x00;
		coeff[2] = 0x00;
		break;
	}

	write(priv, mod->base + HIST_MODE_REG, mode);
	write(priv, mod->base + HIST_COEFF_R_REG, coeff[0]);
	write(priv, mod->base + HIST_COEFF_G_REG, coeff[1]);
	write(priv, mod->base + HIST_COEFF_B_REG, coeff[2]);

	write(priv, mod->base + HIST_FORCED_UPDATE_REG, 1);

	return 0;
}

static int rppx1_hist_stats_rkisp1(struct rpp_module *mod,
				   struct rkisp1_cif_isp_stat *stats)
{
	for (unsigned int i = 0; i < HIST_BIN_REG_NUM; i++)
		stats->hist.hist_bins[i] = rpp_module_read(mod, HIST_BIN_REG(i)) & 0xfffff;

	return 0;
}

const struct rpp_module_ops rppx1_hist_ops = {
	.probe = rppx1_hist_probe,
	.param_rkisp1 = rppx1_hist_param_rkisp1,
	.stats_rkisp1 = rppx1_hist_stats_rkisp1,
};
