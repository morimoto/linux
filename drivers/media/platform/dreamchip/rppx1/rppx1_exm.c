// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define EXM_VERSION_REG			0x0000
#define EXM_START_REG			0x0004

#define EXM_CTRL_REG			0x0008
#define EXM_CTRL_EXM_AUTOSTOP		BIT(1) /* HW doc says not supported. */
#define EXM_CTRL_EXM_UPDATE_ENABLE	BIT(0)

#define EXM_MODE_REG			0x000c
#define EXM_CHANNEL_SEL_REG		0x0010
#define EXM_LAST_MEAS_LINE_REG		0x0014
#define EXM_COEFF_R_REG			0x0018
#define EXM_COEFF_G_GR_REG		0x001c
#define EXM_COEFF_B_REG			0x0020
#define EXM_COEFF_GB_REG		0x0024
#define EXM_H_OFFS_REG			0x0028
#define EXM_V_OFFS_REG			0x002c
#define EXM_H_SIZE_REG			0x0030
#define EXM_V_SIZE_REG			0x0034
#define EXM_FORCED_UPD_START_LINE_REG	0x0038
#define EXM_VSTART_STATUS_REG		0x003c

#define EXM_MEAN_REG_NUM		25
#define EXM_MEAN_REG(n)			(0x0040 + (4 * (n)))

static int rppx1_exm_probe(struct rpp_module *mod)
{
	/* Version check. */
	switch (rpp_module_read(mod, EXM_VERSION_REG)) {
	case 1:
		mod->info.exm.resultbits = 8;
		break;
	case 3:
		mod->info.exm.resultbits = 20;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int
rppx1_exm_param_rkisp1(struct rpp_module *mod,
		       const union rppx1_params_rkisp1_config *block,
		       rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_aec_config *cfg = &block->aec;
	const struct rkisp1_cif_isp_aec_config *arg = &cfg->config;
	u32 h_offs, v_offs, h_size, v_size;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + EXM_MODE_REG, 0);
		return 0;
	}

	/* RGB bayer exposure measurement */
	write(priv, mod->base + EXM_MODE_REG, 2);

	write(priv, mod->base + EXM_CTRL_REG, EXM_CTRL_EXM_UPDATE_ENABLE |
	      arg->autostop ? EXM_CTRL_EXM_AUTOSTOP : 0);

	/*
	 * Select where to sample.
	 * 0 - after input acquisition
	 * 1 - after black level subtraction
	 * 2 - after input linearization
	 * 3 - after lens shade correction
	 * 4 - after white balance gain stage
	 * 5 - after defect pixel correction
	 * 6 - after denoising
	 */
	write(priv, mod->base + EXM_CHANNEL_SEL_REG, 6);

	if (arg->mode == RKISP1_CIF_ISP_EXP_MEASURING_MODE_0) {
		/* Coefficients for a BT.601 BAYER (from datasheet). */
		write(priv, mod->base + EXM_COEFF_R_REG, 38);
		write(priv, mod->base + EXM_COEFF_G_GR_REG, 75);
		write(priv, mod->base + EXM_COEFF_B_REG, 15);
		write(priv, mod->base + EXM_COEFF_GB_REG, 75);
	} else {
		/* Y = (R + Gr + B + Gb) / 4*/
		write(priv, mod->base + EXM_COEFF_R_REG, 128);
		write(priv, mod->base + EXM_COEFF_G_GR_REG, 128);
		write(priv, mod->base + EXM_COEFF_B_REG, 128);
		write(priv, mod->base + EXM_COEFF_GB_REG, 128);
	}

	/*
	 * Adjust and set measurement window to hardware limitations,
	 * - Offsets must be even.
	 * - Width and height must be divisible by 10.
	 */
	h_offs = arg->meas_window.h_offs & 0x1ffe;
	v_offs = arg->meas_window.v_offs & 0x1ffe;
	h_size = (arg->meas_window.h_size - 1) - ((arg->meas_window.h_size - 1) % 10);
	v_size = (arg->meas_window.v_size - 1) - ((arg->meas_window.v_size - 1) % 10);

	write(priv, mod->base + EXM_H_OFFS_REG, h_offs);
	write(priv, mod->base + EXM_V_OFFS_REG, v_offs);
	write(priv, mod->base + EXM_H_SIZE_REG, h_size / 5);
	write(priv, mod->base + EXM_V_SIZE_REG, v_size / 5);

	/* Set last measurement line for ready interrupt. */
	write(priv, mod->base + EXM_LAST_MEAS_LINE_REG, v_offs + v_size + 1);

	write(priv, mod->base + EXM_START_REG, 1);

	return 0;
}

static int rppx1_exm_stats_rkisp1(struct rpp_module *mod,
				  struct rkisp1_cif_isp_stat *stats)
{
	u8 *meas = &stats->ae.exp_mean[0];
	/*
	 * The RkISP mean stats are 8-bit while the RPP can be 8 or 20 bit.
	 * Figure out how much we need to adjust the output parameters.
	 */
	const unsigned int shift = mod->info.exm.resultbits - 8;

	for (unsigned int i = 0; i < EXM_MEAN_REG_NUM; i++)
		meas[i] = rpp_module_read(mod, EXM_MEAN_REG(i)) >> shift;

	return 0;
}

const struct rpp_module_ops rppx1_exm_ops = {
	.probe = rppx1_exm_probe,
	.param_rkisp1 = rppx1_exm_param_rkisp1,
	.stats_rkisp1 = rppx1_exm_stats_rkisp1,
};
