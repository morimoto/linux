// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"
#include "rppx1.h"

#define BLS_VERSION_REG				0x0000

#define BLS_CTRL_REG				0x0004
#define BLS_CTRL_BLS_WIN2			BIT(3)
#define BLS_CTRL_BLS_WIN1			BIT(2)
#define BLS_CTRL_BLS_MODE_MEASURED		BIT(1)
#define BLS_CTRL_BLS_EN				BIT(0)

#define BLS_SAMPLES_REG				0x0008
#define BLS_H1_START_REG			0x000c
#define BLS_H1_STOP_REG				0x0010
#define BLS_V1_START_REG			0x0014
#define BLS_V1_STOP_REG				0x0018
#define BLS_H2_START_REG			0x001c
#define BLS_H2_STOP_REG				0x0020
#define BLS_V2_START_REG			0x0024
#define BLS_V2_STOP_REG				0x0028
#define BLS_A_FIXED_REG				0x002c
#define BLS_B_FIXED_REG				0x0030
#define BLS_C_FIXED_REG				0x0034
#define BLS_D_FIXED_REG				0x0038
#define BLS_A_MEASURED_REG			0x003c
#define BLS_B_MEASURED_REG			0x0040
#define BLS_C_MEASURED_REG			0x0044
#define BLS_D_MEASURED_REG			0x0048

static int rppx1_bls_probe(struct rpp_module *mod)
{
	/* Version check. */
	switch (rpp_module_read(mod, BLS_VERSION_REG)) {
	case 3:
	case 5:
		mod->info.bls.colorbits = 12;
		break;
	case 2:
	case 4:
		mod->info.bls.colorbits = 20;
		break;
	case 6:
		mod->info.bls.colorbits = 24;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static void
rppx1_bls_swap_regs(struct rpp_module *mod, const u32 input[4], u32 output[4])
{
	static const unsigned int swap[4][4] = {
		[RPP_RGGB] = { 0, 1, 2, 3 },
		[RPP_GRBG] = { 1, 0, 3, 2 },
		[RPP_GBRG] = { 2, 3, 0, 1 },
		[RPP_BGGR] = { 3, 2, 1, 0 },
	};

	/* Swap to pattern used in our path, PRE1 or PRE2. */
	struct rpp_module *acq = mod == &mod->rpp->pre1.bls ?
		&mod->rpp->pre1.acq : &mod->rpp->pre2.bls;
	enum rpp_raw_pattern pattern = acq->info.acq.raw_pattern;

	for (unsigned int i = 0; i < 4; ++i)
		output[i] = input[swap[pattern][i]];
}

static int
rppx1_bls_param_rkisp1(struct rpp_module *mod,
		       const union rppx1_params_rkisp1_config *block,
		       rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_bls_config *cfg = &block->bls;

	/* If the modules is disabled, simply bypass it. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + BLS_CTRL_REG, 0);
		return 0;
	}

	u32 ctrl = BLS_CTRL_BLS_EN;

	if (!cfg->config.enable_auto) {
		static const u32 regs[] = {
			BLS_A_FIXED_REG,
			BLS_B_FIXED_REG,
			BLS_C_FIXED_REG,
			BLS_D_FIXED_REG,
		};
		u32 swapped[4];

		rppx1_bls_swap_regs(mod, regs, swapped);

		/*
		 * The RkISP params are 12-bit + 1 signed bit, while the RPP can
		 * be 12, 20 or 24 bit + 1 signed bit. Figure out how much we
		 * need to adjust the input parameters.
		 */
		const unsigned int shift = mod->info.bls.colorbits - 12;

		write(priv, mod->base + swapped[0], cfg->config.fixed_val.r << shift);
		write(priv, mod->base + swapped[1], cfg->config.fixed_val.gr << shift);
		write(priv, mod->base + swapped[2], cfg->config.fixed_val.gb << shift);
		write(priv, mod->base + swapped[3], cfg->config.fixed_val.b << shift);
	} else {
		write(priv, mod->base + BLS_SAMPLES_REG, cfg->config.bls_samples);

		if (cfg->config.en_windows & BIT(0)) {
			write(priv, mod->base + BLS_H1_START_REG, cfg->config.bls_window1.h_offs);
			write(priv, mod->base + BLS_H1_STOP_REG, cfg->config.bls_window1.h_size);
			write(priv, mod->base + BLS_V1_START_REG, cfg->config.bls_window1.v_offs);
			write(priv, mod->base + BLS_V1_STOP_REG, cfg->config.bls_window1.v_size);
			ctrl |= BLS_CTRL_BLS_WIN1;
		}

		if (cfg->config.en_windows & BIT(1)) {
			write(priv, mod->base + BLS_H2_START_REG, cfg->config.bls_window2.h_offs);
			write(priv, mod->base + BLS_H2_STOP_REG, cfg->config.bls_window2.h_size);
			write(priv, mod->base + BLS_V2_START_REG, cfg->config.bls_window2.v_offs);
			write(priv, mod->base + BLS_V2_STOP_REG, cfg->config.bls_window2.v_size);
			ctrl |= BLS_CTRL_BLS_WIN2;
		}

		ctrl |= BLS_CTRL_BLS_MODE_MEASURED;
	}

	write(priv, mod->base + BLS_CTRL_REG, ctrl);

	return 0;
}

static int rppx1_bls_stats_rkisp1(struct rpp_module *mod,
				  struct rkisp1_cif_isp_stat *stats)
{
	struct rkisp1_cif_isp_bls_meas_val *bls = &stats->ae.bls_val;

	static const u32 regs[] = {
		BLS_A_MEASURED_REG,
		BLS_B_MEASURED_REG,
		BLS_C_MEASURED_REG,
		BLS_D_MEASURED_REG,
	};
	u32 swapped[4];

	rppx1_bls_swap_regs(mod, regs, swapped);

	/*
	 * The RkISP BLS stats are 12-bit while the RPP can be 8, 20
	 * or 24 bit. Figure out how much we need to adjust the output
	 * statistics.
	 */
	const unsigned int shift = mod->info.bls.colorbits - 12;

	bls->meas_r = rpp_module_read(mod, swapped[0]) >> shift;
	bls->meas_gr = rpp_module_read(mod, swapped[1]) >> shift;
	bls->meas_gb = rpp_module_read(mod, swapped[2]) >> shift;
	bls->meas_b = rpp_module_read(mod, swapped[3]) >> shift;

	return 0;
}

const struct rpp_module_ops rppx1_bls_ops = {
	.probe = rppx1_bls_probe,
	.param_rkisp1 = rppx1_bls_param_rkisp1,
	.stats_rkisp1 = rppx1_bls_stats_rkisp1
};
