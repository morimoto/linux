// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define CCOR_VERSION_REG				0x0000

#define CCOR_COEFF_REG_NUM				9
#define CCOR_COEFF_REG(n)				(0x0004 + (4 * (n)))

#define CCOR_OFFSET_R_REG				0x0028
#define CCOR_OFFSET_G_REG				0x002c
#define CCOR_OFFSET_B_REG				0x0030

#define CCOR_CONFIG_TYPE_REG				0x0034
#define CCOR_CONFIG_TYPE_USE_OFFSETS_AS_PRE_OFFSETS	BIT(1)
#define CCOR_CONFIG_TYPE_CCOR_RANGE_AVAILABLE		BIT(0)

#define CCOR_RANGE_REG					0x0038
#define CCOR_RANGE_CCOR_C_RANGE				BIT(1)
#define CCOR_RANGE_CCOR_Y_RANGE				BIT(0)

static int rppx1_ccor_probe(struct rpp_module *mod)
{
	/* Version check. */
	switch (rpp_module_read(mod, CCOR_VERSION_REG)) {
	case 3:
		mod->info.ccor.colorbits = 12;
		break;
	case 4:
		mod->info.ccor.colorbits = 20;
		break;
	case 5:
		mod->info.ccor.colorbits = 24;
		break;
	default:
		return -EINVAL;
	}

	mod->info.ccor.type = rpp_module_read(mod, CCOR_CONFIG_TYPE_REG);

	return 0;
}

static int rppx1_ccor_start(struct rpp_module *mod,
			    const struct v4l2_mbus_framefmt *fmt)
{
	/* Configure matrix in bypass mode. */
	rpp_module_write(mod, CCOR_COEFF_REG(0), 0x1000);
	rpp_module_write(mod, CCOR_COEFF_REG(1), 0x0000);
	rpp_module_write(mod, CCOR_COEFF_REG(2), 0x0000);

	rpp_module_write(mod, CCOR_COEFF_REG(3), 0x0000);
	rpp_module_write(mod, CCOR_COEFF_REG(4), 0x1000);
	rpp_module_write(mod, CCOR_COEFF_REG(5), 0x0000);

	rpp_module_write(mod, CCOR_COEFF_REG(6), 0x0000);
	rpp_module_write(mod, CCOR_COEFF_REG(7), 0x0000);
	rpp_module_write(mod, CCOR_COEFF_REG(8), 0x1000);

	rpp_module_write(mod, CCOR_OFFSET_R_REG, 0x00000000);
	rpp_module_write(mod, CCOR_OFFSET_G_REG, 0x00000000);
	rpp_module_write(mod, CCOR_OFFSET_B_REG, 0x00000000);

	return 0;
}

static int
rppx1_ccor_param_rkisp1(struct rpp_module *mod,
			const union rppx1_params_rkisp1_config *block,
			rppx1_reg_write write, void *priv)
{
	const struct rkisp1_ext_params_ctk_config *cfg = &block->ctk;

	/* If the modules is disabled, configure in bypass mode. */
	if (cfg->header.flags & RKISP1_EXT_PARAMS_FL_BLOCK_DISABLE) {
		write(priv, mod->base + CCOR_COEFF_REG(0), 0x1000);
		write(priv, mod->base + CCOR_COEFF_REG(1), 0x0000);
		write(priv, mod->base + CCOR_COEFF_REG(2), 0x0000);

		write(priv, mod->base + CCOR_COEFF_REG(3), 0x0000);
		write(priv, mod->base + CCOR_COEFF_REG(4), 0x1000);
		write(priv, mod->base + CCOR_COEFF_REG(5), 0x0000);

		write(priv, mod->base + CCOR_COEFF_REG(6), 0x0000);
		write(priv, mod->base + CCOR_COEFF_REG(7), 0x0000);
		write(priv, mod->base + CCOR_COEFF_REG(8), 0x1000);

		write(priv, mod->base + CCOR_OFFSET_R_REG, 0x00000000);
		write(priv, mod->base + CCOR_OFFSET_G_REG, 0x00000000);
		write(priv, mod->base + CCOR_OFFSET_B_REG, 0x00000000);

		return 0;
	}

	/*
	 * Coefficient n for color correction matrix.
	 *
	 * RkISP1 coefficients are 11-bit signed fixed-point numbers with 4 bit
	 * integer and 7 bit fractional part, ranging from -8 (0x400) to +7.992
	 * (0x3FF). 0 is represented by 0x000 and a coefficient value of 1 as
	 * 0x080.
	 *
	 * RPP gains are 16-bit signed fixed-point numbers with 4 bit integer
	 * and 12 bit fractional part ranging from -8 (0x8000) to +7.9996
	 * (0x7FFF). 0 is represented by 0x0000 and a coefficient value of 1 as
	 * 0x1000.
	 *
	 * Map the RkISP1 value range by left shifting by 5.
	 */
	write(priv, mod->base + CCOR_COEFF_REG(0), cfg->config.coeff[0][0] << 5);
	write(priv, mod->base + CCOR_COEFF_REG(1), cfg->config.coeff[0][1] << 5);
	write(priv, mod->base + CCOR_COEFF_REG(2), cfg->config.coeff[0][2] << 5);

	write(priv, mod->base + CCOR_COEFF_REG(3), cfg->config.coeff[1][0] << 5);
	write(priv, mod->base + CCOR_COEFF_REG(4), cfg->config.coeff[1][1] << 5);
	write(priv, mod->base + CCOR_COEFF_REG(5), cfg->config.coeff[1][2] << 5);

	write(priv, mod->base + CCOR_COEFF_REG(6), cfg->config.coeff[2][0] << 5);
	write(priv, mod->base + CCOR_COEFF_REG(7), cfg->config.coeff[2][1] << 5);
	write(priv, mod->base + CCOR_COEFF_REG(8), cfg->config.coeff[2][2] << 5);

	/*
	 * Offset for color components correction matrix.
	 *
	 * Values are a two's complement integer with one sign bit.
	 *
	 * The RkISP params are 11-bit while the RPP can be 12, 20 or 24 bit,
	 * all values are excluding the sign bit. Figure out how much we need
	 * to adjust the input parameters.
	 */
	const unsigned int shift = mod->info.wbmeas.colorbits - 12 + 1;

	write(priv, mod->base + CCOR_OFFSET_R_REG, cfg->config.ct_offset[0] << shift);
	write(priv, mod->base + CCOR_OFFSET_G_REG, cfg->config.ct_offset[1] << shift);
	write(priv, mod->base + CCOR_OFFSET_B_REG, cfg->config.ct_offset[2] << shift);

	return 0;
}

const struct rpp_module_ops rppx1_ccor_ops = {
	.probe = rppx1_ccor_probe,
	.start = rppx1_ccor_start,
	.param_rkisp1 = rppx1_ccor_param_rkisp1,
};

static int rppx1_ccor_csm_start(struct rpp_module *mod,
				const struct v4l2_mbus_framefmt *fmt)
{
	/* Reuse bypass matrix setup. */
	if (fmt->code == MEDIA_BUS_FMT_RGB888_1X24)
		return rppx1_ccor_start(mod, fmt);

	/* Color Transformation RGB to YUV according to ITU-R BT.709. */
	rpp_module_write(mod, CCOR_COEFF_REG(0), 0x0367);
	rpp_module_write(mod, CCOR_COEFF_REG(1), 0x0b71);
	rpp_module_write(mod, CCOR_COEFF_REG(2), 0x0128);

	rpp_module_write(mod, CCOR_COEFF_REG(3), 0xfe2b);
	rpp_module_write(mod, CCOR_COEFF_REG(4), 0xf9d5);
	rpp_module_write(mod, CCOR_COEFF_REG(5), 0x0800);

	rpp_module_write(mod, CCOR_COEFF_REG(6), 0x0800);
	rpp_module_write(mod, CCOR_COEFF_REG(7), 0xf8bc);
	rpp_module_write(mod, CCOR_COEFF_REG(8), 0xff44);

	rpp_module_write(mod, CCOR_OFFSET_R_REG, 0x00000000);
	rpp_module_write(mod, CCOR_OFFSET_G_REG, 0x00000800);
	rpp_module_write(mod, CCOR_OFFSET_B_REG, 0x00000800);

	return 0;
}

const struct rpp_module_ops rppx1_ccor_csm_ops = {
	.probe = rppx1_ccor_probe,
	.start = rppx1_ccor_csm_start,
};
