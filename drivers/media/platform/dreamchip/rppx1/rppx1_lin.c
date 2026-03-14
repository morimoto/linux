// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

/* NOTE: The module is called LIN the registers GAMMA_IN. */
#define LIN_VERSION_REG				0x0000

#define LIN_ENABLE_REG				0x0004
#define LIN_ENABLE_GAMMA_IN_EN			BIT(0)

#define LIN_DX_LO_REG				0x0008
#define LIN_DX_HI_REG				0x000c

#define LIN_R_Y_REG_NUM				17
#define LIN_R_Y_REG(n)				(0x0010 + (4 * (n)))

#define LIN_G_Y_REG_NUM				17
#define LIN_G_Y_REG(n)				(0x0054 + (4 * (n)))

#define LIN_B_Y_REG_NUM				17
#define LIN_B_Y_REG(n)				(0x0098 + (4 * (n)))

#define LIN_SAMPLES_NUM	17

static int rppx1_lin_probe(struct rpp_module *mod)
{
	/* Version check. */
	switch (rpp_module_read(mod, LIN_VERSION_REG)) {
	case 7:
		mod->info.lin.colorbits = 12;
		break;
	case 8:
		mod->info.lin.colorbits = 20;
		break;
	case 9:
		mod->info.lin.colorbits = 24;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rppx1_lin_start(struct rpp_module *mod,
			   const struct v4l2_mbus_framefmt *fmt)
{
	rpp_module_clrset(mod, LIN_ENABLE_REG, LIN_ENABLE_GAMMA_IN_EN, 0);

	return 0;
}

const struct rpp_module_ops rppx1_lin_ops = {
	.probe = rppx1_lin_probe,
	.start = rppx1_lin_start,
};
