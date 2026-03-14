// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rpp_module.h"

#define GAMMA_OUT_VERSION_REG			0x0000

#define GAMMA_OUT_ENABLE_REG			0x0004
#define GAMMA_OUT_ENABLE_GAMMA_OUT_EN		BIT(0)

#define GAMMA_OUT_MODE_REG			0x0008
#define GAMMA_OUT_MODE_GAMMA_OUT_EQU_SEGM	BIT(0)

#define GAMMA_OUT_Y_REG_NUM			17
#define GAMMA_OUT_Y_REG(n)			(0x000c + (4 * (n)))

static int rppx1_ga_probe(struct rpp_module *mod)
{
	/* Version check. */
	switch (rpp_module_read(mod, GAMMA_OUT_VERSION_REG)) {
	case 1:
		mod->info.ga.colorbits = 12;
		break;
	case 2:
		mod->info.ga.colorbits = 24;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int rppx1_ga_start(struct rpp_module *mod,
			  const struct v4l2_mbus_framefmt *fmt)
{
	/* Disable stage. */
	rpp_module_write(mod, GAMMA_OUT_ENABLE_REG, 0);

	return 0;
}

const struct rpp_module_ops rppx1_ga_ops = {
	.probe = rppx1_ga_probe,
	.start = rppx1_ga_start,
};
