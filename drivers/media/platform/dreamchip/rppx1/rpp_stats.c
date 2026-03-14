// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include "rppx1.h"

void rppx1_stats_fill_isr(struct rppx1 *rpp, u32 isc, void *buf)
{
	struct rkisp1_stat_buffer *stats = buf;

	stats->meas_type = 0;
}
EXPORT_SYMBOL_GPL(rppx1_stats_fill_isr);
