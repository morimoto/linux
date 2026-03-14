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

	if (isc & RPPX1_IRQ_ID_POST_AWB_MEAS)
		if (!rpp_module_call(&rpp->post.wbmeas, stats_rkisp1, &stats->params))
			stats->meas_type |= RKISP1_CIF_ISP_STAT_AWB;

	if (isc & RPPX1_IRQ_ID_PRE1_EXM)
		if (!rpp_module_call(&rpp->pre1.exm, stats_rkisp1, &stats->params))
			stats->meas_type |= RKISP1_CIF_ISP_STAT_AUTOEXP;
}
EXPORT_SYMBOL_GPL(rppx1_stats_fill_isr);
