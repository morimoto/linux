/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */
#ifndef __MEDIA_DCT_RPPX1_H__
#define __MEDIA_DCT_RPPX1_H__

#include <linux/v4l2-mediabus.h>

#include <media/videobuf2-core.h>

struct rppx1;

struct rppx1 *rppx1_create(void __iomem *base);

void rppx1_destroy(struct rppx1 *rpp);

int rppx1_start(struct rppx1 *rpp, const struct v4l2_mbus_framefmt *input,
		const struct v4l2_mbus_framefmt *hv,
		const struct v4l2_mbus_framefmt *mv);

int rppx1_stop(struct rppx1 *rpp);

bool rppx1_interrupt(struct rppx1 *rpp, u32 *isc);

typedef int (*rppx1_reg_write)(void *priv, u32 offset, u32 value);
int rppx1_params(struct rppx1 *rpp, struct vb2_buffer *vb, size_t max_size,
		 rppx1_reg_write write, void *priv);

void rppx1_stats_fill_isr(struct rppx1 *rpp, u32 isc, void *buf);

#endif /* __MEDIA_DCT_RPPX1_H__ */
