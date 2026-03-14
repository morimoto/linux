// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2025 Renesas Electronics Corp.
 * Copyright 2025 Niklas Söderlund <niklas.soderlund@ragnatech.se>
 */

#include <media/v4l2-isp.h>
#include <media/videobuf2-v4l2.h>

#include "rppx1.h"

#define RKISP1_PARAMS_BLOCK_INFO(block, data) \
	[RKISP1_EXT_PARAMS_BLOCK_TYPE_ ## block] = { \
		.size = sizeof(struct rkisp1_ext_params_ ## data ## _config), \
	}

static const struct v4l2_isp_params_block_info
rkisp1_ext_params_blocks_info[] = {
	RKISP1_PARAMS_BLOCK_INFO(AWB_GAIN, awb_gain),
	RKISP1_PARAMS_BLOCK_INFO(AWB_MEAS, awb_meas),
};

int rppx1_params(struct rppx1 *rpp, struct vb2_buffer *vb, size_t max_size,
		 rppx1_reg_write write, void *priv)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct rkisp1_ext_params_cfg *cfg;
	size_t block_offset;
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(rpp->dev, vb, max_size);
	if (ret)
		return ret;

	cfg = vb2_plane_vaddr(&vbuf->vb2_buf, 0);

	ret = v4l2_isp_params_validate_buffer(rpp->dev, vb,
					      (struct v4l2_isp_params_buffer *)cfg,
					      rkisp1_ext_params_blocks_info,
					      ARRAY_SIZE(rkisp1_ext_params_blocks_info));
	if (ret)
		return ret;

	/* Walk the list of parameter blocks and process them. */
	block_offset = 0;
	while (block_offset < cfg->data_size) {
		const union rppx1_params_rkisp1_config *block =
			(const union rppx1_params_rkisp1_config *)&cfg->data[block_offset];
		struct rpp_module *module;
		int ret;

		block_offset += block->header.size;

		switch (block->header.type) {
		case RKISP1_EXT_PARAMS_BLOCK_TYPE_AWB_GAIN:
			module = &rpp->pre1.awbg;
			break;
		case RKISP1_EXT_PARAMS_BLOCK_TYPE_AWB_MEAS:
			module = &rpp->post.wbmeas;
			break;
		default:
			module = NULL;
			break;
		}

		if (!module) {
			pr_warn("Not handled RPPX1 block type: 0x%04x\n", block->header.type);
			continue;
		}

		ret = rpp_module_call(module, param_rkisp1, block, write, priv);
		if (ret) {
			pr_err("Error processing RPPX1 block type: 0x%04x\n", block->header.type);
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(rppx1_params);
