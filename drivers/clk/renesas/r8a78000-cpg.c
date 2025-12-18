#define DEBUG	// FIXME
// SPDX-License-Identifier: GPL-2.0
/*
 * R-Car X5H Clock Pulse Generator
 *
 * Copyright (C) 2026 Glider bv
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/dev_printk.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/scmi_protocol.h>
#include <linux/slab.h>

#include <dt-bindings/clock/renesas,r8a78000-cpg.h>

struct clk_map_in {
	int dt_id;		/* DT binding clock ID */
	u32 fw_id;		/* SCMI firmware clock ID */
};

struct clk_map {
	int dt_id;		/* DT binding clock ID */
	u32 fw_id;		/* SCMI firmware clock ID */
	struct clk_hw *hw;
};

struct fw_map {
	u32 impl_ver;
	const struct clk_map_in *map;
};

enum fixed_clk {
	FIXED_CLK_66M,
	FIXED_CLK_266M,
	NUM_FIXED_CLKS
};

static const unsigned long fixed_clk_rates[NUM_FIXED_CLKS] = {
	[FIXED_CLK_66M] = 66666000,
	[FIXED_CLK_266M] = 266660000,
};

#define FIXED_CLK_OFFSET	0x80000000
#define FIXED_CLK(rate)		FIXED_CLK_OFFSET + FIXED_CLK_ ## rate

/**
 * struct r8a78000_cpg_priv - Clock Pulse Generator Private Data
 *
 * @dev: CPG device
 * @scmi_clk_np: Device node in DT for the SCMI firmware clock protocol
 * @map: Mapping from DT clock IDs to SCMI clocks
 * @fixed_hws: Fixed rate clocks used to replace SCMI clocks that do not
 *             support the SCMI CLOCK_ATTRIBUTES command
 */
struct r8a78000_cpg_priv {
	struct device *dev;
	struct device_node *scmi_clk_np;
	const struct clk_map *map;
	struct clk_hw *fixed_hws[NUM_FIXED_CLKS];
};

static const struct clk_map *clk_map_find(const struct clk_map *map, u32 id)
{
	if (!map)
		return NULL;

	for (; map->dt_id >= 0; map++) {
		if (map->dt_id == id)
			return map;
	}

	return NULL;
}

static struct clk_hw *r8a78000_clk_get(struct of_phandle_args *spec,
				      void *data)
{
	struct r8a78000_cpg_priv *priv = data;
	struct device *dev = priv->dev;
	const struct clk_map *map;
	struct clk_hw *hw;
	u32 id;

	if (spec->args_count != 1)
		return ERR_PTR(-EINVAL);

	id = spec->args[0];

	map = clk_map_find(priv->map, id);
	if (!map) {
		dev_err(dev, "Unknown clock %u\n", id);
		return ERR_PTR(-ENOENT);
	}

	if (map->fw_id < FIXED_CLK_OFFSET)
		dev_dbg(dev, "Mapping clock %u to SCMI clock %u\n", id,
			map->fw_id);
	else
		dev_dbg(dev, "Mapping clock %u to fixed clock %u\n", id,
			 map->fw_id - FIXED_CLK_OFFSET);

	hw = map->hw;

	if (!hw) {
		// FIXME CLOCK_ATTRIBUTES is not supported
		dev_warn(dev, "Clock %u is not available\n", id);
		return ERR_PTR(-ENOENT);
	}

	dev_dbg(dev, "clock %u is %s at %lu Hz\n", id, clk_hw_get_name(hw),
		clk_hw_get_rate(hw));

	return hw;
}

static struct device_node *scmi_find_proto(struct device_node *scmi, u32 proto)
{
	for_each_available_child_of_node_scoped(scmi, child) {
		u32 x;

		if (of_property_read_u32(child, "reg", &x))
			continue;

		if (x == proto)
			return_ptr(child);
	}

	return NULL;
}

static void unregister_fixed_clks(void *data)
{
	struct r8a78000_cpg_priv *priv = data;

	for (unsigned int i = 0; i < ARRAY_SIZE(priv->fixed_hws); i++)
		clk_hw_unregister_fixed_rate(priv->fixed_hws[i]);
}

static int register_fixed_clks(struct r8a78000_cpg_priv *priv)
{
	struct device *dev = priv->dev;
	unsigned long rate;
	struct clk_hw *hw;
	const char *name;

	for (unsigned int i = 0; i < ARRAY_SIZE(fixed_clk_rates); i++) {
		rate = fixed_clk_rates[i];
		name = devm_kasprintf(dev, GFP_KERNEL, "cpg-%lu", rate);
		if (!name)
			return -ENOMEM;

		hw = clk_hw_register_fixed_rate(dev, name, NULL, 0, rate);
		if (IS_ERR(hw)) {
			while (i-- > 0)
				clk_hw_unregister_fixed_rate(priv->fixed_hws[i]);
			return PTR_ERR(hw);
		}

		priv->fixed_hws[i] = hw;
	}

	return devm_add_action_or_reset(dev, unregister_fixed_clks, priv);
}

static const struct clk_map *fill_clk_map(struct r8a78000_cpg_priv *priv,
					  const struct clk_map_in *map_in)
{
	struct of_phandle_args scmi_spec;
	struct device *dev = priv->dev;
	struct clk_map *map;
	struct clk_hw *hw;
	struct clk *clk;
	unsigned int i;

	for (i = 0; map_in[i].dt_id >= 0; i++) { }

	map = devm_kcalloc(dev, i + 1, sizeof(*map), GFP_KERNEL);
	if (!map)
		return ERR_PTR(-ENOMEM);

	for (i = 0; ; i++) {
		map[i].dt_id = map_in[i].dt_id;
		if (map[i].dt_id < 0)
			break;

		map[i].fw_id = map_in[i].fw_id;
		if (map[i].fw_id >= FIXED_CLK_OFFSET) {
			enum fixed_clk idx = map[i].fw_id - FIXED_CLK_OFFSET;

			map[i].hw = priv->fixed_hws[idx];
			continue;
		}

		scmi_spec.np = priv->scmi_clk_np;
		scmi_spec.args_count = 1;
		scmi_spec.args[0] = map[i].fw_id;

		clk = of_clk_get_from_provider(&scmi_spec);
		if (IS_ERR(clk))
			return dev_err_cast_probe(dev, clk,
				"Failed to get SCMI clock %u\n",
				map[i].fw_id);

		hw = __clk_get_hw(clk);
		if (IS_ERR(hw))
			return dev_err_cast_probe(dev, hw,
				"Failed to get SCMI clock hw %u\n",
				map[i].fw_id);

		if (!hw) {
			// FIXME CLOCK_ATTRIBUTES is not supported
			dev_warn(dev, "SCMI clock %u is NULL!\n", map[i].fw_id);
			continue;
		}

dev_info(priv->dev, "SCMI clock %u is %s at %lu Hz\n", map[i].fw_id, clk_hw_get_name(hw), clk_hw_get_rate(hw));

		map[i].hw = hw;
	}

	return map;
}

static int r8a78000_cpg_probe(struct platform_device *pdev)
{
	struct device_node *scmi __free(device_node) = NULL;
	struct scmi_revision_info version;
	struct device *dev = &pdev->dev;
	const struct fw_map *fw_map;
	struct r8a78000_cpg_priv *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	scmi = of_parse_phandle(dev->of_node, "firmware", 0);
	if (!scmi) {
		// FIXME Fallback to hardware driver?
		dev_err(dev, "Cannot find SCMI firmware node\n");
		return -ENODEV;
	}

	priv->scmi_clk_np = scmi_find_proto(scmi, SCMI_PROTOCOL_CLOCK);
	if (!priv->scmi_clk_np) {
		// FIXME Fallback to hardware driver?
		return -ENODEV;
	}
dev_dbg(dev, "SCMI clock node is %pOF\n", priv->scmi_clk_np);

	ret = scmi_get_revision_info(scmi, &version);
	if (ret) {
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "SCMI not yet available\n");
	}

	if (strcmp(version.vendor_id, "Renesas") ||
	    strcmp(version.sub_vendor_id, "None")) {
		return dev_err_probe(dev, -ENODEV,
				     "Unsupported SCMI firmware %s/%s\n",
				     version.vendor_id, version.sub_vendor_id);
	}

	for (fw_map = of_device_get_match_data(dev); fw_map->map; fw_map++) {
		if (fw_map->impl_ver == version.impl_ver)
			break;
	}

	if (!fw_map->map) {
		return dev_err_probe(dev, -ENODEV,
				     "Unsupported SCMI version 0x%08x\n",
				     version.impl_ver);
	}

	// FIXME We cannot do lazy look-up in r8a78000_clk_get(), as that
	// FIXME function is called with of_clk_mutex is already held.
	ret = register_fixed_clks(priv);
	if (ret)
		return ret;

	priv->map = fill_clk_map(priv, fw_map->map);
	if (IS_ERR(priv->map))
		return PTR_ERR(priv->map);

	return devm_of_clk_add_hw_provider(dev, r8a78000_clk_get, priv);
}

static const struct clk_map_in r8a78000_cpg_fw_4_28_0[] = {
	{ R8A78000_CPG_SGASYNCD4_PERW_BUS,	FIXED_CLK(266M) },
	{ R8A78000_CPG_SGASYNCD16_PERW_BUS,	FIXED_CLK(66M) },
	{ R8A78000_CPG_MSOCK_PERW_BUS,		1671 },
	{ -1 }
};

static const struct clk_map_in r8a78000_cpg_fw_4_31_0[] = {
	{ R8A78000_CPG_SGASYNCD4_PERW_BUS,	FIXED_CLK(266M) },
	{ R8A78000_CPG_SGASYNCD16_PERW_BUS,	FIXED_CLK(66M) },
	{ R8A78000_CPG_MSOCK_PERW_BUS,		1667 },
	{ -1 }
};

static const struct fw_map r8a78000_cpg_fw_map[] = {
	{ 0x010a0000, r8a78000_cpg_fw_4_28_0 },	/* SCP FW SDKv4.28.0 */
	{ 0x010d0000, r8a78000_cpg_fw_4_31_0 },	/* SCP FW SDKv4.31.0 */
	{ 0x010e0000, r8a78000_cpg_fw_4_31_0 },	/* SCP FW SDKv4.32.0 */
	{ 0, NULL }
};

static const struct of_device_id r8a78000_cpg_match[] = {
	{
		.compatible = "renesas,r8a78000-cpg",
		.data = &r8a78000_cpg_fw_map,
	},
	{ /* sentinel */ }
};

static struct platform_driver r8a78000_cpg_driver = {
	.probe = r8a78000_cpg_probe,
	.driver = {
		.name = "r8a78000-cpg",
		.of_match_table = r8a78000_cpg_match,
		.suppress_bind_attrs = true,
	},
};

// FIXME builtin_platform_driver_probe does not support probe deferral
//builtin_platform_driver_probe(r8a78000_cpg_driver, r8a78000_cpg_probe);
builtin_platform_driver(r8a78000_cpg_driver)

MODULE_DESCRIPTION("R-Car X5H CPG Driver");
