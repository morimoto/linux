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
	int dt_id;		/* DT binding clock ID or -1 sentinel */
	u32 fw_id;		/* SCMI firmware clock ID or FIXED_CLK() ID */
};

struct clk_map {
	int dt_id;		/* DT binding clock ID or -1 sentinel */
	u32 fw_id;		/* SCMI firmware clock ID or FIXED_CLK() ID */
	struct clk_hw *hw;
};

struct cpg_data {
	const struct clk_map_in *default_map;
	const struct clk_map_in *fw_map;
};

enum fixed_clk {
	FIXED_CLK_66M,
	FIXED_CLK_133M,
	FIXED_CLK_266M,
};

static const unsigned long fixed_clk_rates[] = {
	[FIXED_CLK_66M] = 66666000,
	[FIXED_CLK_133M] = 133330000,
	[FIXED_CLK_266M] = 266660000,
};

#define FIXED_CLK_OFFSET	0x80000000
#define FIXED_CLK(rate)		FIXED_CLK_OFFSET + FIXED_CLK_ ## rate

/**
 * struct r8a78000_cpg_priv - Clock Pulse Generator Private Data
 *
 * @dev: CPG device
 * @map: Mapping from DT clock IDs to SCMI or fixed-rate clocks
 */
struct r8a78000_cpg_priv {
	struct device *dev;
	const struct clk_map *map;
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
		dev_dbg(dev, "Mapping DT clock %u to SCMI clock %u\n", id,
			map->fw_id);
	else
		dev_dbg(dev, "Mapping DT clock %u to fixed clock %u\n", id,
			 map->fw_id - FIXED_CLK_OFFSET);

	hw = map->hw;
	if (!hw) {
		/* CLOCK_ATTRIBUTES is not supported */
		dev_err(dev, "Clock %u is not available\n", id);
		return ERR_PTR(-ENOENT);
	}

	dev_dbg(dev, "clock %u is %s at %lu Hz\n", id, clk_hw_get_name(hw),
		clk_hw_get_rate(hw));

	return hw;
}

static struct device_node *scmi_find_proto(struct device_node *scmi, u32 proto)
{
	for_each_available_child_of_node_scoped(scmi, child) {
		u32 reg;

		if (of_property_read_u32(child, "reg", &reg))
			continue;

		if (reg == proto)
			return_ptr(child);
	}

	return NULL;
}

static struct clk_hw **register_fixed_clks(struct device *dev)
{
	unsigned int n = ARRAY_SIZE(fixed_clk_rates);
	struct clk_hw **hws, *hw;
	unsigned long rate;
	const char *name;

	hws = devm_kcalloc(dev, n, sizeof(*hws), GFP_KERNEL);
	if (!hws)
		return ERR_PTR(-ENOMEM);

	for (unsigned int i = 0; i < n; i++) {
		rate = fixed_clk_rates[i];
		name = devm_kasprintf(dev, GFP_KERNEL, "cpg-%lu", rate);
		if (!name)
			return ERR_PTR(-ENOMEM);

		hw = devm_clk_hw_register_fixed_rate(dev, name, NULL, 0, rate);
		if (IS_ERR(hw))
			return ERR_CAST(hw);

		hws[i] = hw;
	}

	return hws;
}

static const struct clk_map *fill_clk_map(struct r8a78000_cpg_priv *priv,
					  const struct clk_map_in *map_in,
					  const struct clk_map_in *fallback,
					  struct device_node *scmi_clk_np)
{
	struct of_phandle_args scmi_spec;
	struct device *dev = priv->dev;
	struct clk_hw **fixed_hws;
	struct clk_map *map;
	struct clk_hw *hw;
	struct clk *clk;
	unsigned int i;

	fixed_hws = register_fixed_clks(dev);
	if (IS_ERR(fixed_hws))
		return ERR_CAST(fixed_hws);

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

			map[i].hw = fixed_hws[idx];
			continue;
		}

		scmi_spec.np = scmi_clk_np;
		scmi_spec.args_count = 1;
		scmi_spec.args[0] = map[i].fw_id;

		clk = of_clk_get_from_provider(&scmi_spec);
		if (IS_ERR(clk))
			return dev_err_cast_probe(dev, clk,
				"Failed to get SCMI clock %u\n", map[i].fw_id);

		hw = __clk_get_hw(clk);
		if (!hw && fallback != map_in) {
			for (unsigned int j = 0; fallback[j].dt_id >= 0; j++) {
				if (fallback[j].dt_id == map_in[i].dt_id) {
					enum fixed_clk idx = fallback[j].fw_id - FIXED_CLK_OFFSET;

					map[i].hw = fixed_hws[idx];
					break;
				}
			}
			if (map[i].hw)
				continue;
		}

		if (!hw) {
			/* CLOCK_ATTRIBUTES is not supported */
			dev_warn(dev, "SCMI clock %u is NULL\n", map[i].fw_id);
			continue;
		}

		dev_dbg(priv->dev, "SCMI clock %u is %s at %lu Hz\n",
			map[i].fw_id, clk_hw_get_name(hw), clk_hw_get_rate(hw));

		map[i].hw = hw;
	}

	devm_kfree(dev, fixed_hws);
	return map;
}

static int r8a78000_cpg_probe(struct platform_device *pdev)
{
	struct device_node *scmi_clk_np __free(device_node) = NULL;
	struct device_node *scmi __free(device_node) = NULL;
	struct device *dev = &pdev->dev;
	const struct cpg_data *cpg_data;
	struct r8a78000_cpg_priv *priv;
	const struct clk_map_in *map;

	cpg_data = of_device_get_match_data(dev);
	if (!cpg_data)
		return -ENODEV;

	map = cpg_data->default_map;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	if (!IS_ENABLED(CONFIG_COMMON_CLK_SCMI)) {
		dev_dbg(dev, "SCMI clock support is disabled\n");
		goto fallback;
	}

	scmi = of_find_compatible_node(NULL, NULL, "arm,scmi");
	if (!scmi) {
		dev_dbg(dev, "Cannot find SCMI firmware node\n");
		goto fallback;
	}

	if (!of_device_is_available(scmi)) {
		dev_dbg(dev, "SCMI firmware node is not available\n");
		goto fallback;
	}

	scmi_clk_np = scmi_find_proto(scmi, SCMI_PROTOCOL_CLOCK);
	if (!scmi_clk_np) {
		dev_dbg(dev, "Cannot find SCMI clock management protocol\n");
		goto fallback;
	}

	map = cpg_data->fw_map;

fallback:
	/*
	 * We cannot do lazy look-up in r8a78000_clk_get(), as that function is
	 * called with of_clk_mutex already held.
	 */
	priv->map = fill_clk_map(priv, map, cpg_data->default_map, scmi_clk_np);
	if (IS_ERR(priv->map))
		return PTR_ERR(priv->map);

	return devm_of_clk_add_hw_provider(dev, r8a78000_clk_get, priv);
}

static const struct clk_map_in r8a78000_cpg_default[] = {
	{ R8A78000_CPG_SGASYNCD4_PERW_BUS,	FIXED_CLK(266M) },
	{ R8A78000_CPG_SGASYNCD8_PERW_BUS,	FIXED_CLK(133M) },
	{ R8A78000_CPG_SGASYNCD16_PERW_BUS,	FIXED_CLK(66M) },
	{ -1 }
};

static const struct clk_map_in r8a78000_cpg_fw[] = {
	{ R8A78000_CPG_SGASYNCD4_PERW_BUS,	1661 },
	{ R8A78000_CPG_SGASYNCD8_PERW_BUS,	1662 },
	{ R8A78000_CPG_SGASYNCD16_PERW_BUS,	1663 },
	{ R8A78000_CPG_MSO_SCP_MAIN,		1459 },
	{ R8A78000_CPG_MSOCK_PERW_BUS,		1667 },
	{ -1 }
};

static const struct cpg_data r8a78000_cpg_data = {
	.default_map = r8a78000_cpg_default,
	.fw_map = r8a78000_cpg_fw,
};

static const struct of_device_id r8a78000_cpg_match[] = {
	{
		.compatible = "renesas,r8a78000-cpg",
		.data = &r8a78000_cpg_data,
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

builtin_platform_driver(r8a78000_cpg_driver)

MODULE_DESCRIPTION("R-Car X5H CPG Driver");
