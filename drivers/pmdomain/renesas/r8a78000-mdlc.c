// SPDX-License-Identifier: GPL-2.0
/*
 * R-Car X5H Module Controller
 *
 * Copyright (C) 2026 Glider bv
 */

#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/dev_printk.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_clock.h>
#include <linux/pm_domain.h>
#include <linux/reset-controller.h>
#include <linux/scmi_protocol.h>
#include <linux/slab.h>

#include <dt-bindings/power/renesas,r8a78000-mdlc.h>

struct power_map_in {
	int hw_id;		/* Hardware power domain ID or -1 sentinel */
	u32 fw_id;		/* SCMI firmware power domain ID */
};

struct power_map {
	int hw_id;		/* Hardware power domain ID or -1 sentinel */
	u32 fw_id;		/* SCMI firmware power domain ID */
	struct generic_pm_domain *genpd;
};

struct mod_map {
	int hw_id;		/* Hardware module ID or -1 sentinel */
	u32 fw_id;		/* SCMI clock and reset IDs are identical */
};

struct mdlc_info {
	u32 base;
	const struct power_map_in *power_map;
	const struct mod_map *mod_map;
};

struct fw_map {
	u32 impl_ver;
	const struct mdlc_info *info;
};

struct mdlc_data {
	const struct mdlc_info *default_info;
	const struct fw_map *fw_map;
};

/**
 * struct r8a78000_mdlc_priv - Module Controller Private Data
 *
 * @link: Link into list of MDLC instances
 * @genpd_data: PM domain provider data
 * @rcdev: Reset controller entity
 * @dev: MDLC device
 * @np: Device node in DT representing the MDLC
 * @scmi_clk_np: Device node in DT for the SCMI firmware clock protocol
 * @scmi_rcdev: SCMI reset controller entity
 * @power_map: Mapping from hardware power domain IDs to SCMI power domains
 * @mod_map: Mapping from hardware module IDs to SCMI clocks and resets
 */
struct r8a78000_mdlc_priv {
	struct hlist_node link;
	struct genpd_onecell_data genpd_data;
	struct reset_controller_dev rcdev;
	struct device *dev;
	struct device_node *np;
	struct device_node *scmi_clk_np;
	struct reset_controller_dev *scmi_rcdev;
	const struct power_map *power_map;
	const struct mod_map *mod_map;
};

static struct generic_pm_domain *r8a78000_genpd_always_on;
static HLIST_HEAD(r8a78000_mdlc_list);
static DEFINE_MUTEX(r8a78000_mdlc_lock);	/* protects the two above */

static const struct power_map *power_map_find(const struct power_map *map,
					      u32 id)
{
	if (!map)
		return NULL;

	for (; map->hw_id >= 0; map++) {
		if (map->hw_id == id)
			return map;
	}

	return NULL;
}

static struct generic_pm_domain *r8a78000_genpd_xlate(
			const struct of_phandle_args *spec, void *data)
{
	struct r8a78000_mdlc_priv *priv = container_of(data,
					struct r8a78000_mdlc_priv, genpd_data);
	struct generic_pm_domain *genpd;
	struct device *dev = priv->dev;
	const struct power_map *map;
	u32 id;

	if (spec->args_count != 2)
		return ERR_PTR(-EINVAL);

	id = spec->args[0];

	if (id >= R8A78000_MDLC_PD_AON) {
		dev_dbg(dev,
			"Mapping HW power domain 0x%x to always-on domain\n",
			id);
		return r8a78000_genpd_always_on;
	}

	map = power_map_find(priv->power_map, id);
	if (!map) {
		dev_err(dev, "Unknown power domain 0x%x\n", id);
		return ERR_PTR(-ENOENT);
	}

	dev_dbg(dev, "Mapping HW power domain 0x%x to SCMI power domain %u\n",
		id, map->fw_id);

	genpd = map->genpd;

	return genpd;
}

#define rcdev_to_priv(_rcdev)	\
	container_of(_rcdev, struct r8a78000_mdlc_priv, rcdev)

static const struct mod_map *mod_map_find(const struct mod_map *map, u32 id)
{
	if (!map)
		return NULL;

	for (; map->hw_id >= 0; map++) {
		if (map->hw_id == id)
			return map;
	}

	return NULL;
}

static int r8a78000_mdlc_reset_xlate(struct reset_controller_dev *rcdev,
				     const struct of_phandle_args *spec)
{
	struct r8a78000_mdlc_priv *priv = rcdev_to_priv(rcdev);
	struct device *dev = priv->dev;
	const struct mod_map *map;
	u32 id;

	if (spec->args_count != 1)
		return -EINVAL;

	id = spec->args[0];

	map = mod_map_find(priv->mod_map, id);
	if (!map) {
		dev_err(dev, "Unknown reset 0x%x\n", id);
		return -ENOENT;
	}

	if (!priv->scmi_rcdev) {
		dev_dbg(dev, "Ignoring HW reset 0x%x\n", id);
		return id;
	}

	dev_dbg(dev, "Mapping HW reset 0x%x to SCMI reset %u\n", id,
		map->fw_id);

	return map->fw_id;
}

#define DEFINE_MDLC_RESET_WRAPPER(op)					    \
	static int r8a78000_mdlc_ ## op(struct reset_controller_dev *rcdev, \
					unsigned long id)		    \
	{								    \
		struct r8a78000_mdlc_priv *priv = rcdev_to_priv(rcdev);	    \
		int ret;						    \
									    \
		if (!priv->scmi_rcdev) {				    \
			dev_dbg(priv->dev, "%s: Ignoring\n", __func__);	    \
			return 0;					    \
		}							    \
									    \
		if (!priv->scmi_rcdev->ops->op)				    \
			return -ENOTSUPP;				    \
									    \
		ret = priv->scmi_rcdev->ops->op(priv->scmi_rcdev, id);	    \
		if (ret == -EOPNOTSUPP)					    \
			dev_dbg(priv->dev,				    \
				"%s: Ignoring unsupported reset %lu\n",	    \
				__func__, id);				    \
		return ret == -EOPNOTSUPP ? 0 : ret;			    \
	}

DEFINE_MDLC_RESET_WRAPPER(reset)
DEFINE_MDLC_RESET_WRAPPER(assert)
DEFINE_MDLC_RESET_WRAPPER(deassert)
DEFINE_MDLC_RESET_WRAPPER(status)

static const struct reset_control_ops r8a78000_mdlc_reset_ops = {
	.reset = r8a78000_mdlc_reset,
	.assert = r8a78000_mdlc_assert,
	.deassert = r8a78000_mdlc_deassert,
	.status = r8a78000_mdlc_status,
};

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

static int r8a78000_mdlc_attach_dev(struct generic_pm_domain *domain,
				    struct device *dev)
{
	struct of_phandle_args pd_spec, scmi_spec;
	struct device_node *np = dev->of_node;
	struct r8a78000_mdlc_priv *priv;
	const struct mod_map *map;
	unsigned int id;
	struct clk *clk;
	int ret;

	ret = of_parse_phandle_with_args(np, "power-domains",
					 "#power-domain-cells", 0, &pd_spec);
	if (ret < 0)
		return ret;

	if (pd_spec.args_count != 2) {
		of_node_put(pd_spec.np);
		return -EINVAL;
	}

	scoped_guard(mutex, &r8a78000_mdlc_lock) {
		hlist_for_each_entry(priv, &r8a78000_mdlc_list, link) {
			if (priv->np == pd_spec.np)
				break;
		}
	}

	if (!priv) {
		dev_err(dev, "%s: MDLC %pOF not found\n", __func__, pd_spec.np);
		of_node_put(pd_spec.np);
		return -ENODEV;
	}

	id = pd_spec.args[1];
	of_node_put(pd_spec.np);

	map = mod_map_find(priv->mod_map, id);
	if (!map) {
		dev_err(dev, "Unknown module 0x%x\n", id);
		return -ENOENT;
	}

	if (!priv->scmi_clk_np) {
		dev_dbg(dev, "Ignoring HW module 0x%x\n", id);
		return 0;
	}

	dev_dbg(dev, "Mapping HW module 0x%x to SCMI clock %u\n", id,
		map->fw_id);

	scmi_spec.np = priv->scmi_clk_np;
	scmi_spec.args_count = 1;
	scmi_spec.args[0] = map->fw_id;

	clk = of_clk_get_from_provider(&scmi_spec);
	if (IS_ERR(clk)) {
		dev_err(dev, "Cannot get SCMI clock %u: %pe\n", map->fw_id,
			clk);
		return PTR_ERR(clk);
	}

	dev_dbg(dev, "SCMI clock %u is %pC\n", map->fw_id, clk);

	if (!clk) {
		/* Ignore missing SCMI module clocks */
		return 0;
	}

	ret = pm_clk_create(dev);
	if (ret)
		goto fail_put;

	ret = pm_clk_add_clk(dev, clk);
	if (ret)
		goto fail_destroy;

	return 0;

fail_destroy:
	pm_clk_destroy(dev);
fail_put:
	clk_put(clk);
	return ret;
}

static void r8a78000_mdlc_detach_dev(struct generic_pm_domain *domain,
				     struct device *dev)
{
	if (!pm_clk_no_clocks(dev))
		pm_clk_destroy(dev);
}

static const struct power_map *fill_power_map(struct r8a78000_mdlc_priv *priv,
					      const struct power_map_in *map_in,
					      struct device_node *scmi_power_np)
{
	struct of_phandle_args scmi_spec;
	struct generic_pm_domain *genpd;
	struct device *dev = priv->dev;
	struct power_map *map;
	unsigned int i;

	if (!map_in)
		return NULL;

	for (i = 0; map_in[i].hw_id >= 0; i++) { }

	map = devm_kcalloc(dev, i + 1, sizeof(*map), GFP_KERNEL);
	if (!map)
		return ERR_PTR(-ENOMEM);

	for (i = 0; ; i++) {
		map[i].hw_id = map_in[i].hw_id;
		if (map[i].hw_id < 0)
			break;

		map[i].fw_id = map_in[i].fw_id;

		scmi_spec.np = scmi_power_np;
		scmi_spec.args_count = 1;
		scmi_spec.args[0] = map[i].fw_id;

		genpd = genpd_get_from_provider(&scmi_spec);
		if (IS_ERR(genpd))
			return dev_err_cast_probe(dev, genpd,
					"Failed to get SCMI power domain %u\n",
					map[i].fw_id);

		dev_dbg(dev, "SCMI power domain %u is %s\n", map[i].fw_id,
			genpd->name);

		map[i].genpd = genpd;

		/* Hook up clock domain support */
		genpd->attach_dev = r8a78000_mdlc_attach_dev;
		genpd->detach_dev = r8a78000_mdlc_detach_dev;
		/* Setting flags this late has no impact, but does not hurt */
		genpd->flags |= GENPD_FLAG_PM_CLK;
		genpd->dev_ops.stop = pm_clk_suspend;
		genpd->dev_ops.start = pm_clk_resume;
	}

	return map;
}

static void r8a78000_mdlc_unlink(void *data)
{
	struct r8a78000_mdlc_priv *priv = data;

	scoped_guard(mutex, &r8a78000_mdlc_lock) {
		hlist_del(&priv->link);
	}
}

static void r8a78000_genpd_del_provider(void *data)
{
	of_genpd_del_provider(data);
}

static int r8a78000_genpd_always_on_singleton(struct device *dev)
{
	struct generic_pm_domain *genpd;
	int ret;

	guard(mutex)(&r8a78000_mdlc_lock);

	if (r8a78000_genpd_always_on)
		return 0;

	genpd = kzalloc(sizeof(*genpd), GFP_KERNEL);
	if (!genpd)
		return -ENOMEM;

	genpd->name = "always-on";
	genpd->attach_dev = r8a78000_mdlc_attach_dev;
	genpd->detach_dev = r8a78000_mdlc_detach_dev;
	genpd->flags |= GENPD_FLAG_PM_CLK;

	ret = pm_genpd_init(genpd, &pm_domain_always_on_gov, false);
	if (ret) {
		kfree(genpd);
		return dev_err_probe(dev, ret,
				     "Failed to create always-on domain\n");
	}

	r8a78000_genpd_always_on = genpd;
	return 0;
}

static int r8a78000_mdlc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *scmi __free(device_node) = NULL;
	struct device_node *scmi_power_np = NULL;
	const struct mdlc_data *mdlc_data;
	struct device_node *scmi_reset_np;
	struct device_node *scmi_clk_np;
	struct r8a78000_mdlc_priv *priv;
	struct scmi_base_info version;
	const struct mdlc_info *info;
	const struct fw_map *fw_map;
	struct resource *res;
	int ret;

	ret = r8a78000_genpd_always_on_singleton(dev);
	if (ret)
		return ret;

	mdlc_data = of_device_get_match_data(dev);
	if (!mdlc_data)
		return -ENODEV;

	info = mdlc_data->default_info;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->np = np;

	scmi = of_parse_phandle(dev->of_node, "firmware", 0);
	if (!scmi) {
		dev_dbg(dev, "Cannot find SCMI firmware node\n");
		goto fallback;
	}

	if (!of_device_is_available(scmi)) {
		dev_dbg(dev, "SCMI firmware node is not available\n");
		goto fallback;
	}

	scmi_power_np = scmi_find_proto(scmi, SCMI_PROTOCOL_POWER);
	if (!scmi_power_np) {
		dev_dbg(dev,
			"Cannot find SCMI power domain management protocol\n");
		goto fallback;
	}

	scmi_clk_np = scmi_find_proto(scmi, SCMI_PROTOCOL_CLOCK);
	if (!scmi_clk_np) {
		dev_dbg(dev, "Cannot find SCMI clock management protocol\n");
		goto fallback;
	}

	scmi_reset_np = scmi_find_proto(scmi, SCMI_PROTOCOL_RESET);
	if (!scmi_reset_np) {
		dev_dbg(dev, "Cannot find SCMI reset management protocol\n");
		goto fallback;
	}

	ret = scmi_get_base_info(scmi, &version);
	if (ret == -EPROBE_DEFER)
		return dev_err_probe(dev, ret, "SCMI provider not ready\n");
	if (ret) {
		dev_dbg(dev, "SCMI is not available\n");
		goto fallback;
	}

	if (strcmp(version.vendor_id, "Renesas") ||
	    strcmp(version.sub_vendor_id, "None")) {
		dev_warn(dev, "Unsupported SCMI firmware %s/%s\n",
			 version.vendor_id, version.sub_vendor_id);
		goto fallback;
	}

	for (fw_map = mdlc_data->fw_map; fw_map->info; fw_map++) {
		if (fw_map->impl_ver == version.impl_ver)
			break;
	}

	if (!fw_map->info) {
		dev_warn(dev, "Unsupported SCMI firmware version 0x%08x\n",
			 version.impl_ver);
		goto fallback;
	}

	priv->scmi_rcdev = reset_controller_get_provider(scmi_reset_np);
	if (!priv->scmi_rcdev)
		return dev_err_probe(dev, -EPROBE_DEFER,
				     "SCMI reset not yet available\n");

	priv->scmi_clk_np = scmi_clk_np;
	info = fw_map->info;

fallback:
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	for (; info->base; info++) {
		if (info->base == res->start)
			break;
	}

	if (!info->base) {
		dev_dbg(dev, "Unsupported MDLC instance 0x%pa\n", &res->start);
		return -ENODEV;
	}

	/*
	 * We cannot do lazy look-up in r8a78000_genpd_xlate(), as that
	 * function is called with of_genpd_mutex already held.
	 */
	priv->power_map = fill_power_map(priv, info->power_map, scmi_power_np);
	if (IS_ERR(priv->power_map))
		return PTR_ERR(priv->power_map);

	priv->mod_map = info->mod_map;

	scoped_guard(mutex, &r8a78000_mdlc_lock) {
		hlist_add_head(&priv->link, &r8a78000_mdlc_list);
	}

	ret = devm_add_action_or_reset(dev, r8a78000_mdlc_unlink, priv);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add action\n");

	/* Note that no actual domains are registered, just need translation */
	priv->genpd_data.xlate = r8a78000_genpd_xlate;
	ret = of_genpd_add_provider_onecell(np, &priv->genpd_data);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register genpd provider\n");

	ret = devm_add_action_or_reset(dev, r8a78000_genpd_del_provider, np);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to add unregister action\n");

	priv->rcdev.ops = &r8a78000_mdlc_reset_ops;
	priv->rcdev.of_node = np;
	priv->rcdev.of_reset_n_cells = 1;
	priv->rcdev.of_xlate = r8a78000_mdlc_reset_xlate;

	ret = devm_reset_controller_register(dev, &priv->rcdev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register reset controller\n");

	return 0;
}

static const struct mod_map r8a78000_mdlc_perw_mod_default[] = {
	{ 0x54 },	/* HSCIF0 */
	{ -1 },
};

static const struct mdlc_info r8a78000_mdlc_default[] = {
	{
		.base = 0xc05d0000 /* mdlc_perw */,
		.mod_map = r8a78000_mdlc_perw_mod_default,
	},
	{ 0 }
};

static const struct fw_map r8a78000_mdlc_fw_map[] = {
	{ 0, NULL }
};

static const struct mdlc_data r8a78000_mdlc_data = {
	.default_info = r8a78000_mdlc_default,
	.fw_map = r8a78000_mdlc_fw_map,
};

static const struct of_device_id r8a78000_mdlc_match[] = {
	{
		.compatible = "renesas,r8a78000-mdlc",
		.data = &r8a78000_mdlc_data,
	},
	{ /* sentinel */ }
};

static struct platform_driver r8a78000_mdlc_driver = {
	.probe = r8a78000_mdlc_probe,
	.driver = {
		.name = "r8a78000-mdlc",
		.of_match_table = r8a78000_mdlc_match,
		.suppress_bind_attrs = true,
	},
};

builtin_platform_driver(r8a78000_mdlc_driver)

MODULE_DESCRIPTION("R-Car X5H MDLC Driver");
