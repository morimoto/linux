// SPDX-License-Identifier: GPL-2.0-only
// This file incorporates work covered by the following copyright notice:
// Copyright (c) 2024 Intel Corporation
// Copyright (c) 2024 Advanced Micro Devices, Inc.

#include <linux/device.h>
#include <linux/errno.h>
#include <sound/soc.h>
#include <sound/soc-acpi.h>
#include <sound/soc-dapm.h>
#include <sound/soc_sdw_utils.h>

/* Card pin "Dmic" → codec input (name_prefix + " DMIC") */
static const struct snd_soc_dapm_route senary_dmic_map[] = {
	{ "sn624x DMIC", NULL, "Dmic" },
};

int asoc_sdw_senary_dmic_rtd_init(struct snd_soc_pcm_runtime *rtd, struct snd_soc_dai *dai)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(card);
	struct snd_soc_component *component;
	struct device *dev = snd_soc_card_to_dev(card);
	char *mic_name;
	int ret;

	component = snd_soc_dai_to_component(dai);

	ret = snd_soc_dapm_add_routes(dapm, senary_dmic_map,
				      ARRAY_SIZE(senary_dmic_map));
	if (ret) {
		dev_err(dev, "senary dmic map addition failed: %d\n", ret);
		return ret;
	}

	mic_name = devm_kasprintf(dev, GFP_KERNEL, "%s",
				  snd_soc_component_name_prefix(component));
	if (!mic_name)
		return -ENOMEM;

	snd_soc_card_set_components(card,
				devm_kasprintf(dev, GFP_KERNEL,
					"%s mic:%s-dmic",
					snd_soc_card_components(card),
					mic_name));
	if (!snd_soc_card_components(card))
		return -ENOMEM;

	dev_dbg(dev, "card components: %s\n", snd_soc_card_components(card));

	return 0;
}
EXPORT_SYMBOL_NS(asoc_sdw_senary_dmic_rtd_init, "SND_SOC_SDW_UTILS");
