// SPDX-License-Identifier: GPL-2.0-only
// This file incorporates work covered by the following copyright notice:
// Copyright (c) 2020 Intel Corporation
// Copyright (c) 2024 Advanced Micro Devices, Inc.

/*
 *  soc_sdw_rt700 - Helpers to handle RT700 from generic machine driver
 */

#include <linux/device.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <sound/soc-acpi.h>
#include <sound/soc-dapm.h>
#include <sound/jack.h>
#include <sound/soc_sdw_utils.h>

static const struct snd_soc_dapm_route rt700_map[] = {
	/* Headphones */
	{ "Headphones", NULL, "rt700 HP" },
	{ "Speaker", NULL, "rt700 SPK" },
	{ "rt700 MIC2", NULL, "AMIC" },
};

static struct snd_soc_jack_pin rt700_jack_pins[] = {
	{
		.pin    = "Headphones",
		.mask   = SND_JACK_HEADPHONE,
	},
	{
		.pin    = "AMIC",
		.mask   = SND_JACK_MICROPHONE,
	},
};

int asoc_sdw_rt700_rtd_init(struct snd_soc_pcm_runtime *rtd, struct snd_soc_dai *dai)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dapm_context *dapm = snd_soc_card_to_dapm(card);
	struct asoc_sdw_mc_private *ctx = snd_soc_card_to_priv(card);
	struct snd_soc_component *component = snd_soc_dai_to_component(dai);
	struct snd_soc_jack *jack;
	struct device *dev = snd_soc_card_to_dev(card);
	int ret;

	snd_soc_card_set_components(card, devm_kasprintf(dev, GFP_KERNEL,
					  "%s hs:rt700",
					  snd_soc_card_components(card)));
	if (!snd_soc_card_components(card))
		return -ENOMEM;

	ret = snd_soc_dapm_add_routes(dapm, rt700_map,
				      ARRAY_SIZE(rt700_map));

	if (ret) {
		dev_err(dev, "rt700 map addition failed: %d\n", ret);
		return ret;
	}

	ret = snd_soc_card_jack_new_pins(rtd->card, "Headset Jack",
					 SND_JACK_HEADSET | SND_JACK_BTN_0 |
					 SND_JACK_BTN_1 | SND_JACK_BTN_2 |
					 SND_JACK_BTN_3,
					 &ctx->sdw_headset,
					 rt700_jack_pins,
					 ARRAY_SIZE(rt700_jack_pins));
	if (ret) {
		dev_err(dev, "Headset Jack creation failed: %d\n", ret);
		return ret;
	}

	jack = &ctx->sdw_headset;

	snd_jack_set_key(jack->jack, SND_JACK_BTN_0, KEY_PLAYPAUSE);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_1, KEY_VOICECOMMAND);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_2, KEY_VOLUMEUP);
	snd_jack_set_key(jack->jack, SND_JACK_BTN_3, KEY_VOLUMEDOWN);

	ret = snd_soc_component_set_jack(component, jack, NULL);
	if (ret)
		dev_err(dev, "Headset Jack call-back failed: %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_NS(asoc_sdw_rt700_rtd_init, "SND_SOC_SDW_UTILS");
