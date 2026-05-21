// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright(c) 2021-2022 Intel Corporation
//
// Authors: Cezary Rojewski <cezary.rojewski@intel.com>
//          Amadeusz Slawinski <amadeuszx.slawinski@linux.intel.com>
//

#include <linux/device.h>
#include <linux/module.h>
#include <sound/soc.h>

static int avs_create_dai_links(struct device *dev, struct snd_soc_dai_link **links, int *num_links)
{
	struct snd_soc_dai_link *dl;

	dl = devm_kzalloc(dev, sizeof(*dl), GFP_KERNEL);
	if (!dl)
		return -ENOMEM;

	dl->cpus = devm_kzalloc(dev, sizeof(*dl->cpus), GFP_KERNEL);
	dl->platforms = devm_kzalloc(dev, sizeof(*dl->platforms), GFP_KERNEL);
	if (!dl->cpus || !dl->platforms)
		return -ENOMEM;

	dl->name = "Compress Probe Capture";
	dl->cpus->dai_name = "Probe Extraction CPU DAI";
	dl->num_cpus = 1;
	dl->codecs = &snd_soc_dummy_dlc;
	dl->num_codecs = 1;
	dl->platforms->name = dev_name(dev);
	dl->num_platforms = 1;
	dl->nonatomic = 1;

	*links = dl;
	*num_links = 1;
	return 0;
}

static int avs_probe_mb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card;
	struct snd_soc_card_driver *card_driver;
	int ret;

	card = snd_soc_card_alloc(dev);
	card_driver = devm_kzalloc(dev, sizeof(*card_driver), GFP_KERNEL);
	if (!card || !card_driver)
		return -ENOMEM;

	ret = avs_create_dai_links(dev, &card_driver->dai_link, &card_driver->num_links);
	if (ret)
		return ret;

	card_driver->driver_name = "avs_probe_mb";
	card_driver->owner = THIS_MODULE;
	card_driver->fully_routed = true;
	snd_soc_card_set_name(card, "AVS PROBE");
	snd_soc_card_set_long_name(card, "AVS PROBE");

	return devm_snd_soc_card_register(card, card_driver);
}

static const struct platform_device_id avs_probe_mb_driver_ids[] = {
	{
		.name = "avs_probe_mb",
	},
	{ }
};
MODULE_DEVICE_TABLE(platform, avs_probe_mb_driver_ids);

static struct platform_driver avs_probe_mb_driver = {
	.probe = avs_probe_mb_probe,
	.driver = {
		.name = "avs_probe_mb",
		.pm = &snd_soc_pm_ops,
	},
	.id_table = avs_probe_mb_driver_ids,
};

module_platform_driver(avs_probe_mb_driver);

MODULE_DESCRIPTION("Intel probe machine driver");
MODULE_LICENSE("GPL");
