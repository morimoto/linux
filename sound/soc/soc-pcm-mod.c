// SPDX-License-Identifier: GPL-2.0+
//
// soc-pcm-mod.c  --  ALSA SoC PCM for module
//
// Copyright (C) 2026 Renesas Electronics Corp.
//
// Authors: Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//
#include <sound/soc.h>
#include "soc-internal.h"

#define soc_pcm_ret(rtd, ret) snd_soc_ret(rtd->dev, ret, "on %s\n", rtd->dai_link->name)

static int soc_get_playback_capture(struct snd_soc_pcm_runtime *rtd,
				    int *playback, int *capture)
{
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_soc_dai_link_ch_map *ch_maps;
	int has_playback = 0;
	int has_capture  = 0;
	int i, ret = -EINVAL;

	/*
	 * see
	 *	soc.h :: [dai_link->ch_maps Image sample]
	 */
	for_each_rtd_ch_maps(rtd, i, ch_maps) {
		struct snd_soc_dai *cpu_dai   = snd_soc_rtd_to_cpu(rtd,   ch_maps->cpu);
		struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, ch_maps->codec);

		if (snd_soc_dai_stream_valid(codec_dai, SNDRV_PCM_STREAM_PLAYBACK) &&
		    snd_soc_dai_stream_valid(cpu_dai,   SNDRV_PCM_STREAM_PLAYBACK))
			has_playback = 1;
		if (snd_soc_dai_stream_valid(codec_dai, SNDRV_PCM_STREAM_CAPTURE) &&
		    snd_soc_dai_stream_valid(cpu_dai,   SNDRV_PCM_STREAM_CAPTURE))
			has_capture = 1;
	}

	if (dai_link->playback_only)
		has_capture = 0;

	if (dai_link->capture_only)
		has_playback = 0;

	if (has_playback || has_capture) {
		ret = 0;
		*playback = has_playback;
		*capture  = has_capture;
	}

	return soc_pcm_ret(rtd, ret);
}

static int soc_create_pcm(struct snd_pcm **pcm,
			  struct snd_soc_pcm_runtime *rtd,
			  int playback, int capture)
{
	struct snd_card *snd_card = snd_soc_card_to_snd_card(rtd->card);
	char new_name[64];
	int ret;

	snprintf(new_name, sizeof(new_name), "%s-%d", rtd->dai_link->stream_name, rtd->id);

	ret = snd_pcm_new(snd_card, new_name, rtd->id, playback, capture, pcm);

	return soc_pcm_ret(rtd, ret);
}

/* create a new pcm */
int soc_new_pcm_mod(struct snd_soc_pcm_runtime *rtd)
{
//	struct snd_soc_component *component;
	struct snd_pcm *pcm;
	int ret = 0, playback = 0, capture = 0;

	ret = soc_get_playback_capture(rtd, &playback, &capture);
	if (ret < 0)
		goto err;

	ret = soc_create_pcm(&pcm, rtd, playback, capture);
	if (ret < 0)
		goto err;

	rtd->pcm		= pcm;
	pcm->nonatomic		= rtd->dai_link->nonatomic;
	pcm->private_data	= rtd;
	pcm->no_device_suspend	= true;

	rtd->ops.open		= soc_pcm_open;
	rtd->ops.hw_params	= soc_pcm_hw_params;
	rtd->ops.prepare	= soc_pcm_prepare;
	rtd->ops.trigger	= soc_pcm_trigger;
	rtd->ops.hw_free	= soc_pcm_hw_free;
	rtd->ops.close		= soc_pcm_close;
	rtd->ops.pointer	= soc_pcm_pointer;

/*
	for_each_rtd_components(rtd, i, component) {
		const struct snd_soc_component_driver *drv = snd_soc_component_to_driver(component);

		if (drv->ioctl)
			rtd->ops.ioctl		= snd_soc_pcm_component_ioctl;
		if (drv->sync_stop)
			rtd->ops.sync_stop	= snd_soc_pcm_component_sync_stop;
		if (drv->copy)
			rtd->ops.copy		= snd_soc_pcm_component_copy;
		if (drv->page)
			rtd->ops.page		= snd_soc_pcm_component_page;
		if (drv->mmap)
			rtd->ops.mmap		= snd_soc_pcm_component_mmap;
		if (drv->ack)
			rtd->ops.ack            = snd_soc_pcm_component_ack;
	}
*/
	if (playback)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &rtd->ops);

	if (capture)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &rtd->ops);

	ret = snd_soc_pcm_component_new(rtd);
//	if (ret < 0)
//		goto err;
//out:
//	dev_dbg(snd_soc_card_to_dev(rtd->card), "%s <-> %s mapping ok\n",
//		soc_codec_dai_name(rtd), soc_cpu_dai_name(rtd));
err:
	return soc_pcm_ret(rtd, ret);
}
