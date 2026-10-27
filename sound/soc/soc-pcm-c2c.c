// SPDX-License-Identifier: GPL-2.0+
//
// soc-pcm-c2c.c -- ALSA SoC PCM for Codec to Codec
//
// Copyright (C) 2026 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//
#include <sound/soc.h>
#include "soc-internal.h"

static int c2c_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	return snd_soc_pcm_new_nopcm(rtd, "codec2codec", false, false);
}

struct snd_soc_rtd_ops rtd_ops_c2c = {
	.new_pcm	= c2c_pcm_new,
};
