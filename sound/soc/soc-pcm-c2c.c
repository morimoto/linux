// SPDX-License-Identifier: GPL-2.0+
//
// soc-pcm-c2c.c -- ALSA SoC PCM for Codec to Codec
//
// Copyright (C) 2026 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//
#include <sound/soc.h>
#include "soc-internal.h"

int snd_soc_pcm_new_c2c(struct snd_soc_pcm_runtime *rtd)
{
	return snd_soc_pcm_new_nopcm(rtd, "codec2codec", false, false);
}
