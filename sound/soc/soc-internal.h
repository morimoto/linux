/* SPDX-License-Identifier: GPL-2.0-only
 *
 * soc-internal.h
 *
 * Copyright (c) 2026 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */
#ifndef __SOC_INTERNAL_H
#define __SOC_INTERNAL_H

/*
 * This header is for ALSA SoC Framework internal, not for Vender drivers.
 * The ALSA SoC functions for Vender drivers are defined in linux/include/sound/xxx.h
 * as snd_soc_xxx();
 */

/*
 * In soc-core
 */
extern struct mutex client_mutex;

char *snd_soc_fmt_single_name(struct device *dev, int *id);
char *snd_soc_fmt_multiple_name(struct device *dev, struct snd_soc_dai_driver *dai_drv);
int snd_soc_add_controls(struct snd_card *card, struct device *dev,
			 const struct snd_kcontrol_new *controls, int num_controls,
			 const char *prefix, void *data);
void snd_soc_flush_all_delayed_work(struct snd_soc_card *card);
struct snd_soc_component *snd_soc_find_component(const struct snd_soc_dai_link_component *dlc);
#ifdef CONFIG_PM_SLEEP
void snd_soc_playback_digital_mute(struct snd_soc_card *card, int mute);
void snd_soc_dapm_suspend_resume(struct snd_soc_card *card, int event);
#endif

/*
 * In soc-card
 */
void snd_soc_card_unbind(struct snd_soc_card *card, bool reuse);
void snd_soc_card_rebind(void);
int snd_soc_card_bind_call(struct snd_soc_card *card);

void snd_soc_card_mutex_lock_root(struct snd_soc_card *card);
void snd_soc_card_mutex_lock(struct snd_soc_card *card);
void snd_soc_card_mutex_unlock(struct snd_soc_card *card);
void snd_soc_card_dpcm_mutex_lock(struct snd_soc_card *card);
void snd_soc_card_dpcm_mutex_unlock(struct snd_soc_card *card);
void snd_soc_card_dpcm_mutex_assert_held(struct snd_soc_card *card);
void snd_soc_card_flush_all_delayed_work(struct snd_soc_card *card);
#ifdef CONFIG_PM_SLEEP
int snd_soc_card_deferred_resume(struct snd_soc_card *card);
#endif

/*
 * In soc-dai
 */
void snd_soc_dai_symmetric_set_params(struct snd_soc_dai *dai,
				      struct snd_pcm_hw_params *params);
int snd_soc_dai_symmetric_apply(struct snd_pcm_substream *substream, struct snd_soc_dai *dai);
int snd_soc_dai_symmetric_params(struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params);
void snd_soc_dai_symmetric_update(struct snd_pcm_substream *substream);
void snd_soc_dai_active_action(struct snd_soc_dai *dai, int stream, int action);
void snd_soc_dai_get_bclk(struct snd_soc_dai *dai, struct clk **bclk, unsigned int *bclk_ratio);

/*
 * In soc-component
 */
struct device_node *snd_soc_component_to_node(struct snd_soc_component *component);
bool snd_soc_component_matches_dlc(struct snd_soc_component *component,
				   const struct snd_soc_dai_link_component *dlc);
void snd_soc_component_remove(struct snd_soc_component *component, int probed);
int snd_soc_component_probe(struct snd_soc_card *card, struct snd_soc_component *component);
struct list_head *snd_soc_component_total_list_head(void);
void snd_soc_component_of_put(struct snd_soc_dai_link_component *component);
void snd_soc_component_connect_dai(struct snd_soc_component *component, struct snd_soc_dai *dai);
void snd_soc_component_active_action(struct snd_soc_component *component, int action);
void snd_soc_component_device_link_del(struct snd_soc_component *component);
int snd_soc_component_device_link_add(struct snd_soc_component *component);

#define for_each_component(component)							\
	for (component = snd_soc_component_from_component_total_list(snd_soc_component_total_list_head()->next);	\
	     snd_soc_component_to_component_total_list(component) != snd_soc_component_total_list_head();	\
	     component = snd_soc_component_from_component_total_list(snd_soc_component_to_component_total_list(component)->next))

/*
 *	PCM helper functions
 */
static inline void snd_soc_dpcm_mutex_lock_r(struct snd_soc_pcm_runtime *rtd)
{
	snd_soc_card_dpcm_mutex_lock(rtd->card);
}

static inline void snd_soc_dpcm_mutex_unlock_r(struct snd_soc_pcm_runtime *rtd)
{
	snd_soc_card_dpcm_mutex_unlock(rtd->card);
}

static inline void snd_soc_dpcm_mutex_assert_held_r(struct snd_soc_pcm_runtime *rtd)
{
	snd_soc_card_dpcm_mutex_assert_held(rtd->card);
}

#define snd_soc_dpcm_mutex_lock(x) _Generic((x),		\
struct snd_soc_card * :		snd_soc_card_dpcm_mutex_lock,	\
struct snd_soc_pcm_runtime * :	snd_soc_dpcm_mutex_lock_r)(x)

#define snd_soc_dpcm_mutex_unlock(x) _Generic((x),		\
struct snd_soc_card * :		snd_soc_card_dpcm_mutex_unlock,	\
struct snd_soc_pcm_runtime * :	snd_soc_dpcm_mutex_unlock_r)(x)

#define snd_soc_dpcm_mutex_assert_held(x) _Generic((x),		\
struct snd_soc_card * :		snd_soc_card_dpcm_mutex_assert_held, \
struct snd_soc_pcm_runtime * :	snd_soc_dpcm_mutex_assert_held_r)(x)

#endif /* __SOC_INTERNAL_H */
