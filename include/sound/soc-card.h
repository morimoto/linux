/* SPDX-License-Identifier: GPL-2.0
 *
 * soc-card.h
 *
 * Copyright (C) 2019 Renesas Electronics Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */
#ifndef __SOC_CARD_H
#define __SOC_CARD_H

enum snd_soc_card_subclass {
	SND_SOC_CARD_CLASS_ROOT		= 0,
	SND_SOC_CARD_CLASS_RUNTIME	= 1,
};

int devm_snd_soc_register_card(struct device *dev, struct snd_soc_card *card);

int snd_soc_card_add_controls(struct snd_soc_card *soc_card,
			      const struct snd_kcontrol_new *controls, int num_controls);
struct snd_kcontrol *snd_soc_card_get_kcontrol(struct snd_soc_card *soc_card,
					       const char *name);
int snd_soc_card_jack_new(struct snd_soc_card *card, const char *id, int type,
			  struct snd_soc_jack *jack);
int snd_soc_card_jack_new_pins(struct snd_soc_card *card, const char *id,
			       int type, struct snd_soc_jack *jack,
			       struct snd_soc_jack_pin *pins,
			       unsigned int num_pins);

int snd_soc_card_suspend_pre(struct snd_soc_card *card);
int snd_soc_card_suspend_post(struct snd_soc_card *card);
int snd_soc_card_resume_pre(struct snd_soc_card *card);
int snd_soc_card_resume_post(struct snd_soc_card *card);

int snd_soc_card_probe(struct snd_soc_card *card);
int snd_soc_card_late_probe(struct snd_soc_card *card);
void snd_soc_card_fixup_controls(struct snd_soc_card *card);
int snd_soc_card_remove(struct snd_soc_card *card);

void snd_soc_card_set_topology_name(struct snd_soc_card *card, const char *preifx);

int snd_soc_card_set_bias_level(struct snd_soc_card *card,
				struct snd_soc_dapm_context *dapm,
				enum snd_soc_bias_level level);
int snd_soc_card_set_bias_level_post(struct snd_soc_card *card,
				     struct snd_soc_dapm_context *dapm,
				     enum snd_soc_bias_level level);

int snd_soc_card_add_dai_link(struct snd_soc_card *card,
			      struct snd_soc_dai_link *dai_link);
void snd_soc_card_remove_dai_link(struct snd_soc_card *card,
				  struct snd_soc_dai_link *dai_link);

int snd_soc_card_of_parse_simple_widgets(struct snd_soc_card *card, const char *propname);
int snd_soc_card_of_parse_name(struct snd_soc_card *card, const char *propname);
int snd_soc_card_of_parse_pin_switches(struct snd_soc_card *card, const char *propname);
int snd_soc_card_of_parse_audio_routing(struct snd_soc_card *card, const char *propname);
int snd_soc_card_of_parse_aux_devs(struct snd_soc_card *card, const char *propname);
int snd_soc_card_of_parse_ignore_suspend_widgets(struct snd_soc_card *card, const char *propname);

int snd_soc_card_fixup_dai_links_platform_name(struct snd_soc_card *card,
					       const char *platform_name);

#ifdef CONFIG_PCI
void snd_soc_card_set_pci_ssid(struct snd_soc_card *card,
			       unsigned short vendor,
			       unsigned short device);
int snd_soc_card_get_pci_ssid(struct snd_soc_card *card,
			      unsigned short *vendor,
			      unsigned short *device);
#else /* !CONFIG_PCI */
static inline void snd_soc_card_set_pci_ssid(struct snd_soc_card *card,
					     unsigned short vendor,
					     unsigned short device)
{
}

static inline int snd_soc_card_get_pci_ssid(struct snd_soc_card *card,
					    unsigned short *vendor,
					    unsigned short *device)
{
	return -ENOENT;
}
#endif /* CONFIG_PCI */

struct snd_soc_dai *snd_soc_card_get_codec_dai(struct snd_soc_card *card,
					       const char *dai_name);

void snd_soc_card_dapm_mutex_lock_root(struct snd_soc_card *card);
void snd_soc_card_dapm_mutex_lock(struct snd_soc_card *card);
void snd_soc_card_dapm_mutex_unlock(struct snd_soc_card *card);
void snd_soc_card_dapm_mutex_assert_held(struct snd_soc_card *card);

int snd_soc_card_is_instantiated(struct snd_soc_card *card);
struct snd_soc_dapm_context *snd_soc_card_to_dapm(struct snd_soc_card *card);
void *snd_soc_card_to_priv(struct snd_soc_card *card);
void snd_soc_card_set_priv(struct snd_soc_card *card, void *data);
struct snd_soc_pcm_runtime *snd_soc_card_to_rtd(struct snd_soc_card *card,
						 struct snd_soc_dai_link *dai_link);

#define for_each_card_prelinks(card, i, link)				\
	for ((i) = 0;							\
	     ((i) < (card)->num_links) && ((link) = &(card)->dai_link[i]); \
	     (i)++)
#define for_each_card_pre_auxs(card, i, aux)				\
	for ((i) = 0;							\
	     ((i) < (card)->num_aux_devs) && ((aux) = &(card)->aux_dev[i]); \
	     (i)++)

#define for_each_card_rtds(card, rtd)					\
	list_for_each_entry(rtd, &(card)->rtd_list_head, rtd_list)
#define for_each_card_rtds_safe(card, rtd, _rtd)			\
	list_for_each_entry_safe(rtd, _rtd, &(card)->rtd_list_head, rtd_list)

#define for_each_card_auxs(card, component)				\
	list_for_each_entry(component, &card->aux_list_head, aux_list)
#define for_each_card_auxs_safe(card, component, _comp)			\
	list_for_each_entry_safe(component, _comp,			\
				 &card->aux_list_head, aux_list)

#define for_each_card_components(card, component)			\
	list_for_each_entry(component, &(card)->component_list_head, component_list)

#define for_each_card_dapms(card, dapm)					\
	list_for_each_entry(dapm, &card->dapm_list_head, dapm_list)

#define for_each_card_widgets(card, w)					\
	list_for_each_entry(w, &card->widget_list_head, widget_list)
#define for_each_card_widgets_safe(card, w, _w)				\
	list_for_each_entry_safe(w, _w, &card->widget_list_head, widget_list)

/* REMOVE ME */
#define snd_soc_card_set_drvdata	snd_soc_card_set_priv
#define snd_soc_card_get_drvdata	snd_soc_card_to_priv
#define snd_soc_get_pcm_runtime		snd_soc_card_to_rtd

#endif /* __SOC_CARD_H */
