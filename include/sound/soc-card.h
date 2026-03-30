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

/* REMOVE ME */
#define snd_soc_card_set_drvdata	snd_soc_card_set_priv
#define snd_soc_card_get_drvdata	snd_soc_card_to_priv

#endif /* __SOC_CARD_H */
