/* SPDX-License-Identifier: GPL-2.0
 *
 * soc-card.h
 *
 * Copyright (C) 2019 Renesas Electronics Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */
#ifndef __SOC_CARD_H
#define __SOC_CARD_H

struct snd_soc_card_driver {
	const char *default_name;
	const char *default_long_name;
	const char *default_components;
	const char *driver_name;

	struct module *owner;

	int (*probe)(struct snd_soc_card *card);
	int (*late_probe)(struct snd_soc_card *card);
	void (*fixup_controls)(struct snd_soc_card *card);
	int (*remove)(struct snd_soc_card *card);

	/* the pre and post PM functions are used to do any PM work before and
	 * after the codec and DAI's do any PM work. */
	int (*suspend_pre)(struct snd_soc_card *card);
	int (*suspend_post)(struct snd_soc_card *card);
	int (*resume_pre)(struct snd_soc_card *card);
	int (*resume_post)(struct snd_soc_card *card);

	/* callbacks */
	int (*set_bias_level)(struct snd_soc_card *,
			      struct snd_soc_dapm_context *dapm,
			      enum snd_soc_bias_level level);
	int (*set_bias_level_post)(struct snd_soc_card *,
				   struct snd_soc_dapm_context *dapm,
				   enum snd_soc_bias_level level);

	int (*add_dai_link)(struct snd_soc_card *,
			    struct snd_soc_dai_link *link);
	void (*remove_dai_link)(struct snd_soc_card *,
				struct snd_soc_dai_link *link);

	/* CPU <--> Codec DAI links  */
	struct snd_soc_dai_link *dai_link;	/* predefined links only */
	int num_links;				/* predefined links only */

	/* optional codec specific configuration */
	struct snd_soc_codec_conf *codec_conf;
	int num_configs;

	/*
	 * optional auxiliary devices such as amplifiers or codecs with DAI
	 * link unused
	 */
	struct snd_soc_aux_dev *aux_dev;
	int num_aux_devs;

	const struct snd_kcontrol_new *controls;
	int num_controls;

	/*
	 * Card-specific routes and widgets.
	 * Note: of_dapm_xxx for Device Tree; Otherwise for driver build-in.
	 */
	const struct snd_soc_dapm_widget *dapm_widgets;
	int num_dapm_widgets;
	const struct snd_soc_dapm_route *dapm_routes;
	int num_dapm_routes;
	const char **ignore_suspend_widgets;
	int num_ignore_suspend_widgets;
	const struct snd_soc_dapm_widget *of_dapm_widgets;
	int num_of_dapm_widgets;
	const struct snd_soc_dapm_route *of_dapm_routes;
	int num_of_dapm_routes;
	const char **of_ignore_suspend_widgets;
	int num_of_ignore_suspend_widgets;

	unsigned int fully_routed:1;
	unsigned int component_chaining:1;
};

enum snd_soc_card_subclass {
	SND_SOC_CARD_CLASS_ROOT		= 0,
	SND_SOC_CARD_CLASS_RUNTIME	= 1,
};

const char *snd_soc_card_name(struct snd_soc_card *card);
const char *snd_soc_card_long_name(struct snd_soc_card *card);
const char *snd_soc_card_components(struct snd_soc_card *card);
void snd_soc_card_set_name(struct snd_soc_card *card, const char *name);
void snd_soc_card_set_long_name(struct snd_soc_card *card, const char *long_name);
void snd_soc_card_set_components(struct snd_soc_card *card, const char *components);

struct snd_soc_card *snd_soc_card_alloc(struct device *dev);
int snd_soc_card_register_c(struct snd_soc_card *card, struct snd_soc_card_driver *driver);
int snd_soc_card_register_d(struct device *dev, struct snd_soc_card_driver *driver);
#define snd_soc_card_register(x, ...) _Generic((x),	\
struct snd_soc_card * :	snd_soc_card_register_c,	\
struct device * :	snd_soc_card_register_d)(x, __VA_ARGS__)
int devm_snd_soc_card_register_c(struct snd_soc_card *card, struct snd_soc_card_driver *driver);
int devm_snd_soc_card_register_d(struct device *dev, struct snd_soc_card_driver *driver);
#define devm_snd_soc_card_register(x, ...) _Generic((x),			\
struct snd_soc_card * :	devm_snd_soc_card_register_c, \
struct device * :	devm_snd_soc_card_register_d)(x, __VA_ARGS__)
void snd_soc_card_unregister(struct snd_soc_card *card);

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

int snd_soc_card_to_num_rtd(struct snd_soc_card *card);
struct device *snd_soc_card_to_dev(struct snd_soc_card *card);
struct snd_card *snd_soc_card_to_snd_card(struct snd_soc_card *card);

int snd_soc_card_is_instantiated(struct snd_soc_card *card);
struct snd_soc_dapm_context *snd_soc_card_to_dapm(struct snd_soc_card *card);
struct snd_soc_dapm_stats *snd_soc_card_to_dapm_stats(struct snd_soc_card *card);
void *snd_soc_card_to_priv(struct snd_soc_card *card);
void snd_soc_card_set_priv(struct snd_soc_card *card, void *data);
struct snd_soc_pcm_runtime *snd_soc_card_to_rtd(struct snd_soc_card *card,
						 struct snd_soc_dai_link *dai_link);

#define SOC_CARD_LIST_DEFINE(member) \
struct list_head* snd_soc_card_to_##member##_list(struct snd_soc_card *card); \
struct snd_soc_card *snd_soc_card_from_##member##_list(struct list_head *list)

#define SOC_CARD_LIST_HEAD_DEFINE(member) \
struct list_head* snd_soc_card_to_##member##_list_head(struct snd_soc_card *card)

/* see with SOC_CARD_LIST_ENTRY() in soc-component.c */
SOC_CARD_LIST_DEFINE(unbind);

SOC_CARD_LIST_HEAD_DEFINE(rtd);
SOC_CARD_LIST_HEAD_DEFINE(aux);
SOC_CARD_LIST_HEAD_DEFINE(component);
SOC_CARD_LIST_HEAD_DEFINE(widget);
SOC_CARD_LIST_HEAD_DEFINE(path);
SOC_CARD_LIST_HEAD_DEFINE(dapm);
SOC_CARD_LIST_HEAD_DEFINE(dapm_dirty);

#define for_each_card_driver_prelinks(card_driver, i, link)		\
	for ((i) = 0;							\
	     ((i) < (card_driver)->num_links) && ((link) = &(card_driver)->dai_link[i]); \
	     (i)++)
#define for_each_card_driver_pre_auxs(card_driver, i, aux)		\
	for ((i) = 0;							\
	     ((i) < (card_driver)->num_aux_devs) && ((aux) = &(card_driver)->aux_dev[i]); \
	     (i)++)

/* REMOVE ME */
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

int devm_snd_soc_register_card(struct device *dev, struct snd_soc_card *card);

#endif /* __SOC_CARD_H */
