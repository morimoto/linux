/* SPDX-License-Identifier: GPL-2.0
 *
 * soc-component.h
 *
 * Copyright (C) 2019 Renesas Electronics Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */
#ifndef __SOC_COMPONENT_H
#define __SOC_COMPONENT_H

#include <sound/soc.h>

struct device_link;

/*
 * Component probe and remove ordering levels for components with runtime
 * dependencies.
 */
#define SND_SOC_COMP_ORDER_FIRST	-2
#define SND_SOC_COMP_ORDER_EARLY	-1
#define SND_SOC_COMP_ORDER_NORMAL	 0
#define SND_SOC_COMP_ORDER_LATE		 1
#define SND_SOC_COMP_ORDER_LAST		 2

/* component interface */
struct snd_compress_ops {
	int (*open)(struct snd_soc_component *component,
		    struct snd_compr_stream *stream);
	int (*free)(struct snd_soc_component *component,
		    struct snd_compr_stream *stream);
	int (*set_params)(struct snd_soc_component *component,
			  struct snd_compr_stream *stream,
			  struct snd_compr_params *params);
	int (*get_params)(struct snd_soc_component *component,
			  struct snd_compr_stream *stream,
			  struct snd_codec *params);
	int (*set_metadata)(struct snd_soc_component *component,
			    struct snd_compr_stream *stream,
			    struct snd_compr_metadata *metadata);
	int (*get_metadata)(struct snd_soc_component *component,
			    struct snd_compr_stream *stream,
			    struct snd_compr_metadata *metadata);
	int (*trigger)(struct snd_soc_component *component,
		       struct snd_compr_stream *stream, int cmd);
	int (*pointer)(struct snd_soc_component *component,
		       struct snd_compr_stream *stream,
		       struct snd_compr_tstamp64 *tstamp);
	int (*copy)(struct snd_soc_component *component,
		    struct snd_compr_stream *stream, char __user *buf,
		    size_t count);
	int (*mmap)(struct snd_soc_component *component,
		    struct snd_compr_stream *stream,
		    struct vm_area_struct *vma);
	int (*ack)(struct snd_soc_component *component,
		   struct snd_compr_stream *stream, size_t bytes);
	int (*get_caps)(struct snd_soc_component *component,
			struct snd_compr_stream *stream,
			struct snd_compr_caps *caps);
	int (*get_codec_caps)(struct snd_soc_component *component,
			      struct snd_compr_stream *stream,
			      struct snd_compr_codec_caps *codec);
};

struct snd_soc_component_driver {
	const char *name;

	/* Default control and setup, added after probe() is run */
	const struct snd_kcontrol_new *controls;
	unsigned int num_controls;
	const struct snd_soc_dapm_widget *dapm_widgets;
	unsigned int num_dapm_widgets;
	const struct snd_soc_dapm_route *dapm_routes;
	unsigned int num_dapm_routes;

	int (*probe)(struct snd_soc_component *component);
	int (*fixup_controls)(struct snd_soc_component *component);
	void (*remove)(struct snd_soc_component *component);
	int (*suspend)(struct snd_soc_component *component);
	int (*resume)(struct snd_soc_component *component);

	unsigned int (*read)(struct snd_soc_component *component,
			     unsigned int reg);
	int (*write)(struct snd_soc_component *component,
		     unsigned int reg, unsigned int val);

	/* pcm creation and destruction */
	int (*pcm_new)(struct snd_soc_component *component,
		       struct snd_soc_pcm_runtime *rtd);
	void (*pcm_free)(struct snd_soc_component *component,
			 struct snd_pcm *pcm);

	/* component wide operations */
	int (*set_sysclk)(struct snd_soc_component *component,
			  int clk_id, int source, unsigned int freq, int dir);
	int (*set_pll)(struct snd_soc_component *component, int pll_id,
		       int source, unsigned int freq_in, unsigned int freq_out);
	int (*set_jack)(struct snd_soc_component *component,
			struct snd_soc_jack *jack,  void *data);
	int (*get_jack_type)(struct snd_soc_component *component);

	/* DT */
	int (*of_xlate_dai_name)(struct snd_soc_component *component,
				 const struct of_phandle_args *args,
				 const char **dai_name);
	int (*of_xlate_dai_id)(struct snd_soc_component *comment,
			       struct device_node *endpoint);
	void (*seq_notifier)(struct snd_soc_component *component,
			     enum snd_soc_dapm_type type, int subseq);
	int (*stream_event)(struct snd_soc_component *component, int event);
	int (*set_bias_level)(struct snd_soc_component *component,
			      enum snd_soc_bias_level level);

	int (*open)(struct snd_soc_component *component,
		    struct snd_pcm_substream *substream);
	int (*close)(struct snd_soc_component *component,
		     struct snd_pcm_substream *substream);
	int (*ioctl)(struct snd_soc_component *component,
		     struct snd_pcm_substream *substream,
		     unsigned int cmd, void *arg);
	int (*hw_params)(struct snd_soc_component *component,
			 struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params);
	int (*hw_free)(struct snd_soc_component *component,
		       struct snd_pcm_substream *substream);
	int (*prepare)(struct snd_soc_component *component,
		       struct snd_pcm_substream *substream);
	int (*trigger)(struct snd_soc_component *component,
		       struct snd_pcm_substream *substream, int cmd);
	int (*sync_stop)(struct snd_soc_component *component,
			 struct snd_pcm_substream *substream);
	snd_pcm_uframes_t (*pointer)(struct snd_soc_component *component,
				     struct snd_pcm_substream *substream);
	int (*get_time_info)(struct snd_soc_component *component,
		struct snd_pcm_substream *substream, struct timespec64 *system_ts,
		struct timespec64 *audio_ts,
		struct snd_pcm_audio_tstamp_config *audio_tstamp_config,
		struct snd_pcm_audio_tstamp_report *audio_tstamp_report);
	int (*copy)(struct snd_soc_component *component,
		    struct snd_pcm_substream *substream, int channel,
		    unsigned long pos, struct iov_iter *iter,
		    unsigned long bytes);
	struct page *(*page)(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream,
			     unsigned long offset);
	int (*mmap)(struct snd_soc_component *component,
		    struct snd_pcm_substream *substream,
		    struct vm_area_struct *vma);
	int (*ack)(struct snd_soc_component *component,
		   struct snd_pcm_substream *substream);
	snd_pcm_sframes_t (*delay)(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream);
	int (*be_hw_params_fixup)(struct snd_soc_pcm_runtime *rtd,
				  struct snd_pcm_hw_params *params);

	const struct snd_compress_ops *compress_ops;

	/* probe ordering - for components with runtime dependencies */
	int probe_order;
	int remove_order;

	/*
	 * soc_pcm_trigger() start/stop sequence.
	 * see also
	 *	snd_soc_dai_link
	 *	soc_pcm_trigger()
	 */
	enum snd_soc_trigger_order trigger_start;
	enum snd_soc_trigger_order trigger_stop;

	/*
	 * signal if the module handling the component should not be removed
	 * if a pcm is open. Setting this would prevent the module
	 * refcount being incremented in probe() but allow it be incremented
	 * when a pcm is opened and decremented when it is closed.
	 */
	unsigned int module_get_upon_open:1;

	/* bits */
	unsigned int idle_bias_on:1;
	unsigned int suspend_bias_off:1;
	unsigned int use_pmdown_time:1; /* care pmdown_time at stop */
	/*
	 * Indicates that the component does not care about the endianness of
	 * PCM audio data and the core will ensure that both LE and BE variants
	 * of each used format are present. Typically this is because the
	 * component sits behind a bus that abstracts away the endian of the
	 * original data, ie. one for which the transmission endian is defined
	 * (I2S/SLIMbus/SoundWire), or the concept of endian doesn't exist (PDM,
	 * analogue).
	 */
	unsigned int endianness:1;
	unsigned int legacy_dai_naming:1;

	/* use DAI link PCM ID as PCM device number */
	unsigned int use_dai_pcm_id:1;

	/* this component uses topology and ignore machine driver FEs */
	const char *ignore_machine;
	const char *topology_name_prefix;

	int be_pcm_base;	/* base device ID for all BE PCMs */

	const char *debugfs_prefix;
};

#define for_each_component_dais(component, dai)					\
	for (dai = snd_soc_dai_from_dai_list(snd_soc_component_to_dai_list_head(component)->next);\
	     snd_soc_dai_to_dai_list(dai) != snd_soc_component_to_dai_list_head(component);	\
	     dai = snd_soc_dai_from_dai_list(snd_soc_dai_to_dai_list(dai)->next))

#define for_each_component_dais_safe(component, dai, _dai)				\
	for (dai = snd_soc_dai_from_dai_list(snd_soc_component_to_dai_list_head(component)->next),\
	     _dai = snd_soc_dai_from_dai_list(snd_soc_dai_to_dai_list(dai)->next);		\
	     snd_soc_dai_to_dai_list(dai) != snd_soc_component_to_dai_list_head(component);	\
	     dai = _dai, _dai = snd_soc_dai_from_dai_list(snd_soc_dai_to_dai_list(_dai)->next))

int snd_soc_component_register_c(struct snd_soc_component *component,
				 const struct snd_soc_component_driver *component_driver,
				 struct snd_soc_dai_driver *dai_drv, int num_dai);
int snd_soc_component_register_d(struct device *dev,
				 const struct snd_soc_component_driver *component_driver,
				 struct snd_soc_dai_driver *dai_drv, int num_dai);
#define snd_soc_component_register(x, ...) _Generic((x),		\
struct device * :		snd_soc_component_register_d, \
struct snd_soc_component * :	snd_soc_component_register_c)(x, __VA_ARGS__)

int devm_snd_soc_component_register(struct device *dev,
				    const struct snd_soc_component_driver *component_driver,
				    struct snd_soc_dai_driver *dai_drv, int num_dai);
#define snd_soc_component_unregister(dev) snd_soc_component_unregister_by_driver(dev, NULL)
void snd_soc_component_unregister_by_driver(struct device *dev,
					    const struct snd_soc_component_driver *component_driver);

struct device *snd_soc_component_to_dev(struct snd_soc_component *component);
struct snd_soc_dapm_context *snd_soc_component_to_dapm(struct snd_soc_component *component);
const struct snd_soc_component_driver *snd_soc_component_to_driver(struct snd_soc_component *component);
struct snd_soc_card *snd_soc_component_to_card(struct snd_soc_component *component);
struct dentry *snd_soc_component_to_debugfs_root(struct snd_soc_component *component);
struct regmap *snd_soc_component_to_regmap(struct snd_soc_component *component);

struct snd_soc_component *snd_soc_component_alloc(struct device *dev);

void snd_soc_component_set_name(struct snd_soc_component *component, const char *name);
const char *snd_soc_component_name(struct snd_soc_component *component);
const char *snd_soc_component_name_prefix(struct snd_soc_component *component);

void snd_soc_component_set_priv(struct snd_soc_component *component, void *priv);
void *snd_soc_component_to_priv(struct snd_soc_component *component);

struct snd_soc_component *snd_soc_component_lookup_nolock(struct device *dev,
							  const char *driver_name);
struct snd_soc_component *snd_soc_component_lookup(struct device *dev, const char *driver_name);
struct snd_soc_component *snd_soc_component_lookup_by_name(const char *component_name);

#define SOC_COMPOENT_LIST_DEFINE(member)				\
struct list_head* snd_soc_component_to_##member##_list(struct snd_soc_component *component);\
struct snd_soc_component *snd_soc_component_from_##member##_list(struct list_head *list)

#define SOC_COMPOENT_LIST_HEAD_DEFINE(member)				\
struct list_head* snd_soc_component_to_##member##_list_head(struct snd_soc_component *component)

/* see with SOC_COMPOENT_LIST_ENTRY() in soc-component.c */
SOC_COMPOENT_LIST_DEFINE(component_total);
SOC_COMPOENT_LIST_DEFINE(component);
SOC_COMPOENT_LIST_DEFINE(aux);
SOC_COMPOENT_LIST_HEAD_DEFINE(dobj);
SOC_COMPOENT_LIST_HEAD_DEFINE(dai);

/* component IO */
unsigned int snd_soc_component_read(struct snd_soc_component *component,
				      unsigned int reg);
int snd_soc_component_write(struct snd_soc_component *component,
			    unsigned int reg, unsigned int val);
int snd_soc_component_update_bits(struct snd_soc_component *component,
				  unsigned int reg, unsigned int mask,
				  unsigned int val);
int snd_soc_component_update_bits_async(struct snd_soc_component *component,
					unsigned int reg, unsigned int mask,
					unsigned int val);
int snd_soc_component_test_bits(struct snd_soc_component *component,
				unsigned int reg, unsigned int mask,
				unsigned int value);

unsigned int snd_soc_component_read_field(struct snd_soc_component *component,
					  unsigned int reg, unsigned int mask);
int snd_soc_component_write_field(struct snd_soc_component *component,
				  unsigned int reg, unsigned int mask,
				  unsigned int val);

/* component wide operations */
int snd_soc_component_set_sysclk(struct snd_soc_component *component,
				 int clk_id, int source,
				 unsigned int freq, int dir);
int snd_soc_component_set_pll(struct snd_soc_component *component, int pll_id,
			      int source, unsigned int freq_in,
			      unsigned int freq_out);
int snd_soc_component_set_jack(struct snd_soc_component *component,
			       struct snd_soc_jack *jack, void *data);
int snd_soc_component_get_jack_type(struct snd_soc_component *component);

void snd_soc_component_seq_notifier(struct snd_soc_component *component,
				    enum snd_soc_dapm_type type, int subseq);
int snd_soc_component_stream_event(struct snd_soc_component *component,
				   int event);
int snd_soc_component_set_bias_level(struct snd_soc_component *component,
				     enum snd_soc_bias_level level);

int snd_soc_component_fixup_controls(struct snd_soc_component *component);
int snd_soc_component_add_controls(struct snd_soc_component *component,
				const struct snd_kcontrol_new *controls, unsigned int num_controls);

unsigned int snd_soc_component_active(struct snd_soc_component *component);
int snd_soc_component_num_dai(struct snd_soc_component *component);

/* component controls */
struct snd_kcontrol *snd_soc_component_get_kcontrol(struct snd_soc_component *component,
						    const char * const ctl);
int snd_soc_component_notify_control(struct snd_soc_component *component,
				     const char * const ctl);

/* component driver ops */
int snd_soc_pcm_component_pointer(struct snd_pcm_substream *substream);
int snd_soc_pcm_component_ioctl(struct snd_pcm_substream *substream,
				unsigned int cmd, void *arg);
int snd_soc_pcm_component_sync_stop(struct snd_pcm_substream *substream);
int snd_soc_pcm_component_copy(struct snd_pcm_substream *substream,
			       int channel, unsigned long pos,
			       struct iov_iter *iter, unsigned long bytes);
struct page *snd_soc_pcm_component_page(struct snd_pcm_substream *substream,
					unsigned long offset);
int snd_soc_pcm_component_mmap(struct snd_pcm_substream *substream,
			       struct vm_area_struct *vma);
int snd_soc_pcm_component_new(struct snd_soc_pcm_runtime *rtd);
void snd_soc_pcm_component_free(struct snd_soc_pcm_runtime *rtd);
int snd_soc_pcm_component_prepare(struct snd_pcm_substream *substream);
int snd_soc_pcm_component_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params);
void snd_soc_pcm_component_hw_free(struct snd_pcm_substream *substream,
				   int rollback);
int snd_soc_pcm_component_trigger(struct snd_pcm_substream *substream,
				  int cmd, int rollback);
int snd_soc_pcm_component_pm_runtime_get(struct snd_soc_pcm_runtime *rtd,
					 void *stream);
void snd_soc_pcm_component_pm_runtime_put(struct snd_soc_pcm_runtime *rtd,
					  void *stream, int rollback);
int snd_soc_pcm_component_ack(struct snd_pcm_substream *substream);
void snd_soc_pcm_component_delay(struct snd_pcm_substream *substream,
				 snd_pcm_sframes_t *cpu_delay, snd_pcm_sframes_t *codec_delay);

/*
 * regmap
 */
void snd_soc_component_regmap_init(struct snd_soc_component *component,
				   struct regmap *regmap);
void snd_soc_component_regmap_exit(struct snd_soc_component *component);

static inline int snd_soc_component_regmap_get_val_bytes(struct snd_soc_component *component)
{
	struct regmap *regmap = snd_soc_component_to_regmap(component);
	int val_bytes;

	/* Errors are legitimate for non-integer byte multiples */

	if (!regmap)
		return 0;

	val_bytes = regmap_get_val_bytes(regmap);
	if (val_bytes < 0)
		return 0;

	return val_bytes;
}

static inline int snd_soc_component_regcache_sync(struct snd_soc_component *component)
{
	struct regmap *regmap = snd_soc_component_to_regmap(component);

	if (!regmap)
		return 0;

	return regcache_sync(regmap);
}

static inline int snd_soc_component_regmap_multi_reg_write_bypassed(struct snd_soc_component *component,
								    const struct reg_sequence *regs,
								    int num_regs)
{
	struct regmap *regmap = snd_soc_component_to_regmap(component);

	if (!regmap)
		return 0;

	return regmap_multi_reg_write_bypassed(regmap, regs, num_regs);
}

static inline void snd_soc_component_regmap_async_complete(struct snd_soc_component *component)
{
	struct regmap *regmap = snd_soc_component_to_regmap(component);

	if (regmap)
		regmap_async_complete(regmap);
}

static inline void snd_soc_component_regcache_mark_dirty(struct snd_soc_component *component)
{
	struct regmap *regmap = snd_soc_component_to_regmap(component);

	if (regmap)
		regcache_mark_dirty(regmap);
}

static inline void snd_soc_component_regcache_cache_only(struct snd_soc_component *component, bool enable)
{
	struct regmap *regmap = snd_soc_component_to_regmap(component);

	if (regmap)
		regcache_cache_only(regmap, enable);
}

static inline void snd_soc_component_regcache_cache_bypass(struct snd_soc_component *component, bool enable)
{
	struct regmap *regmap = snd_soc_component_to_regmap(component);

	if (regmap)
		regcache_cache_bypass(regmap, enable);
}

#endif /* __SOC_COMPONENT_H */
