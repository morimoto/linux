// SPDX-License-Identifier: GPL-2.0+
//
// soc-pcm.c  --  ALSA SoC PCM
//
// Copyright 2005 Wolfson Microelectronics PLC.
// Copyright 2005 Openedhand Ltd.
// Copyright (C) 2010 Slimlogic Ltd.
// Copyright (C) 2010 Texas Instruments Inc.
//
// Authors: Liam Girdwood <lrg@ti.com>
//          Mark Brown <broonie@opensource.wolfsonmicro.com>

#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <sound/soc.h>
#include "soc-internal.h"


DEFINE_GUARD(snd_soc_card_mutex, struct snd_soc_card *,
	     snd_soc_card_mutex_lock(_T), snd_soc_card_mutex_unlock(_T))
#define soc_pcm_ret(rtd, ret) snd_soc_ret(rtd->dev, ret, "on %s\n", rtd->dai_link->name)


static inline const char *soc_cpu_dai_name(struct snd_soc_pcm_runtime *rtd)
{
	return (rtd)->dai_link->num_cpus == 1 ? snd_soc_dai_name(snd_soc_rtd_to_cpu(rtd, 0)) : "multicpu";
}
static inline const char *soc_codec_dai_name(struct snd_soc_pcm_runtime *rtd)
{
	return (rtd)->dai_link->num_codecs == 1 ? snd_soc_dai_name(snd_soc_rtd_to_codec(rtd, 0)) : "multicodec";
}

/**
 * snd_soc_runtime_action() - Increment/Decrement active count for
 * PCM runtime components
 * @rtd: ASoC PCM runtime that is activated
 * @stream: Direction of the PCM stream
 * @action: Activate stream if 1. Deactivate if -1.
 *
 * Increments/Decrements the active count for all the DAIs and components
 * attached to a PCM runtime.
 * Should typically be called when a stream is opened.
 *
 * Must be called with the rtd->card->pcm_mutex being held
 */
void snd_soc_runtime_action(struct snd_soc_pcm_runtime *rtd,
			    int stream, int action)
{
	struct snd_soc_component *component;
	struct snd_soc_dai *dai;
	int i;

	snd_soc_dpcm_mutex_assert_held(rtd);

	for_each_rtd_dais(rtd, i, dai)
		snd_soc_dai_active_update(dai, stream, action);

	/* Increments/Decrements the active count for components without DAIs */
	for_each_rtd_components(rtd, i, component) {
		if (snd_soc_component_num_dai(component))
			continue;
		snd_soc_component_active_action(component, action);
	}
}
EXPORT_SYMBOL_GPL(snd_soc_runtime_action);

/**
 * snd_soc_runtime_ignore_pmdown_time() - Check whether to ignore the power down delay
 * @rtd: The ASoC PCM runtime that should be checked.
 *
 * This function checks whether the power down delay should be ignored for a
 * specific PCM runtime. Returns true if the delay is 0, if the DAI link has
 * been configured to ignore the delay, or if none of the components benefits
 * from having the delay.
 */
bool snd_soc_runtime_ignore_pmdown_time(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component;
	int i;

	if (!rtd->pmdown_time || rtd->dai_link->ignore_pmdown_time)
		return true;

	for_each_rtd_components(rtd, i, component) {
		const struct snd_soc_component_driver *driver = snd_soc_component_to_driver(component);

		if (driver->use_pmdown_time)
			/* No need to go through all components */
			return false;
	}

	return true;
}

/*
 * Shared BCLK constraint: when multiple DAIs share the same physical BCLK,
 * constrain hw_params so that the BCLK rate (rate * channels * sample_bits,
 * or rate * slots * slot_width for TDM) remains consistent.
 */

static int soc_pcm_shared_bclk_rule_rate(struct snd_pcm_hw_params *params,
					 struct snd_pcm_hw_rule *rule)
{
	struct snd_soc_dai *dai = rule->private;
	struct snd_soc_component *component = snd_soc_dai_to_component(dai);
	struct device *dev = snd_soc_component_to_dev(component);
	struct snd_soc_card *card = snd_soc_component_to_card(component);
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_dai *other_dai;
	unsigned long active_bclk_rate = 0;
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval constraint = { .empty = 1 };
	unsigned int target_rate;
	struct clk *dai_bclk;
	unsigned int dai_bclk_ratio;
	int i;

	/* Protect the rtd list traversal with the ASoC card mutex helper. */
	guard(snd_soc_card_mutex)(card);

	dai_bclk = snd_soc_dai_get_bclk(dai);
	dai_bclk_ratio = snd_soc_dai_get_bclk_ratio(dai);

	/* Scan all DAIs on the card for an active peer sharing the same BCLK */
	for_each_card_rtds(card, rtd) {
		for_each_rtd_cpu_dais(rtd, i, other_dai) {
			struct clk *other_dai_bclk = snd_soc_dai_get_bclk(other_dai);
			unsigned int other_dai_symmetric_rate = snd_soc_dai_get_symmetric_rate(other_dai);

			if (other_dai == dai)
				continue;
			if (!other_dai_bclk)
				continue;
			if (!snd_soc_dai_active(other_dai))
				continue;
			/*
			 * Skip peers whose hw_params hasn't run yet.
			 * symmetric_rate is set by soc_pcm_set_dai_params()
			 * after snd_soc_dai_hw_params(), so non-zero means
			 * the DAI's clk_set_rate() has already executed.
			 */
			if (!other_dai_symmetric_rate)
				continue;
			if (!clk_is_match(dai_bclk, other_dai_bclk))
				continue;

			active_bclk_rate = clk_get_rate(other_dai_bclk);
			if (active_bclk_rate)
				goto found;
		}
	}

	return 0;

found:
	if (dai_bclk_ratio) {
		/*
		 * Driver has set an explicit BCLK ratio (e.g. for TDM where
		 * BCLK = rate * slots * slot_width). The only valid rate is
		 * active_bclk_rate / bclk_ratio.
		 */
		target_rate = active_bclk_rate / dai_bclk_ratio;

		constraint.min = target_rate;
		constraint.max = target_rate;
	} else {
		struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
		struct snd_interval *sample_bits = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_SAMPLE_BITS);

		/*
		 * Default: BCLK = rate * channels * sample_bits.
		 * Calculate the range of valid rates given the current
		 * channel and sample_bits intervals.
		 */
		if (!channels->min || !sample_bits->min)
			return 0;

		constraint.max = active_bclk_rate /
				 ((unsigned long)channels->min * sample_bits->min);

		if (channels->max && sample_bits->max)
			constraint.min = active_bclk_rate /
					 ((unsigned long)channels->max * sample_bits->max);
		else
			constraint.min = constraint.max;
	}

	constraint.integer = 1;
	constraint.empty = 0;

	return snd_soc_ret(dev, snd_interval_refine(rate, &constraint), "\n");
}

static int soc_pcm_apply_shared_bclk(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_component *component = snd_soc_dai_to_component(dai);
	struct device *dev = snd_soc_component_to_dev(component);

	if (!snd_soc_dai_get_bclk(dai))
		return 0;

	dev_dbg(dev, "ASoC: registering shared BCLK rate constraint\n");

	return soc_pcm_ret(rtd,
			snd_pcm_hw_rule_add(substream->runtime, 0,
					SNDRV_PCM_HW_PARAM_RATE,
					soc_pcm_shared_bclk_rule_rate, dai,
					SNDRV_PCM_HW_PARAM_CHANNELS,
					SNDRV_PCM_HW_PARAM_SAMPLE_BITS, -1));
}

static void soc_pcm_set_msb(struct snd_pcm_substream *substream, int bits)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	int ret;

	if (!bits)
		return;

	ret = snd_pcm_hw_constraint_msbits(substream->runtime, 0, 0, bits);
	if (ret != 0)
		dev_warn(rtd->dev, "ASoC: Failed to set MSB %d: %d\n",
				 bits, ret);
}

static void soc_pcm_apply_msb(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	int stream = substream->stream;
	int i;
	unsigned int bits = 0, cpu_bits = 0;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		const struct snd_soc_pcm_stream *pcm_codec = snd_soc_dai_pcm_stream_get(codec_dai, stream);

		if (pcm_codec->sig_bits == 0) {
			bits = 0;
			break;
		}
		bits = max(pcm_codec->sig_bits, bits);
	}

	for_each_rtd_cpu_dais(rtd, i, cpu_dai) {
		const struct snd_soc_pcm_stream *pcm_cpu = snd_soc_dai_pcm_stream_get(cpu_dai, stream);

		if (pcm_cpu->sig_bits == 0) {
			cpu_bits = 0;
			break;
		}
		cpu_bits = max(pcm_cpu->sig_bits, cpu_bits);
	}

	soc_pcm_set_msb(substream, bits);
	soc_pcm_set_msb(substream, cpu_bits);
}

void snd_soc_pcm_hw_init(struct snd_pcm_hardware *hw, bool force)
{
	if (force) {
		hw->rates = UINT_MAX;
		hw->rate_min = 0;
		hw->rate_max = UINT_MAX;
		hw->channels_min = 0;
		hw->channels_max = UINT_MAX;
		hw->formats = ULLONG_MAX;
	} else {
		/* Preserve initialized parameters */
		if (!hw->rates)
			hw->rates = UINT_MAX;
		if (!hw->rate_max)
			hw->rate_max = UINT_MAX;
		if (!hw->channels_max)
			hw->channels_max = UINT_MAX;
		if (!hw->formats)
			hw->formats = ULLONG_MAX;
	}
}

void snd_soc_pcm_hw_update_rate(struct snd_pcm_hardware *hw, const struct snd_soc_pcm_stream *p)
{
	hw->rates = snd_pcm_rate_mask_intersect(hw->rates, p->rates);

	/* setup hw->rate_min/max via hw->rates first */
	snd_pcm_hw_limit_rates(hw);

	/* update hw->rate_min/max by snd_soc_pcm_stream */
	hw->rate_min = max(hw->rate_min, p->rate_min);
	hw->rate_max = min_not_zero(hw->rate_max, p->rate_max);
}

void snd_soc_pcm_hw_update_chan(struct snd_pcm_hardware *hw, const struct snd_soc_pcm_stream *p)
{
	hw->channels_min = max(hw->channels_min, p->channels_min);
	hw->channels_max = min(hw->channels_max, p->channels_max);
}

void snd_soc_pcm_hw_update_format(struct snd_pcm_hardware *hw, const struct snd_soc_pcm_stream *p)
{
	hw->formats	&= p->formats;
	hw->subformats	&= p->subformats;
}

/**
 * snd_soc_runtime_calc_hw() - Calculate hw limits for a PCM stream
 * @rtd: ASoC PCM runtime
 * @hw: PCM hardware parameters (output)
 * @stream: Direction of the PCM stream
 *
 * Calculates the subset of stream parameters supported by all DAIs
 * associated with the PCM stream.
 */
int snd_soc_runtime_calc_hw(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hardware *hw, int stream)
{
	struct snd_soc_dai *codec_dai;
	struct snd_soc_dai *cpu_dai;
	const struct snd_soc_pcm_stream *codec_stream;
	const struct snd_soc_pcm_stream *cpu_stream;
	unsigned int cpu_chan_min = 0, cpu_chan_max = UINT_MAX;
	int i;

	snd_soc_pcm_hw_init(hw, true);

	/* first calculate min/max only for CPUs in the DAI link */
	for_each_rtd_cpu_dais(rtd, i, cpu_dai) {

		/*
		 * Skip CPUs which don't support the current stream type.
		 * Otherwise, since the rate, channel, and format values will
		 * zero in that case, we would have no usable settings left,
		 * causing the resulting setup to fail.
		 */
		if (!snd_soc_dai_stream_valid(cpu_dai, stream))
			continue;

		cpu_stream = snd_soc_dai_pcm_stream_get(cpu_dai, stream);

		snd_soc_pcm_hw_update_chan(hw, cpu_stream);
		snd_soc_pcm_hw_update_rate(hw, cpu_stream);
		snd_soc_pcm_hw_update_format(hw, cpu_stream);
	}
	cpu_chan_min = hw->channels_min;
	cpu_chan_max = hw->channels_max;

	/* second calculate min/max only for CODECs in the DAI link */
	for_each_rtd_codec_dais(rtd, i, codec_dai) {

		/*
		 * Skip CODECs which don't support the current stream type.
		 * Otherwise, since the rate, channel, and format values will
		 * zero in that case, we would have no usable settings left,
		 * causing the resulting setup to fail.
		 */
		if (!snd_soc_dai_stream_valid(codec_dai, stream))
			continue;

		codec_stream = snd_soc_dai_pcm_stream_get(codec_dai, stream);

		snd_soc_pcm_hw_update_chan(hw, codec_stream);
		snd_soc_pcm_hw_update_rate(hw, codec_stream);
		snd_soc_pcm_hw_update_format(hw, codec_stream);
	}

	/* Verify both a valid CPU DAI and a valid CODEC DAI were found */
	if (!hw->channels_min)
		return soc_pcm_ret(rtd, -EINVAL);

	/*
	 * chan min/max cannot be enforced if there are multiple CODEC DAIs
	 * connected to CPU DAI(s), use CPU DAI's directly and let
	 * channel allocation be fixed up later
	 */
	if (rtd->dai_link->num_codecs > 1) {
		hw->channels_min = cpu_chan_min;
		hw->channels_max = cpu_chan_max;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_runtime_calc_hw);

static void soc_pcm_init_runtime_hw(struct snd_pcm_substream *substream)
{
	struct snd_pcm_hardware *hw = &substream->runtime->hw;
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	u64 formats = hw->formats;

	/*
	 * At least one CPU and one CODEC should match. Otherwise, we should
	 * have bailed out on a higher level, since there would be no CPU or
	 * CODEC to support the transfer direction in that case.
	 */
	snd_soc_runtime_calc_hw(rtd, hw, substream->stream);

	if (formats)
		hw->formats &= formats;
}

static int soc_pcm_components_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_component *component;
	int i, ret = 0;

	for_each_rtd_components(rtd, i, component) {
		ret = snd_soc_component_module_get_when_open(component, substream);
		if (ret < 0)
			break;

		ret = snd_soc_component_open(component, substream);
		if (ret < 0)
			break;
	}

	return soc_pcm_ret(rtd, ret);
}

static int soc_pcm_components_close(struct snd_pcm_substream *substream,
				    int rollback)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_component *component;
	int i, ret = 0;

	for_each_rtd_components(rtd, i, component) {
		int r = snd_soc_component_close(component, substream, rollback);
		if (r < 0)
			ret = r; /* use last ret */

		snd_soc_component_module_put_when_close(component, substream, rollback);
	}

	return soc_pcm_ret(rtd, ret);
}

int snd_soc_pcm_close_no_lock(struct snd_soc_pcm_runtime *rtd,
			      struct snd_pcm_substream *substream, int rollback)
{
	struct snd_soc_component *component;
	struct snd_soc_dai *dai;
	int i;

	snd_soc_dpcm_mutex_assert_held(rtd);

	if (!rollback) {
		snd_soc_runtime_deactivate(rtd, substream->stream);

		/* Make sure DAI parameters cleared if the DAI becomes inactive */
		for_each_rtd_dais(rtd, i, dai) {
			if (snd_soc_dai_active(dai) == 0)
				snd_soc_dai_symmetric_set_params(dai, NULL);
		}
	}

	for_each_rtd_dais_reverse(rtd, i, dai)
		snd_soc_dai_shutdown(dai, substream, rollback);

	snd_soc_link_shutdown(substream, rollback);

	soc_pcm_components_close(substream, rollback);

	snd_soc_pcm_component_pm_runtime_put(rtd, substream, rollback);

	for_each_rtd_components(rtd, i, component)
		if (!snd_soc_component_active(component)) {
			struct device *dev = snd_soc_component_to_dev(component);

			pinctrl_pm_select_sleep_state(dev);
		}

	return 0;
}

/*
 * Called by ALSA when a PCM substream is closed. Private data can be
 * freed here. The cpu DAI, codec DAI, machine and components are also
 * shutdown.
 */
/* PCM close ops for non-DPCM streams */
static int soc_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);

	snd_soc_dpcm_mutex_lock(rtd);
	snd_soc_pcm_close_no_lock(rtd, substream, 0);
	snd_soc_dpcm_mutex_unlock(rtd);
	return 0;
}

static int soc_hw_sanity_check(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_pcm_hardware *hw = &substream->runtime->hw;
	const char *name_cpu = soc_cpu_dai_name(rtd);
	const char *name_codec = soc_codec_dai_name(rtd);
	const char *err_msg;
	struct device *dev = rtd->dev;

	err_msg = "rates";
	if (!hw->rates)
		goto config_err;

	err_msg = "formats";
	if (!hw->formats)
		goto config_err;

	err_msg = "channels";
	if (!hw->channels_min || !hw->channels_max ||
	     hw->channels_min  >  hw->channels_max)
		goto config_err;

	dev_dbg(dev, "ASoC: %s <-> %s info:\n",		name_codec,
							name_cpu);
	dev_dbg(dev, "ASoC: rate mask 0x%x\n",		hw->rates);
	dev_dbg(dev, "ASoC: ch   min %d max %d\n",	hw->channels_min,
							hw->channels_max);
	dev_dbg(dev, "ASoC: rate min %d max %d\n",	hw->rate_min,
							hw->rate_max);

	return 0;

config_err:
	return snd_soc_ret(dev, -EINVAL,
			"%s <-> %s No matching %s\n", name_codec, name_cpu, err_msg);
}

/*
 * Called by ALSA when a PCM substream is opened, the runtime->hw record is
 * then initialized and any private data can be allocated. This also calls
 * startup for the cpu DAI, component, machine and codec DAI.
 */
int snd_soc_pcm_open_no_lock(struct snd_soc_pcm_runtime *rtd,
			     struct snd_pcm_substream *substream)
{
	struct snd_soc_component *component;
	struct snd_soc_dai *dai;
	int i, ret = 0;

	snd_soc_dpcm_mutex_assert_held(rtd);

	for_each_rtd_components(rtd, i, component) {
		struct device *dev = snd_soc_component_to_dev(component);

		pinctrl_pm_select_default_state(dev);
	}

	ret = snd_soc_pcm_component_pm_runtime_get(rtd, substream);
	if (ret < 0)
		goto err;

	ret = soc_pcm_components_open(substream);
	if (ret < 0)
		goto err;

	ret = snd_soc_link_startup(substream);
	if (ret < 0)
		goto err;

	/* startup the audio subsystem */
	for_each_rtd_dais(rtd, i, dai) {
		ret = snd_soc_dai_startup(dai, substream);
		if (ret < 0)
			goto err;
	}

	/* Dynamic PCM DAI links compat checks use dynamic capabilities */
	if (rtd->dai_link->dynamic || rtd->dai_link->no_pcm)
		goto dynamic;

	/* Check that the codec and cpu DAIs are compatible */
	soc_pcm_init_runtime_hw(substream);

	snd_soc_dai_symmetric_update(substream);

	ret = soc_hw_sanity_check(substream);
	if (ret < 0)
		goto err;

	soc_pcm_apply_msb(substream);

	/* Symmetry only applies if we've already got an active stream. */
	for_each_rtd_dais(rtd, i, dai) {
		ret = snd_soc_dai_symmetric_apply(substream, dai);
		if (ret != 0)
			goto err;
	}

	/* Shared BCLK constraint across DAIs on the same card */
	for_each_rtd_cpu_dais(rtd, i, dai) {
		ret = soc_pcm_apply_shared_bclk(substream, dai);
		if (ret != 0)
			goto err;
	}
dynamic:
	snd_soc_runtime_activate(rtd, substream->stream);
	ret = 0;
err:
	if (ret < 0)
		snd_soc_pcm_close_no_lock(rtd, substream, 1);

	return soc_pcm_ret(rtd, ret);
}

/* PCM open ops for non-DPCM streams */
static int soc_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	int ret;

	snd_soc_dpcm_mutex_lock(rtd);
	ret = snd_soc_pcm_open_no_lock(rtd, substream);
	snd_soc_dpcm_mutex_unlock(rtd);
	return ret;
}

/*
 * Called by ALSA when the PCM substream is prepared, can set format, sample
 * rate, etc.  This function is non atomic and can be called multiple times,
 * it can refer to the runtime info.
 */
int snd_soc_pcm_prepare_no_lock(struct snd_soc_pcm_runtime *rtd,
				struct snd_pcm_substream *substream)
{
	struct snd_soc_dai *dai;
	int i, ret = 0;

	snd_soc_dpcm_mutex_assert_held(rtd);

	ret = snd_soc_link_prepare(substream);
	if (ret < 0)
		goto out;

	ret = snd_soc_pcm_component_prepare(substream);
	if (ret < 0)
		goto out;

	ret = snd_soc_pcm_dai_prepare(substream);
	if (ret < 0)
		goto out;

	/* cancel any delayed stream shutdown that is pending */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK &&
	    rtd->pop_wait) {
		rtd->pop_wait = 0;
		cancel_delayed_work(&rtd->delayed_work);
	}

	snd_soc_dapm_stream_event(rtd, substream->stream,
			SND_SOC_DAPM_STREAM_START);

	for_each_rtd_dais(rtd, i, dai) {
		if (!snd_soc_dai_mute_is_ctrled_at_trigger(dai))
			snd_soc_dai_digital_mute(dai, 0, substream->stream);
	}

out:
	/*
	 * Don't use soc_pcm_ret() on .prepare callback to lower error log severity
	 *
	 * We don't want to log an error since we do not want to give userspace a way to do a
	 * denial-of-service attack on the syslog / diskspace.
	 */
	return ret;
}

/* PCM prepare ops for non-DPCM streams */
static int soc_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	int ret;

	snd_soc_dpcm_mutex_lock(rtd);
	ret = snd_soc_pcm_prepare_no_lock(rtd, substream);
	snd_soc_dpcm_mutex_unlock(rtd);

	/*
	 * Don't use soc_pcm_ret() on .prepare callback to lower error log severity
	 *
	 * We don't want to log an error since we do not want to give userspace a way to do a
	 * denial-of-service attack on the syslog / diskspace.
	 */
	return ret;
}

static void soc_pcm_codec_params_fixup(struct snd_pcm_hw_params *params,
				       unsigned int mask)
{
	struct snd_interval *interval;
	int channels = hweight_long(mask);

	interval = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	interval->min = channels;
	interval->max = channels;
}

int snd_soc_pcm_hw_clean(struct snd_soc_pcm_runtime *rtd,
			 struct snd_pcm_substream *substream, int rollback)
{
	struct snd_soc_dai *dai;
	int i;

	snd_soc_dpcm_mutex_assert_held(rtd);

	/* clear the corresponding DAIs parameters when going to be inactive */
	for_each_rtd_dais(rtd, i, dai) {
		if (snd_soc_dai_active(dai) == 1)
			snd_soc_dai_symmetric_set_params(dai, NULL);

		if (snd_soc_dai_stream_active(dai, substream->stream) == 1) {
			if (!snd_soc_dai_mute_is_ctrled_at_trigger(dai))
				snd_soc_dai_digital_mute(dai, 1, substream->stream);
		}
	}

	/* run the stream event */
	snd_soc_dapm_stream_stop(rtd, substream->stream);

	/* free any machine hw params */
	snd_soc_link_hw_free(substream, rollback);

	/* free any component resources */
	snd_soc_pcm_component_hw_free(substream, rollback);

	/* now free hw params for the DAIs  */
	for_each_rtd_dais(rtd, i, dai)
		if (snd_soc_dai_stream_valid(dai, substream->stream))
			snd_soc_dai_hw_free(dai, substream, rollback);

	return 0;
}

/*
 * Frees resources allocated by hw_params, can be called multiple times
 */
/* hw_free PCM ops for non-DPCM streams */
static int soc_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	int ret;

	snd_soc_dpcm_mutex_lock(rtd);
	ret = snd_soc_pcm_hw_clean(rtd, substream, 0);
	snd_soc_dpcm_mutex_unlock(rtd);
	return ret;
}

/*
 * Called by ALSA when the hardware params are set by application. This
 * function can also be called multiple times and can allocate buffers
 * (using snd_pcm_lib_* ). It's non-atomic.
 */
int snd_soc_pcm_hw_params_no_lock(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	struct snd_pcm_hw_params tmp_params;
	int i, ret = 0;

	snd_soc_dpcm_mutex_assert_held(rtd);

	ret = snd_soc_dai_symmetric_params(substream, params);
	if (ret)
		goto out;

	ret = snd_soc_link_hw_params(substream, params);
	if (ret < 0)
		goto out;

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		unsigned int ch_mask = snd_soc_dai_stream_tdm_mask_get(codec_dai, substream->stream);
		struct snd_soc_dai_link_ch_map *ch_maps;
		int j;

		/*
		 * Skip CODECs which don't support the current stream type,
		 * the idea being that if a CODEC is not used for the currently
		 * set up transfer direction, it should not need to be
		 * configured, especially since the configuration used might
		 * not even be supported by that CODEC. There may be cases
		 * however where a CODEC needs to be set up although it is
		 * actually not being used for the transfer, e.g. if a
		 * capture-only CODEC is acting as an LRCLK and/or BCLK master
		 * for the DAI link including a playback-only CODEC.
		 * If this becomes necessary, we will have to augment the
		 * machine driver setup with information on how to act, so
		 * we can do the right thing here.
		 */
		if (!snd_soc_dai_stream_valid(codec_dai, substream->stream))
			continue;

		/* copy params for each codec */
		tmp_params = *params;

		/* fixup params based on TDM or ch_map masks */
		if (!ch_mask) {
			for_each_rtd_ch_maps(rtd, j, ch_maps)
				if (ch_maps->codec == i)
					ch_mask |= ch_maps->codec_ch_mask;
		}

		if (ch_mask)
			soc_pcm_codec_params_fixup(&tmp_params, ch_mask);

		ret = snd_soc_dai_hw_params(codec_dai, substream,
					    &tmp_params);
		if(ret < 0)
			goto out;

		snd_soc_dai_symmetric_set_params(codec_dai, &tmp_params);
		snd_soc_dapm_update_dai(substream, &tmp_params, codec_dai);
	}

	for_each_rtd_cpu_dais(rtd, i, cpu_dai) {
		struct snd_soc_dai_link_ch_map *ch_maps;
		unsigned int ch_mask = 0;
		int j;

		/*
		 * Skip CPUs which don't support the current stream
		 * type. See soc_pcm_init_runtime_hw() for more details
		 */
		if (!snd_soc_dai_stream_valid(cpu_dai, substream->stream))
			continue;

		/* copy params for each cpu */
		tmp_params = *params;

		/*
		 * construct cpu channel mask by combining ch_mask of each
		 * codec which maps to the cpu.
		 * see
		 *	soc.h :: [dai_link->ch_maps Image sample]
		 */
		for_each_rtd_ch_maps(rtd, j, ch_maps)
			if (ch_maps->cpu == i)
				ch_mask |= ch_maps->cpu_ch_mask;

		/* fixup cpu channel number */
		if (ch_mask)
			soc_pcm_codec_params_fixup(&tmp_params, ch_mask);

		ret = snd_soc_dai_hw_params(cpu_dai, substream, &tmp_params);
		if (ret < 0)
			goto out;

		/* store the parameters for each DAI */
		snd_soc_dai_symmetric_set_params(cpu_dai, &tmp_params);
		snd_soc_dapm_update_dai(substream, &tmp_params, cpu_dai);
	}

	ret = snd_soc_pcm_component_hw_params(substream, params);
out:
	if (ret < 0)
		snd_soc_pcm_hw_clean(rtd, substream, 1);

	return soc_pcm_ret(rtd, ret);
}

/* hw_params PCM ops for non-DPCM streams */
static int soc_pcm_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	int ret;

	snd_soc_dpcm_mutex_lock(rtd);
	ret = snd_soc_pcm_hw_params_no_lock(substream, params);
	snd_soc_dpcm_mutex_unlock(rtd);
	return ret;
}

#define TRIGGER_MAX 3
static int (* const trigger[][TRIGGER_MAX])(struct snd_pcm_substream *substream, int cmd, int rollback) = {
	[SND_SOC_TRIGGER_ORDER_DEFAULT] = {
		snd_soc_link_trigger,
		snd_soc_pcm_component_trigger,
		snd_soc_pcm_dai_trigger,
	},
	[SND_SOC_TRIGGER_ORDER_LDC] = {
		snd_soc_link_trigger,
		snd_soc_pcm_dai_trigger,
		snd_soc_pcm_component_trigger,
	},
};

int snd_soc_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_component *component;
	int ret = 0, r = 0, i;
	int rollback = 0;
	int start = 0, stop = 0;

	/*
	 * select START/STOP sequence
	 */
	for_each_rtd_components(rtd, i, component) {
		const struct snd_soc_component_driver *driver = snd_soc_component_to_driver(component);

		if (driver->trigger_start)
			start = driver->trigger_start;
		if (driver->trigger_stop)
			stop = driver->trigger_stop;
	}
	if (rtd->dai_link->trigger_start)
		start = rtd->dai_link->trigger_start;
	if (rtd->dai_link->trigger_stop)
		stop  = rtd->dai_link->trigger_stop;

	if (start < 0 || start >= SND_SOC_TRIGGER_ORDER_MAX ||
	    stop  < 0 || stop  >= SND_SOC_TRIGGER_ORDER_MAX) {
		ret = -EINVAL;
		goto err;
	}

	/*
	 * START
	 */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		for (i = 0; i < TRIGGER_MAX; i++) {
			r = trigger[start][i](substream, cmd, 0);
			if (r < 0)
				break;
		}
	}

	/*
	 * Rollback if START failed
	 * find correspond STOP command
	 */
	if (r < 0) {
		rollback = 1;
		ret = r;
		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			cmd = SNDRV_PCM_TRIGGER_STOP;
			break;
		case SNDRV_PCM_TRIGGER_RESUME:
			cmd = SNDRV_PCM_TRIGGER_SUSPEND;
			break;
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			cmd = SNDRV_PCM_TRIGGER_PAUSE_PUSH;
			break;
		}
	}

	/*
	 * STOP
	 */
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		for (i = TRIGGER_MAX; i > 0; i--) {
			r = trigger[stop][i - 1](substream, cmd, rollback);
			if (r < 0)
				ret = r;
		}
	}
err:
	return soc_pcm_ret(rtd, ret);
}

/*
 * soc level wrapper for pointer callback
 * If cpu_dai, codec_dai, component driver has the delay callback, then
 * the runtime->delay will be updated via snd_soc_pcm_component/dai_delay().
 */
snd_pcm_uframes_t snd_soc_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	snd_pcm_uframes_t offset = 0;
	snd_pcm_sframes_t codec_delay = 0;
	snd_pcm_sframes_t cpu_delay = 0;

	offset = snd_soc_pcm_component_pointer(substream);

	/* should be called *after* snd_soc_pcm_component_pointer() */
	snd_soc_pcm_dai_delay(substream, &cpu_delay, &codec_delay);
	snd_soc_pcm_component_delay(substream, &cpu_delay, &codec_delay);

	runtime->delay = cpu_delay + codec_delay;

	return offset;
}

int widget_in_list(struct snd_soc_dapm_widget_list *list,
		struct snd_soc_dapm_widget *widget)
{
	struct snd_soc_dapm_widget *w;
	int i;

	for_each_dapm_widgets(list, i, w)
		if (widget == w)
			return 1;

	return 0;
}
EXPORT_SYMBOL_GPL(widget_in_list);

int snd_soc_pcm_detect_playback_capture(struct snd_soc_pcm_runtime *rtd,
					int *playback, int *capture, bool use_dummy_codec)
{
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_dai_link_ch_map *ch_maps;
	struct snd_soc_dai *dummy_dai = snd_soc_find_dai_nolock(&snd_soc_dummy_dlc);
	int cpu_capture;
	int cpu_playback;
	int has_playback = 0;
	int has_capture  = 0;
	int i;

	/* Adapt stream for codec2codec links */
	cpu_capture  = snd_soc_get_stream_cpu(dai_link, SNDRV_PCM_STREAM_CAPTURE);
	cpu_playback = snd_soc_get_stream_cpu(dai_link, SNDRV_PCM_STREAM_PLAYBACK);

	/*
	 * see
	 *	soc.h :: [dai_link->ch_maps Image sample]
	 */
	for_each_rtd_ch_maps(rtd, i, ch_maps) {
		cpu_dai	  = snd_soc_rtd_to_cpu(rtd,   ch_maps->cpu);
		codec_dai = snd_soc_rtd_to_codec(rtd, ch_maps->codec);

		/*
		 * FIXME
		 *
		 * DPCM Codec has been no checked before.
		 * It should be checked, but it breaks compatibility.
		 *
		 * For example there is a case that CPU have loopback capabilities which is used
		 * for tests on boards where the Codec has no capture capabilities. In this case,
		 * Codec capture validation check will be fail, but system should allow capture
		 * capabilities. We have no solution for it today.
		 */
		if (use_dummy_codec)
			codec_dai = dummy_dai;

		if (snd_soc_dai_stream_valid(codec_dai, SNDRV_PCM_STREAM_PLAYBACK) &&
		    snd_soc_dai_stream_valid(cpu_dai,   cpu_playback))
			has_playback = 1;
		if (snd_soc_dai_stream_valid(codec_dai, SNDRV_PCM_STREAM_CAPTURE) &&
		    snd_soc_dai_stream_valid(cpu_dai,   cpu_capture))
			has_capture = 1;
	}

	if (dai_link->playback_only)
		has_capture = 0;

	if (dai_link->capture_only)
		has_playback = 0;

	if (!has_playback && !has_capture)
		return snd_soc_ret(rtd->dev, -EINVAL,
			"substream %s has no playback, no capture\n", dai_link->stream_name);

	*playback = has_playback;
	*capture  = has_capture;

	return 0;
}

static int soc_create_pcm(struct snd_pcm **pcm,
			  struct snd_soc_pcm_runtime *rtd,
			  int playback, int capture)
{
	struct snd_card *snd_card = snd_soc_card_to_snd_card(rtd->card);
	char new_name[64];
	int ret;

	/* create the PCM */
	if (rtd->dai_link->c2c_params) {
		snprintf(new_name, sizeof(new_name), "codec2codec(%s)",
			 rtd->dai_link->stream_name);

		ret = snd_pcm_new_internal(snd_card, new_name, rtd->id,
					   playback, capture, pcm);
	} else {
		snprintf(new_name, sizeof(new_name), "%s %s-%d",
			 rtd->dai_link->stream_name,
			 soc_codec_dai_name(rtd), rtd->id);

		ret = snd_pcm_new(snd_card, new_name, rtd->id, playback,
			capture, pcm);
	}
	if (ret < 0)
		return snd_soc_ret(rtd->dev, ret,
			"can't create pcm %s for dailink %s\n", new_name, rtd->dai_link->name);

	dev_dbg(snd_soc_card_to_dev(rtd->card), "ASoC: registered pcm #%d %s\n", rtd->id, new_name);

	return 0;
}

int snd_soc_pcm_new_common(struct snd_soc_pcm_runtime *rtd, int playback, int capture)
{
	struct snd_soc_component *component;
	int i, ret;

	if (!rtd->pcm)
		return soc_pcm_ret(rtd, -EINVAL);

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

	if (playback)
		snd_pcm_set_ops(rtd->pcm, SNDRV_PCM_STREAM_PLAYBACK, &rtd->ops);

	if (capture)
		snd_pcm_set_ops(rtd->pcm, SNDRV_PCM_STREAM_CAPTURE, &rtd->ops);

	ret = snd_soc_pcm_component_new(rtd);

	return soc_pcm_ret(rtd, ret);
}

/* create a new pcm */
static int soc_new_pcm_orig(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_pcm *pcm;
	int ret = 0, playback = 0, capture = 0;

	ret = snd_soc_pcm_detect_playback_capture(rtd, &playback, &capture, false);
	if (ret < 0)
		goto err;

	ret = soc_create_pcm(&pcm, rtd, playback, capture);
	if (ret < 0)
		goto err;

	/* DAPM dai link stream work */
	/*
	 * Currently nothing to do for c2c links
	 * Since c2c links are internal nodes in the DAPM graph and
	 * don't interface with the outside world or application layer
	 * we don't have to do any special handling on close.
	 */
	if (!rtd->dai_link->c2c_params)
		rtd->close_delayed_work_func = snd_soc_close_delayed_work;

	rtd->pcm = pcm;
	pcm->nonatomic = rtd->dai_link->nonatomic;
	pcm->private_data = rtd;
	pcm->no_device_suspend = true;

	if (rtd->dai_link->c2c_params) {
		if (playback)
			pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream->private_data = rtd;
		if (capture)
			pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream->private_data = rtd;
		goto err;
	}

	/* ASoC PCM operations */
	rtd->ops.open		= soc_pcm_open;
	rtd->ops.hw_params	= soc_pcm_hw_params;
	rtd->ops.prepare	= soc_pcm_prepare;
	rtd->ops.trigger	= snd_soc_pcm_trigger;
	rtd->ops.hw_free	= soc_pcm_hw_free;
	rtd->ops.close		= soc_pcm_close;
	rtd->ops.pointer	= snd_soc_pcm_pointer;

	ret = snd_soc_pcm_new_common(rtd, playback, capture);
err:
	return soc_pcm_ret(rtd, ret);
}

int soc_new_pcm(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct device *dev = rtd->dev;
	int ret;

	/*
	 * UPDATE ME
	 *
	 * Maybe this should be grouped as a collection of function
	 * pointers in the future. But is calling each function directly for now
	 */
	if (dai_link->dynamic)
		ret = snd_soc_pcm_new_dpcm_fe(rtd);
	else if (dai_link->no_pcm)
		ret = snd_soc_pcm_new_dpcm_be(rtd);
	else
		ret = soc_new_pcm_orig(rtd);

	if (ret < 0)
		dev_err(dev, "ASoC: can't create pcm %s :%d\n",
			dai_link->stream_name, ret);
	else
		dev_dbg(dev, "%s <-> %s mapping ok\n",
			soc_codec_dai_name(rtd), soc_cpu_dai_name(rtd));

	return soc_pcm_ret(rtd, ret);
}

/* get the substream for this BE */
struct snd_pcm_substream *
	snd_soc_dpcm_get_substream(struct snd_soc_pcm_runtime *be, int stream)
{
	return be->pcm->streams[stream].substream;
}
EXPORT_SYMBOL_GPL(snd_soc_dpcm_get_substream);
