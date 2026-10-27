// SPDX-License-Identifier: GPL-2.0+
//
// soc-pcm-dpcm.c -- ALSA SoC PCM for DPCM
//
// Copyright (c) 2026 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//
#include <linux/debugfs.h>
#include <sound/soc.h>
#include "soc-internal.h"

#define DPCM_MAX_BE_USERS	8
#define soc_pcm_ret(rtd, ret) snd_soc_ret(rtd->dev, ret, "on %s\n", rtd->dai_link->name)

static int snd_soc_dpcm_check_state(struct snd_soc_pcm_runtime *fe,
				    struct snd_soc_pcm_runtime *be,
				    int stream,
				    const enum snd_soc_dpcm_state *states,
				    int num_states)
{
	struct snd_soc_dpcm *dpcm;
	int state;
	int ret = 1;
	int i;

	for_each_dpcm_fe(be, stream, dpcm) {

		if (dpcm->fe == fe)
			continue;

		state = dpcm->fe->dpcm[stream].state;
		for (i = 0; i < num_states; i++) {
			if (state == states[i]) {
				ret = 0;
				break;
			}
		}
	}

	/* it's safe to do this BE DAI */
	return ret;
}

/* is the current PCM operation for this FE ? */
#if 0
static int snd_soc_dpcm_can_fe_update(struct snd_soc_pcm_runtime *fe, int stream)
{
	if (fe->dpcm[stream].runtime_update == SND_SOC_DPCM_UPDATE_FE)
		return 1;
	return 0;
}
#endif

/* is the current PCM operation for this BE ? */
static int snd_soc_dpcm_can_be_update(struct snd_soc_pcm_runtime *fe,
				      struct snd_soc_pcm_runtime *be, int stream)
{
	if ((fe->dpcm[stream].runtime_update == SND_SOC_DPCM_UPDATE_FE) ||
	    ((fe->dpcm[stream].runtime_update == SND_SOC_DPCM_UPDATE_BE) &&
	     be->dpcm[stream].runtime_update))
		return 1;
	return 0;
}

/*
 * We can only hw_free, stop, pause or suspend a BE DAI if any of it's FE
 * are not running, paused or suspended for the specified stream direction.
 */
static int snd_soc_dpcm_can_be_free_stop(struct snd_soc_pcm_runtime *fe,
					 struct snd_soc_pcm_runtime *be, int stream)
{
	const enum snd_soc_dpcm_state state[] = {
		SND_SOC_DPCM_STATE_START,
		SND_SOC_DPCM_STATE_PAUSED,
		SND_SOC_DPCM_STATE_SUSPEND,
	};

	return snd_soc_dpcm_check_state(fe, be, stream, state, ARRAY_SIZE(state));
}

/*
 * We can only change hw params a BE DAI if any of it's FE are not prepared,
 * running, paused or suspended for the specified stream direction.
 */
static int snd_soc_dpcm_can_be_params(struct snd_soc_pcm_runtime *fe,
				      struct snd_soc_pcm_runtime *be, int stream)
{
	const enum snd_soc_dpcm_state state[] = {
		SND_SOC_DPCM_STATE_START,
		SND_SOC_DPCM_STATE_PAUSED,
		SND_SOC_DPCM_STATE_SUSPEND,
		SND_SOC_DPCM_STATE_PREPARE,
	};

	return snd_soc_dpcm_check_state(fe, be, stream, state, ARRAY_SIZE(state));
}

/*
 * We can only prepare a BE DAI if any of it's FE are not prepared,
 * running or paused for the specified stream direction.
 */
static int snd_soc_dpcm_can_be_prepared(struct snd_soc_pcm_runtime *fe,
					struct snd_soc_pcm_runtime *be, int stream)
{
	const enum snd_soc_dpcm_state state[] = {
		SND_SOC_DPCM_STATE_START,
		SND_SOC_DPCM_STATE_PAUSED,
		SND_SOC_DPCM_STATE_PREPARE,
	};

	return snd_soc_dpcm_check_state(fe, be, stream, state, ARRAY_SIZE(state));
}

static const char *dpcm_state_string(enum snd_soc_dpcm_state state)
{
	switch (state) {
	case SND_SOC_DPCM_STATE_NEW:
		return "new";
	case SND_SOC_DPCM_STATE_OPEN:
		return "open";
	case SND_SOC_DPCM_STATE_HW_PARAMS:
		return "hw_params";
	case SND_SOC_DPCM_STATE_PREPARE:
		return "prepare";
	case SND_SOC_DPCM_STATE_START:
		return "start";
	case SND_SOC_DPCM_STATE_STOP:
		return "stop";
	case SND_SOC_DPCM_STATE_SUSPEND:
		return "suspend";
	case SND_SOC_DPCM_STATE_PAUSED:
		return "paused";
	case SND_SOC_DPCM_STATE_HW_FREE:
		return "hw_free";
	case SND_SOC_DPCM_STATE_CLOSE:
		return "close";
	}

	return "unknown";
}

#ifdef CONFIG_DEBUG_FS
static ssize_t dpcm_show_state(struct snd_soc_pcm_runtime *fe,
			       int stream, char *buf, size_t size)
{
	struct snd_pcm_hw_params *params = &fe->dpcm[stream].hw_params;
	struct snd_soc_dpcm *dpcm;
	ssize_t offset = 0;

	/* FE state */
	offset += scnprintf(buf + offset, size - offset,
			    "[%s - %s]\n", fe->dai_link->name,
			    stream ? "Capture" : "Playback");

	offset += scnprintf(buf + offset, size - offset, "State: %s\n",
			    dpcm_state_string(fe->dpcm[stream].state));

	if ((fe->dpcm[stream].state >= SND_SOC_DPCM_STATE_HW_PARAMS) &&
	    (fe->dpcm[stream].state <= SND_SOC_DPCM_STATE_STOP))
		offset += scnprintf(buf + offset, size - offset,
				    "Hardware Params: "
				    "Format = %s, Channels = %d, Rate = %d\n",
				    snd_pcm_format_name(params_format(params)),
				    params_channels(params),
				    params_rate(params));

	/* BEs state */
	offset += scnprintf(buf + offset, size - offset, "Backends:\n");

	if (list_empty(&fe->dpcm[stream].be_clients)) {
		offset += scnprintf(buf + offset, size - offset,
				    " No active DSP links\n");
		goto out;
	}

	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_soc_pcm_runtime *be = dpcm->be;
		params = &be->dpcm[stream].hw_params;

		offset += scnprintf(buf + offset, size - offset,
				    "- %s\n", be->dai_link->name);

		offset += scnprintf(buf + offset, size - offset,
				    "   State: %s\n",
				    dpcm_state_string(be->dpcm[stream].state));

		if ((be->dpcm[stream].state >= SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state <= SND_SOC_DPCM_STATE_STOP))
			offset += scnprintf(buf + offset, size - offset,
					    "   Hardware Params: "
					    "Format = %s, Channels = %d, Rate = %d\n",
					    snd_pcm_format_name(params_format(params)),
					    params_channels(params),
					    params_rate(params));
	}
out:
	return offset;
}

static ssize_t dpcm_state_read_file(struct file *file, char __user *user_buf,
				    size_t count, loff_t *ppos)
{
	struct snd_soc_pcm_runtime *fe = file->private_data;
	ssize_t out_count = PAGE_SIZE, offset = 0, ret = 0;
	int stream;
	char *buf;

	if (fe->dai_link->num_cpus > 1)
		return snd_soc_ret(fe->dev, -EINVAL,
				   "doesn't support Multi CPU yet\n");

	buf = kmalloc(out_count, GFP_KERNEL);
	if (!buf)
		return soc_pcm_ret(fe, -ENOMEM);

	snd_soc_dpcm_mutex_lock(fe);
	for_each_pcm_streams(stream)
		if (snd_soc_dai_stream_valid(snd_soc_rtd_to_cpu(fe, 0), stream))
			offset += dpcm_show_state(fe, stream,
						  buf + offset,
						  out_count - offset);
	snd_soc_dpcm_mutex_unlock(fe);

	ret = simple_read_from_buffer(user_buf, count, ppos, buf, offset);

	kfree(buf);

	return soc_pcm_ret(fe, ret);
}

static const struct file_operations dpcm_state_fops = {
	.open = simple_open,
	.read = dpcm_state_read_file,
	.llseek = default_llseek,
};

void soc_dpcm_debugfs_add(struct snd_soc_pcm_runtime *rtd)
{
	struct dentry *debugfs_root = snd_soc_card_to_debugfs_root(rtd->card);

	if (!rtd->dai_link->dynamic)
		return;

	if (!debugfs_root)
		return;

	rtd->debugfs_dpcm_root = debugfs_create_dir(rtd->dai_link->name,
						    debugfs_root);

	debugfs_create_file("state", 0444, debugfs_root,
			    rtd, &dpcm_state_fops);
}

static void dpcm_create_debugfs_state(struct snd_soc_dpcm *dpcm, int stream)
{
	char *name;

	name = kasprintf(GFP_KERNEL, "%s:%s", dpcm->be->dai_link->name,
			 snd_pcm_direction_name(stream));
	if (name) {
		dpcm->debugfs_state = debugfs_create_dir(
			name, dpcm->fe->debugfs_dpcm_root);
		debugfs_create_u32("state", 0644, dpcm->debugfs_state,
				   &dpcm->state);
		kfree(name);
	}
}

static void dpcm_remove_debugfs_state(struct snd_soc_dpcm *dpcm)
{
	debugfs_remove_recursive(dpcm->debugfs_state);
}

#else
static inline void dpcm_create_debugfs_state(struct snd_soc_dpcm *dpcm,
					     int stream)
{
}

static inline void dpcm_remove_debugfs_state(struct snd_soc_dpcm *dpcm)
{
}
#endif

/* Set FE's runtime_update state; the state is protected via PCM stream lock
 * for avoiding the race with trigger callback.
 * If the state is unset and a trigger is pending while the previous operation,
 * process the pending trigger action here.
 */
static int dpcm_fe_dai_do_trigger(struct snd_pcm_substream *substream, int cmd);
static void dpcm_set_fe_update_state(struct snd_soc_pcm_runtime *fe,
				     int stream, enum snd_soc_dpcm_update state)
{
	struct snd_pcm_substream *substream =
		snd_soc_dpcm_get_substream(fe, stream);

	snd_pcm_stream_lock_irq(substream);
	if (state == SND_SOC_DPCM_UPDATE_NO && fe->dpcm[stream].trigger_pending) {
		dpcm_fe_dai_do_trigger(substream,
				       fe->dpcm[stream].trigger_pending - 1);
		fe->dpcm[stream].trigger_pending = 0;
	}
	fe->dpcm[stream].runtime_update = state;
	snd_pcm_stream_unlock_irq(substream);
}

static void dpcm_set_be_update_state(struct snd_soc_pcm_runtime *be,
				     int stream, enum snd_soc_dpcm_update state)
{
	be->dpcm[stream].runtime_update = state;
}

/* DPCM stream event, send event to FE and all active BEs. */
void dpcm_dapm_stream_event(struct snd_soc_pcm_runtime *fe, int dir, int event)
{
	struct snd_soc_dpcm *dpcm;

	snd_soc_dpcm_mutex_assert_held(fe);

	for_each_dpcm_be(fe, dir, dpcm) {

		struct snd_soc_pcm_runtime *be = dpcm->be;

		dev_dbg(be->dev, "ASoC: BE %s event %d dir %d\n",
			be->dai_link->name, event, dir);

		if ((event == SND_SOC_DAPM_STREAM_STOP) &&
		    (be->dpcm[dir].users >= 1))
			continue;

		snd_soc_dapm_stream_event(be, dir, event);
	}

	snd_soc_dapm_stream_event(fe, dir, event);
}

/* connect a FE and BE */
static int dpcm_be_connect(struct snd_soc_pcm_runtime *fe,
			   struct snd_soc_pcm_runtime *be, int stream)
{
	struct snd_pcm_substream *fe_substream;
	struct snd_pcm_substream *be_substream;
	struct snd_soc_dpcm *dpcm;

	snd_soc_dpcm_mutex_assert_held(fe);

	/* only add new dpcms */
	for_each_dpcm_be(fe, stream, dpcm)
		if (dpcm->be == be)
			return 0;

	fe_substream = snd_soc_dpcm_get_substream(fe, stream);
	be_substream = snd_soc_dpcm_get_substream(be, stream);

	if (!fe_substream->pcm->nonatomic && be_substream->pcm->nonatomic)
		return snd_soc_ret(be->dev, -EINVAL,
				   "%s is atomic but %s is nonatomic, invalid configuration\n",
				   fe->dai_link->name, be->dai_link->name);

	if (fe_substream->pcm->nonatomic && !be_substream->pcm->nonatomic) {
		dev_dbg(be->dev, "FE is nonatomic but BE is not, forcing BE as nonatomic\n");
		be_substream->pcm->nonatomic = 1;
	}

	dpcm = kzalloc_obj(struct snd_soc_dpcm);
	if (!dpcm)
		return soc_pcm_ret(fe, -ENOMEM);

	dpcm->be = be;
	dpcm->fe = fe;
	dpcm->state = SND_SOC_DPCM_LINK_STATE_NEW;
	snd_pcm_stream_lock_irq(fe_substream);
	list_add(&dpcm->list_be, &fe->dpcm[stream].be_clients);
	list_add(&dpcm->list_fe, &be->dpcm[stream].fe_clients);
	snd_pcm_stream_unlock_irq(fe_substream);

	dev_dbg(fe->dev, "connected new DPCM %s path %s %s %s\n",
		snd_pcm_direction_name(stream),  fe->dai_link->name,
		stream ? "<-" : "->", be->dai_link->name);

	dpcm_create_debugfs_state(dpcm, stream);

	return 1;
}

/* reparent a BE onto another FE */
static void dpcm_be_reparent(struct snd_soc_pcm_runtime *fe,
			     struct snd_soc_pcm_runtime *be, int stream)
{
	struct snd_soc_dpcm *dpcm;
	struct snd_pcm_substream *fe_substream, *be_substream;

	/* reparent if BE is connected to other FEs */
	if (!be->dpcm[stream].users)
		return;

	be_substream = snd_soc_dpcm_get_substream(be, stream);
	if (!be_substream)
		return;

	for_each_dpcm_fe(be, stream, dpcm) {
		if (dpcm->fe == fe)
			continue;

		dev_dbg(fe->dev, "reparent %s path %s %s %s\n",
			snd_pcm_direction_name(stream),
			dpcm->fe->dai_link->name,
			stream ? "<-" : "->", dpcm->be->dai_link->name);

		fe_substream = snd_soc_dpcm_get_substream(dpcm->fe, stream);
		be_substream->runtime = fe_substream->runtime;
		break;
	}
}

/* disconnect a BE and FE */
void dpcm_be_disconnect(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm *dpcm, *d;
	struct snd_pcm_substream *substream = snd_soc_dpcm_get_substream(fe, stream);
	LIST_HEAD(deleted_dpcms);

	snd_soc_dpcm_mutex_assert_held(fe);

	snd_pcm_stream_lock_irq(substream);
	for_each_dpcm_be_safe(fe, stream, dpcm, d) {
		dev_dbg(fe->dev, "ASoC: BE %s disconnect check for %s\n",
			snd_pcm_direction_name(stream),
			dpcm->be->dai_link->name);

		if (dpcm->state != SND_SOC_DPCM_LINK_STATE_FREE)
			continue;

		dev_dbg(fe->dev, "freed DSP %s path %s %s %s\n",
			snd_pcm_direction_name(stream), fe->dai_link->name,
			stream ? "<-" : "->", dpcm->be->dai_link->name);

		/* BEs still alive need new FE */
		dpcm_be_reparent(fe, dpcm->be, stream);

		list_del(&dpcm->list_be);
		list_move(&dpcm->list_fe, &deleted_dpcms);
	}
	snd_pcm_stream_unlock_irq(substream);

	while (!list_empty(&deleted_dpcms)) {
		dpcm = list_first_entry(&deleted_dpcms, struct snd_soc_dpcm,
					list_fe);
		list_del(&dpcm->list_fe);
		dpcm_remove_debugfs_state(dpcm);
		kfree(dpcm);
	}
}

/* get BE for DAI widget and stream */
static struct snd_soc_pcm_runtime *dpcm_get_be(struct snd_soc_card *card,
					       struct snd_soc_dapm_widget *widget, int stream)
{
	struct device *dev = snd_soc_card_to_dev(card);
	struct snd_soc_pcm_runtime *be;
	struct snd_soc_dapm_widget *w;
	struct snd_soc_dai *dai;
	int i;

	dev_dbg(dev, "ASoC: find BE for widget %s\n", widget->name);

	for_each_card_rtds(card, be) {

		if (!be->dai_link->no_pcm)
			continue;

		if (!snd_soc_dpcm_get_substream(be, stream))
			continue;

		for_each_rtd_dais(be, i, dai) {
			w = snd_soc_dai_stream_widget_get(dai, stream);

			dev_dbg(dev, "ASoC: try BE : %s\n",
				w ? w->name : "(not set)");

			if (w == widget)
				return be;
		}
	}

	/* Widget provided is not a BE */
	return NULL;
}

bool dpcm_end_walk_at_be(struct snd_soc_dapm_widget *widget, enum snd_soc_dapm_direction dir)
{
	struct snd_soc_card *card = snd_soc_dapm_to_card(widget->dapm);
	struct snd_soc_pcm_runtime *rtd;
	int stream;

	/* adjust dir to stream */
	if (dir == SND_SOC_DAPM_DIR_OUT)
		stream = SNDRV_PCM_STREAM_PLAYBACK;
	else
		stream = SNDRV_PCM_STREAM_CAPTURE;

	rtd = dpcm_get_be(card, widget, stream);
	if (rtd)
		return true;

	return false;
}
EXPORT_SYMBOL_GPL(dpcm_end_walk_at_be);

int dpcm_path_get(struct snd_soc_pcm_runtime *fe,
		  int stream, struct snd_soc_dapm_widget_list **list)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(fe, 0);
	int paths;

	if (fe->dai_link->num_cpus > 1)
		return snd_soc_ret(fe->dev, -EINVAL, "doesn't support Multi CPU yet\n");

	/* get number of valid DAI paths and their widgets */
	paths = snd_soc_dapm_dai_get_connected_widgets(cpu_dai, stream, list,
						       snd_soc_card_to_driver(fe->card)->component_chaining ?
						       NULL : dpcm_end_walk_at_be);

	if (paths > 0)
		dev_dbg(fe->dev, "ASoC: found %d audio %s paths\n", paths,
			snd_pcm_direction_name(stream));
	else if (paths == 0)
		dev_dbg(fe->dev, "ASoC: %s no valid %s path\n", fe->dai_link->name,
			snd_pcm_direction_name(stream));

	return paths;
}

void dpcm_path_put(struct snd_soc_dapm_widget_list **list)
{
	snd_soc_dapm_dai_free_widgets(list);
}

static bool dpcm_be_is_active(struct snd_soc_dpcm *dpcm, int stream,
			      struct snd_soc_dapm_widget_list *list)
{
	struct snd_soc_dai *dai;
	unsigned int i;

	/* is there a valid DAI widget for this BE */
	for_each_rtd_dais(dpcm->be, i, dai) {
		struct snd_soc_dapm_widget *widget = snd_soc_dai_stream_widget_get(dai, stream);

		/*
		 * The BE is pruned only if none of the dai
		 * widgets are in the active list.
		 */
		if (widget && widget_in_list(list, widget))
			return true;
	}

	return false;
}

static int dpcm_prune_paths(struct snd_soc_pcm_runtime *fe, int stream,
			    struct snd_soc_dapm_widget_list **list_)
{
	struct snd_soc_dpcm *dpcm;
	int prune = 0;

	/* Destroy any old FE <--> BE connections */
	for_each_dpcm_be(fe, stream, dpcm) {
		if (dpcm_be_is_active(dpcm, stream, *list_))
			continue;

		dev_dbg(fe->dev, "ASoC: pruning %s BE %s for %s\n",
			snd_pcm_direction_name(stream),
			dpcm->be->dai_link->name, fe->dai_link->name);
		dpcm->state = SND_SOC_DPCM_LINK_STATE_FREE;
		dpcm_set_be_update_state(dpcm->be, stream, SND_SOC_DPCM_UPDATE_BE);
		prune++;
	}

	dev_dbg(fe->dev, "ASoC: found %d old BE paths for pruning\n", prune);
	return prune;
}

int dpcm_add_paths(struct snd_soc_pcm_runtime *fe, int stream,
		   struct snd_soc_dapm_widget_list **list_)
{
	struct snd_soc_card *card = fe->card;
	struct snd_soc_dapm_widget_list *list = *list_;
	struct snd_soc_pcm_runtime *be;
	struct snd_soc_dapm_widget *widget;
	struct snd_pcm_substream *fe_substream = snd_soc_dpcm_get_substream(fe, stream);
	int i, new = 0, err;

	/* don't connect if FE is not running */
	if (!fe_substream->runtime && !fe->fe_compr)
		return new;

	/* Create any new FE <--> BE connections */
	for_each_dapm_widgets(list, i, widget) {

		switch (widget->id) {
		case snd_soc_dapm_dai_in:
			if (stream != SNDRV_PCM_STREAM_PLAYBACK)
				continue;
			break;
		case snd_soc_dapm_dai_out:
			if (stream != SNDRV_PCM_STREAM_CAPTURE)
				continue;
			break;
		default:
			continue;
		}

		/* is there a valid BE rtd for this widget */
		be = dpcm_get_be(card, widget, stream);
		if (!be) {
			dev_dbg(fe->dev, "ASoC: no BE found for %s\n",
				widget->name);
			continue;
		}

		/*
		 * Filter for systems with 'component_chaining' enabled.
		 * This helps to avoid unnecessary re-configuration of an
		 * already active BE on such systems and ensures the BE DAI
		 * widget is powered ON after hw_params() BE DAI callback.
		 */
		if (snd_soc_card_to_driver(fe->card)->component_chaining &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_NEW) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_OPEN) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_CLOSE))
			continue;

		/* newly connected FE and BE */
		err = dpcm_be_connect(fe, be, stream);
		if (err < 0) {
			dev_err(fe->dev, "ASoC: can't connect %s\n",
				widget->name);
			break;
		} else if (err == 0) /* already connected */
			continue;

		/* new */
		dpcm_set_be_update_state(be, stream, SND_SOC_DPCM_UPDATE_BE);
		new++;
	}

	dev_dbg(fe->dev, "ASoC: found %d new BE paths\n", new);
	return new;
}

void dpcm_clear_pending_state(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm *dpcm;

	for_each_dpcm_be(fe, stream, dpcm)
		dpcm_set_be_update_state(dpcm->be, stream, SND_SOC_DPCM_UPDATE_NO);
}

void dpcm_be_dai_stop(struct snd_soc_pcm_runtime *fe, int stream,
		      int do_hw_free, struct snd_soc_dpcm *last)
{
	struct snd_soc_dpcm *dpcm;

	/* disable any enabled and non active backends */
	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_soc_pcm_runtime *be = dpcm->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		if (dpcm == last)
			return;

		/* is this op for this BE ? */
		if (!snd_soc_dpcm_can_be_update(fe, be, stream))
			continue;

		if (be->dpcm[stream].users == 0) {
			dev_err(be->dev, "ASoC: no users %s at close - state %s\n",
				snd_pcm_direction_name(stream),
				dpcm_state_string(be->dpcm[stream].state));
			continue;
		}

		if (--be->dpcm[stream].users != 0)
			continue;

		if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_OPEN) {
			if (!do_hw_free)
				continue;

			if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_FREE) {
				snd_soc_pcm_hw_clean(be, be_substream, 0);
				be->dpcm[stream].state = SND_SOC_DPCM_STATE_HW_FREE;
			}
		}

		snd_soc_pcm_close_no_lock(be, be_substream, 0);
		be_substream->runtime = NULL;
		be->dpcm[stream].state = SND_SOC_DPCM_STATE_CLOSE;
	}
}

int dpcm_be_dai_startup(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_pcm_substream *fe_substream = snd_soc_dpcm_get_substream(fe, stream);
	struct snd_soc_pcm_runtime *be;
	struct snd_soc_dpcm *dpcm;
	int err, count = 0;

	/* only startup BE DAIs that are either sinks or sources to this FE DAI */
	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_pcm_substream *be_substream;

		be = dpcm->be;
		be_substream = snd_soc_dpcm_get_substream(be, stream);

		if (!be_substream) {
			dev_err(be->dev, "ASoC: no backend %s stream\n",
				snd_pcm_direction_name(stream));
			continue;
		}

		/* is this op for this BE ? */
		if (!snd_soc_dpcm_can_be_update(fe, be, stream))
			continue;

		/* first time the dpcm is open ? */
		if (be->dpcm[stream].users == DPCM_MAX_BE_USERS) {
			dev_err(be->dev, "ASoC: too many users %s at open %s\n",
				snd_pcm_direction_name(stream),
				dpcm_state_string(be->dpcm[stream].state));
			continue;
		}

		if (be->dpcm[stream].users++ != 0)
			continue;

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_NEW) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_CLOSE))
			continue;

		dev_dbg(be->dev, "ASoC: open %s BE %s\n",
			snd_pcm_direction_name(stream), be->dai_link->name);

		be_substream->runtime = fe_substream->runtime;
		err = snd_soc_pcm_open_no_lock(be, be_substream);
		if (err < 0) {
			be->dpcm[stream].users--;
			if (be->dpcm[stream].users < 0)
				dev_err(be->dev, "ASoC: no users %s at unwind %s\n",
					snd_pcm_direction_name(stream),
					dpcm_state_string(be->dpcm[stream].state));

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_CLOSE;
			goto unwind;
		}
		be->dpcm[stream].be_start = 0;
		be->dpcm[stream].state = SND_SOC_DPCM_STATE_OPEN;
		count++;
	}

	return count;

unwind:
	dpcm_be_dai_startup_rollback(fe, stream, dpcm);

	return soc_pcm_ret(fe, err);
}

static void dpcm_runtime_setup_fe(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_pcm_hardware *hw = &runtime->hw;
	struct snd_soc_dai *dai;
	int stream = substream->stream;
	int i;

	snd_soc_pcm_hw_init(hw, false);

	for_each_rtd_cpu_dais(fe, i, dai) {
		const struct snd_soc_pcm_stream *cpu_stream;

		/*
		 * Skip CPUs which don't support the current stream
		 * type. See soc_pcm_init_runtime_hw() for more details
		 */
		if (!snd_soc_dai_stream_valid(dai, stream))
			continue;

		cpu_stream = snd_soc_dai_pcm_stream_get(dai, stream);

		snd_soc_pcm_hw_update_rate(hw, cpu_stream);
		snd_soc_pcm_hw_update_chan(hw, cpu_stream);
		snd_soc_pcm_hw_update_format(hw, cpu_stream);
	}

}

static void dpcm_runtime_setup_be_format(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_pcm_hardware *hw = &runtime->hw;
	struct snd_soc_dpcm *dpcm;
	struct snd_soc_dai *dai;
	int stream = substream->stream;

	if (!fe->dai_link->dpcm_merged_format)
		return;

	/*
	 * It returns merged BE codec format
	 * if FE want to use it (= dpcm_merged_format)
	 */

	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_soc_pcm_runtime *be = dpcm->be;
		const struct snd_soc_pcm_stream *codec_stream;
		int i;

		for_each_rtd_codec_dais(be, i, dai) {
			/*
			 * Skip CODECs which don't support the current stream
			 * type. See soc_pcm_init_runtime_hw() for more details
			 */
			if (!snd_soc_dai_stream_valid(dai, stream))
				continue;

			codec_stream = snd_soc_dai_pcm_stream_get(dai, stream);

			snd_soc_pcm_hw_update_format(hw, codec_stream);
		}
	}
}

static void dpcm_runtime_setup_be_chan(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_pcm_hardware *hw = &runtime->hw;
	struct snd_soc_dpcm *dpcm;
	int stream = substream->stream;

	if (!fe->dai_link->dpcm_merged_chan)
		return;

	/*
	 * It returns merged BE codec channel;
	 * if FE want to use it (= dpcm_merged_chan)
	 */

	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_soc_pcm_runtime *be = dpcm->be;
		const struct snd_soc_pcm_stream *cpu_stream;
		struct snd_soc_dai *dai;
		int i;

		for_each_rtd_cpu_dais(be, i, dai) {
			/*
			 * Skip CPUs which don't support the current stream
			 * type. See soc_pcm_init_runtime_hw() for more details
			 */
			if (!snd_soc_dai_stream_valid(dai, stream))
				continue;

			cpu_stream = snd_soc_dai_pcm_stream_get(dai, stream);

			snd_soc_pcm_hw_update_chan(hw, cpu_stream);
		}

		/*
		 * chan min/max cannot be enforced if there are multiple CODEC
		 * DAIs connected to a single CPU DAI, use CPU DAI's directly
		 */
		if (be->dai_link->num_codecs == 1) {
			const struct snd_soc_pcm_stream *codec_stream = snd_soc_dai_pcm_stream_get(
				snd_soc_rtd_to_codec(be, 0), stream);

			snd_soc_pcm_hw_update_chan(hw, codec_stream);
		}
	}
}

static void dpcm_runtime_setup_be_rate(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_pcm_hardware *hw = &runtime->hw;
	struct snd_soc_dpcm *dpcm;
	int stream = substream->stream;

	if (!fe->dai_link->dpcm_merged_rate)
		return;

	/*
	 * It returns merged BE codec channel;
	 * if FE want to use it (= dpcm_merged_chan)
	 */

	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_soc_pcm_runtime *be = dpcm->be;
		const struct snd_soc_pcm_stream *pcm;
		struct snd_soc_dai *dai;
		int i;

		for_each_rtd_dais(be, i, dai) {
			/*
			 * Skip DAIs which don't support the current stream
			 * type. See soc_pcm_init_runtime_hw() for more details
			 */
			if (!snd_soc_dai_stream_valid(dai, stream))
				continue;

			pcm = snd_soc_dai_pcm_stream_get(dai, stream);

			snd_soc_pcm_hw_update_rate(hw, pcm);
		}
	}
}

static int dpcm_apply_symmetry(struct snd_pcm_substream *fe_substream,
			       int stream)
{
	struct snd_soc_dpcm *dpcm;
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(fe_substream);
	struct snd_soc_dai *fe_cpu_dai;
	int err = 0;
	int i;

	/* apply symmetry for FE */
	snd_soc_dai_symmetric_update(fe_substream);

	for_each_rtd_cpu_dais (fe, i, fe_cpu_dai) {
		/* Symmetry only applies if we've got an active stream. */
		err = snd_soc_dai_symmetric_apply(fe_substream, fe_cpu_dai);
		if (err < 0)
			goto error;
	}

	/* apply symmetry for BE */
	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_soc_pcm_runtime *be = dpcm->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);
		struct snd_soc_pcm_runtime *rtd;
		struct snd_soc_dai *dai;

		/* A backend may not have the requested substream */
		if (!be_substream)
			continue;

		rtd = snd_soc_substream_to_rtd(be_substream);
		if (rtd->dai_link->be_hw_params_fixup)
			continue;

		snd_soc_dai_symmetric_update(be_substream);

		/* Symmetry only applies if we've got an active stream. */
		for_each_rtd_dais(rtd, i, dai) {
			err = snd_soc_dai_symmetric_apply(fe_substream, dai);
			if (err < 0)
				goto error;
		}
	}
error:
	return soc_pcm_ret(fe, err);
}

static int dpcm_fe_dai_startup(struct snd_pcm_substream *fe_substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(fe_substream);
	int stream = fe_substream->stream, ret = 0;

	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_FE);

	ret = dpcm_be_dai_startup(fe, stream);
	if (ret < 0)
		goto be_err;

	dev_dbg(fe->dev, "ASoC: open FE %s\n", fe->dai_link->name);

	/* start the DAI frontend */
	ret = snd_soc_pcm_open_no_lock(fe, fe_substream);
	if (ret < 0)
		goto unwind;

	fe->dpcm[stream].state = SND_SOC_DPCM_STATE_OPEN;

	dpcm_runtime_setup_fe(fe_substream);

	dpcm_runtime_setup_be_format(fe_substream);
	dpcm_runtime_setup_be_chan(fe_substream);
	dpcm_runtime_setup_be_rate(fe_substream);

	ret = dpcm_apply_symmetry(fe_substream, stream);

unwind:
	if (ret < 0)
		dpcm_be_dai_startup_unwind(fe, stream);
be_err:
	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_NO);

	return soc_pcm_ret(fe, ret);
}

static int dpcm_fe_dai_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	int stream = substream->stream;

	snd_soc_dpcm_mutex_assert_held(fe);

	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_FE);

	/* shutdown the BEs */
	dpcm_be_dai_shutdown(fe, stream);

	dev_dbg(fe->dev, "ASoC: close FE %s\n", fe->dai_link->name);

	/* now shutdown the frontend */
	snd_soc_pcm_close_no_lock(fe, substream, 0);

	/* run the stream stop event */
	dpcm_dapm_stream_event(fe, stream, SND_SOC_DAPM_STREAM_STOP);

	fe->dpcm[stream].state = SND_SOC_DPCM_STATE_CLOSE;
	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_NO);
	return 0;
}

void dpcm_be_dai_hw_free(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm *dpcm;

	/* only hw_params backends that are either sinks or sources
	 * to this frontend DAI */
	for_each_dpcm_be(fe, stream, dpcm) {

		struct snd_soc_pcm_runtime *be = dpcm->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		/* is this op for this BE ? */
		if (!snd_soc_dpcm_can_be_update(fe, be, stream))
			continue;

		/* only free hw when no longer used - check all FEs */
		if (!snd_soc_dpcm_can_be_free_stop(fe, be, stream))
			continue;

		/* do not free hw if this BE is used by other FE */
		if (be->dpcm[stream].users > 1)
			continue;

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_PREPARE) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_FREE) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_PAUSED) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_STOP) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_SUSPEND))
			continue;

		dev_dbg(be->dev, "ASoC: hw_free BE %s\n",
			be->dai_link->name);

		snd_soc_pcm_hw_clean(be, be_substream, 0);

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_HW_FREE;
	}
}

static int dpcm_fe_dai_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	int stream = substream->stream;

	snd_soc_dpcm_mutex_lock(fe);
	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_FE);

	dev_dbg(fe->dev, "ASoC: hw_free FE %s\n", fe->dai_link->name);

	/* call hw_free on the frontend */
	snd_soc_pcm_hw_clean(fe, substream, 0);

	/* only hw_params backends that are either sinks or sources
	 * to this frontend DAI */
	dpcm_be_dai_hw_free(fe, stream);

	fe->dpcm[stream].state = SND_SOC_DPCM_STATE_HW_FREE;
	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_NO);

	snd_soc_dpcm_mutex_unlock(fe);
	return 0;
}

int dpcm_be_dai_hw_params(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_pcm_runtime *be;
	struct snd_pcm_substream *be_substream;
	struct snd_soc_dpcm *dpcm;
	int ret;

	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_pcm_hw_params hw_params;

		be = dpcm->be;
		be_substream = snd_soc_dpcm_get_substream(be, stream);

		/* is this op for this BE ? */
		if (!snd_soc_dpcm_can_be_update(fe, be, stream))
			continue;

		/* copy params for each dpcm */
		memcpy(&hw_params, &fe->dpcm[stream].hw_params,
		       sizeof(struct snd_pcm_hw_params));

		/* perform any hw_params fixups */
		ret = snd_soc_link_be_hw_params_fixup(be, &hw_params);
		if (ret < 0)
			goto unwind;

		/* copy the fixed-up hw params for BE dai */
		memcpy(&be->dpcm[stream].hw_params, &hw_params,
		       sizeof(struct snd_pcm_hw_params));

		/* only allow hw_params() if no connected FEs are running */
		if (!snd_soc_dpcm_can_be_params(fe, be, stream))
			continue;

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_OPEN) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_FREE))
			continue;

		dev_dbg(be->dev, "ASoC: hw_params BE %s\n",
			be->dai_link->name);

		ret = snd_soc_pcm_hw_params_no_lock(be_substream, &hw_params);
		if (ret < 0)
			goto unwind;

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_HW_PARAMS;
	}
	return 0;

unwind:
	dev_dbg(fe->dev, "ASoC: %s() failed at %s (%d)\n",
		__func__, be->dai_link->name, ret);

	/* disable any enabled and non active backends */
	for_each_dpcm_be_rollback(fe, stream, dpcm) {
		be = dpcm->be;
		be_substream = snd_soc_dpcm_get_substream(be, stream);

		if (!snd_soc_dpcm_can_be_update(fe, be, stream))
			continue;

		/* only allow hw_free() if no connected FEs are running */
		if (!snd_soc_dpcm_can_be_free_stop(fe, be, stream))
			continue;

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_OPEN) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_FREE) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_STOP))
			continue;

		snd_soc_pcm_hw_clean(be, be_substream, 0);
	}

	return soc_pcm_ret(fe, ret);
}

static int dpcm_fe_dai_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	int ret, stream = substream->stream;

	snd_soc_dpcm_mutex_lock(fe);
	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_FE);

	memcpy(&fe->dpcm[stream].hw_params, params,
	       sizeof(struct snd_pcm_hw_params));
	ret = dpcm_be_dai_hw_params(fe, stream);
	if (ret < 0)
		goto out;

	dev_dbg(fe->dev, "ASoC: hw_params FE %s rate %d chan %x fmt %d\n",
		fe->dai_link->name, params_rate(params),
		params_channels(params), params_format(params));

	/* call hw_params on the frontend */
	ret = snd_soc_pcm_hw_params_no_lock(substream, params);
	if (ret < 0)
		dpcm_be_dai_hw_free(fe, stream);
	else
		fe->dpcm[stream].state = SND_SOC_DPCM_STATE_HW_PARAMS;

out:
	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_NO);
	snd_soc_dpcm_mutex_unlock(fe);

	return soc_pcm_ret(fe, ret);
}

int dpcm_be_dai_trigger(struct snd_soc_pcm_runtime *fe, int stream,
			int cmd)
{
	struct snd_soc_pcm_runtime *be;
	bool pause_stop_transition;
	struct snd_soc_dpcm *dpcm;
	unsigned long flags;
	int ret = 0;

	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_pcm_substream *be_substream;

		be = dpcm->be;
		be_substream = snd_soc_dpcm_get_substream(be, stream);

		snd_pcm_stream_lock_irqsave_nested(be_substream, flags);

		/* is this op for this BE ? */
		if (!snd_soc_dpcm_can_be_update(fe, be, stream))
			goto next;

		dev_dbg(be->dev, "ASoC: trigger BE %s cmd %d\n",
			be->dai_link->name, cmd);

		switch (cmd) {
		case SNDRV_PCM_TRIGGER_START:
			if (!be->dpcm[stream].be_start &&
			    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_PREPARE) &&
			    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_STOP) &&
			    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_PAUSED))
				goto next;

			be->dpcm[stream].be_start++;
			if (be->dpcm[stream].be_start != 1)
				goto next;

			if (be->dpcm[stream].state == SND_SOC_DPCM_STATE_PAUSED)
				ret = snd_soc_pcm_trigger(be_substream,
							  SNDRV_PCM_TRIGGER_PAUSE_RELEASE);
			else
				ret = snd_soc_pcm_trigger(be_substream,
							  SNDRV_PCM_TRIGGER_START);
			if (ret) {
				be->dpcm[stream].be_start--;
				goto next;
			}

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_START;
			break;
		case SNDRV_PCM_TRIGGER_RESUME:
			if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_SUSPEND))
				goto next;

			be->dpcm[stream].be_start++;
			if (be->dpcm[stream].be_start != 1)
				goto next;

			ret = snd_soc_pcm_trigger(be_substream, cmd);
			if (ret) {
				be->dpcm[stream].be_start--;
				goto next;
			}

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_START;
			break;
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			if (!be->dpcm[stream].be_start &&
			    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_START) &&
			    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_PAUSED))
				goto next;

			fe->dpcm[stream].fe_pause = false;
			be->dpcm[stream].be_pause--;

			be->dpcm[stream].be_start++;
			if (be->dpcm[stream].be_start != 1)
				goto next;

			ret = snd_soc_pcm_trigger(be_substream, cmd);
			if (ret) {
				be->dpcm[stream].be_start--;
				goto next;
			}

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_START;
			break;
		case SNDRV_PCM_TRIGGER_STOP:
			if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_START) &&
			    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_PAUSED))
				goto next;

			if (be->dpcm[stream].state == SND_SOC_DPCM_STATE_START)
				be->dpcm[stream].be_start--;

			if (be->dpcm[stream].be_start != 0)
				goto next;

			pause_stop_transition = false;
			if (fe->dpcm[stream].fe_pause) {
				pause_stop_transition = true;
				fe->dpcm[stream].fe_pause = false;
				be->dpcm[stream].be_pause--;
			}

			if (be->dpcm[stream].be_pause != 0)
				ret = snd_soc_pcm_trigger(be_substream,
							  SNDRV_PCM_TRIGGER_PAUSE_PUSH);
			else
				ret = snd_soc_pcm_trigger(be_substream,
							  SNDRV_PCM_TRIGGER_STOP);

			if (ret) {
				if (be->dpcm[stream].state == SND_SOC_DPCM_STATE_START)
					be->dpcm[stream].be_start++;
				if (pause_stop_transition) {
					fe->dpcm[stream].fe_pause = true;
					be->dpcm[stream].be_pause++;
				}
				goto next;
			}

			if (be->dpcm[stream].be_pause != 0)
				be->dpcm[stream].state = SND_SOC_DPCM_STATE_PAUSED;
			else
				be->dpcm[stream].state = SND_SOC_DPCM_STATE_STOP;

			break;
		case SNDRV_PCM_TRIGGER_SUSPEND:
			if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_START)
				goto next;

			be->dpcm[stream].be_start--;
			if (be->dpcm[stream].be_start != 0)
				goto next;

			ret = snd_soc_pcm_trigger(be_substream, cmd);
			if (ret) {
				be->dpcm[stream].be_start++;
				goto next;
			}

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_SUSPEND;
			break;
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			if (be->dpcm[stream].state != SND_SOC_DPCM_STATE_START)
				goto next;

			fe->dpcm[stream].fe_pause = true;
			be->dpcm[stream].be_pause++;

			be->dpcm[stream].be_start--;
			if (be->dpcm[stream].be_start != 0)
				goto next;

			ret = snd_soc_pcm_trigger(be_substream, cmd);
			if (ret) {
				be->dpcm[stream].be_start++;
				goto next;
			}

			be->dpcm[stream].state = SND_SOC_DPCM_STATE_PAUSED;
			break;
		}
	next:
		snd_pcm_stream_unlock_irqrestore(be_substream, flags);
		if (ret)
			break;
	}
	return soc_pcm_ret(fe, ret);
}
EXPORT_SYMBOL_GPL(dpcm_be_dai_trigger);

static int dpcm_dai_trigger_fe_be(struct snd_pcm_substream *substream,
				  int cmd, bool fe_first)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	int ret;

	/* call trigger on the frontend before the backend. */
	if (fe_first) {
		dev_dbg(fe->dev, "ASoC: pre trigger FE %s cmd %d\n",
			fe->dai_link->name, cmd);

		ret = snd_soc_pcm_trigger(substream, cmd);
		if (ret < 0)
			goto end;

		ret = dpcm_be_dai_trigger(fe, substream->stream, cmd);
	}
	/* call trigger on the frontend after the backend. */
	else {
		ret = dpcm_be_dai_trigger(fe, substream->stream, cmd);
		if (ret < 0)
			goto end;

		dev_dbg(fe->dev, "ASoC: post trigger FE %s cmd %d\n",
			fe->dai_link->name, cmd);

		ret = snd_soc_pcm_trigger(substream, cmd);
	}
end:
	return snd_soc_ret(fe->dev, ret, "trigger FE cmd: %d failed\n", cmd);
}

static int dpcm_fe_dai_do_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	int stream = substream->stream;
	int ret = 0;
	int fe_first;
	enum snd_soc_dpcm_trigger trigger = fe->dai_link->trigger[stream];

	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_FE;

	switch (trigger) {
	case SND_SOC_DPCM_TRIGGER_PRE:
		fe_first = true;
		break;
	case SND_SOC_DPCM_TRIGGER_POST:
		fe_first = false;
		break;
	default:
		dev_err(fe->dev, "ASoC: invalid trigger cmd %d for %s\n", cmd,
			fe->dai_link->name);
		ret = -EINVAL;
		goto out;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_DRAIN:
		ret = dpcm_dai_trigger_fe_be(substream, cmd, fe_first);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		ret = dpcm_dai_trigger_fe_be(substream, cmd, !fe_first);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	if (ret < 0)
		goto out;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		fe->dpcm[stream].state = SND_SOC_DPCM_STATE_START;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		fe->dpcm[stream].state = SND_SOC_DPCM_STATE_STOP;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		fe->dpcm[stream].state = SND_SOC_DPCM_STATE_PAUSED;
		break;
	}

out:
	fe->dpcm[stream].runtime_update = SND_SOC_DPCM_UPDATE_NO;
	return soc_pcm_ret(fe, ret);
}

static int dpcm_fe_dai_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	int stream = substream->stream;

	/* if FE's runtime_update is already set, we're in race;
	 * process this trigger later at exit
	 */
	if (fe->dpcm[stream].runtime_update != SND_SOC_DPCM_UPDATE_NO) {
		fe->dpcm[stream].trigger_pending = cmd + 1;
		return 0; /* delayed, assuming it's successful */
	}

	/* we're alone, let's trigger */
	return dpcm_fe_dai_do_trigger(substream, cmd);
}

int dpcm_be_dai_prepare(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm *dpcm;
	int ret = 0;

	for_each_dpcm_be(fe, stream, dpcm) {

		struct snd_soc_pcm_runtime *be = dpcm->be;
		struct snd_pcm_substream *be_substream =
			snd_soc_dpcm_get_substream(be, stream);

		/* is this op for this BE ? */
		if (!snd_soc_dpcm_can_be_update(fe, be, stream))
			continue;

		if (!snd_soc_dpcm_can_be_prepared(fe, be, stream))
			continue;

		if ((be->dpcm[stream].state != SND_SOC_DPCM_STATE_HW_PARAMS) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_STOP) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_SUSPEND) &&
		    (be->dpcm[stream].state != SND_SOC_DPCM_STATE_PAUSED))
			continue;

		dev_dbg(be->dev, "ASoC: prepare BE %s\n",
			be->dai_link->name);

		ret = snd_soc_pcm_prepare_no_lock(be, be_substream);
		if (ret < 0)
			break;

		be->dpcm[stream].state = SND_SOC_DPCM_STATE_PREPARE;
	}

	/*
	 * Don't use soc_pcm_ret() on .prepare callback to lower error log severity
	 *
	 * We don't want to log an error since we do not want to give userspace a way to do a
	 * denial-of-service attack on the syslog / diskspace.
	 */
	return ret;
}

static int dpcm_fe_dai_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(substream);
	int stream = substream->stream, ret = 0;

	snd_soc_dpcm_mutex_lock(fe);

	dev_dbg(fe->dev, "ASoC: prepare FE %s\n", fe->dai_link->name);

	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_FE);

	ret = dpcm_be_dai_prepare(fe, stream);
	if (ret < 0)
		goto out;

	/* call prepare on the frontend */
	ret = snd_soc_pcm_prepare_no_lock(fe, substream);
	if (ret < 0)
		goto out;

	fe->dpcm[stream].state = SND_SOC_DPCM_STATE_PREPARE;

out:
	dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_NO);
	snd_soc_dpcm_mutex_unlock(fe);

	/*
	 * Don't use soc_pcm_ret() on .prepare callback to lower error log severity
	 *
	 * We don't want to log an error since we do not want to give userspace a way to do a
	 * denial-of-service attack on the syslog / diskspace.
	 */
	return ret;
}

static int dpcm_run_update_shutdown(struct snd_soc_pcm_runtime *fe, int stream)
{
	int err;

	dev_dbg(fe->dev, "ASoC: runtime %s close on FE %s\n",
		snd_pcm_direction_name(stream), fe->dai_link->name);

	err = dpcm_be_dai_trigger(fe, stream, SNDRV_PCM_TRIGGER_STOP);

	dpcm_be_dai_hw_free(fe, stream);

	dpcm_be_dai_shutdown(fe, stream);

	/* run the stream event for each BE */
	dpcm_dapm_stream_event(fe, stream, SND_SOC_DAPM_STREAM_NOP);

	return soc_pcm_ret(fe, err);
}

static int dpcm_run_update_startup(struct snd_soc_pcm_runtime *fe, int stream)
{
	struct snd_soc_dpcm *dpcm;
	int ret = 0;

	dev_dbg(fe->dev, "ASoC: runtime %s open on FE %s\n",
		snd_pcm_direction_name(stream), fe->dai_link->name);

	/* Only start the BE if the FE is ready */
	if (fe->dpcm[stream].state == SND_SOC_DPCM_STATE_HW_FREE ||
	    fe->dpcm[stream].state == SND_SOC_DPCM_STATE_CLOSE) {
		dev_err(fe->dev, "ASoC: FE %s is not ready %s\n",
			fe->dai_link->name, dpcm_state_string(fe->dpcm[stream].state));
		ret = -EINVAL;
		goto disconnect;
	}

	/* startup must always be called for new BEs */
	ret = dpcm_be_dai_startup(fe, stream);
	if (ret < 0)
		goto disconnect;

	/* keep going if FE state is > open */
	if (fe->dpcm[stream].state == SND_SOC_DPCM_STATE_OPEN)
		return 0;

	ret = dpcm_be_dai_hw_params(fe, stream);
	if (ret < 0)
		goto close;

	/* keep going if FE state is > hw_params */
	if (fe->dpcm[stream].state == SND_SOC_DPCM_STATE_HW_PARAMS)
		return 0;

	ret = dpcm_be_dai_prepare(fe, stream);
	if (ret < 0)
		goto hw_free;

	/* run the stream event for each BE */
	dpcm_dapm_stream_event(fe, stream, SND_SOC_DAPM_STREAM_NOP);

	/* keep going if FE state is > prepare */
	if (fe->dpcm[stream].state == SND_SOC_DPCM_STATE_PREPARE ||
	    fe->dpcm[stream].state == SND_SOC_DPCM_STATE_STOP)
		return 0;

	ret = dpcm_be_dai_trigger(fe, stream, SNDRV_PCM_TRIGGER_START);
	if (ret < 0)
		goto hw_free;

	return 0;

hw_free:
	dpcm_be_dai_hw_free(fe, stream);
close:
	dpcm_be_dai_shutdown(fe, stream);
disconnect:
	/* disconnect any pending BEs */
	for_each_dpcm_be(fe, stream, dpcm) {
		struct snd_soc_pcm_runtime *be = dpcm->be;

		/* is this op for this BE ? */
		if (!snd_soc_dpcm_can_be_update(fe, be, stream))
			continue;

		if (be->dpcm[stream].state == SND_SOC_DPCM_STATE_CLOSE ||
		    be->dpcm[stream].state == SND_SOC_DPCM_STATE_NEW)
			dpcm->state = SND_SOC_DPCM_LINK_STATE_FREE;
	}

	return soc_pcm_ret(fe, ret);
}

static int soc_dpcm_fe_runtime_update(struct snd_soc_pcm_runtime *fe, int new)
{
	struct snd_soc_dapm_widget_list *list;
	int stream;
	int count, paths;

	if (!fe->dai_link->dynamic)
		return 0;

	if (fe->dai_link->num_cpus > 1)
		return snd_soc_ret(fe->dev, -EINVAL, "doesn't support Multi CPU yet\n");

	/* only check active links */
	if (!snd_soc_dai_active(snd_soc_rtd_to_cpu(fe, 0)))
		return 0;

	/* DAPM sync will call this to update DSP paths */
	dev_dbg(fe->dev, "ASoC: DPCM %s runtime update for FE %s\n",
		new ? "new" : "old", fe->dai_link->name);

	for_each_pcm_streams(stream) {

		/* skip if FE doesn't have playback/capture capability */
		if (!snd_soc_dai_stream_valid(snd_soc_rtd_to_cpu(fe, 0),   stream) ||
		    !snd_soc_dai_stream_valid(snd_soc_rtd_to_codec(fe, 0), stream))
			continue;

		/* skip if FE isn't currently playing/capturing */
		if (!snd_soc_dai_stream_active(snd_soc_rtd_to_cpu(fe, 0), stream) ||
		    !snd_soc_dai_stream_active(snd_soc_rtd_to_codec(fe, 0), stream))
			continue;

		paths = dpcm_path_get(fe, stream, &list);
		if (paths < 0)
			return paths;

		/* update any playback/capture paths */
		/*
		 * Find the corresponding BE DAIs that source or sink audio to this
		 * FE substream.
		 */
		if (new)
			count = dpcm_add_paths(fe, stream, &list);
		else
			count = dpcm_prune_paths(fe, stream, &list);
		if (count) {
			dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_BE);
			if (new)
				dpcm_run_update_startup(fe, stream);
			else
				dpcm_run_update_shutdown(fe, stream);
			dpcm_set_fe_update_state(fe, stream, SND_SOC_DPCM_UPDATE_NO);

			dpcm_clear_pending_state(fe, stream);
			dpcm_be_disconnect(fe, stream);
		}

		dpcm_path_put(&list);
	}

	return 0;
}

/* Called by DAPM mixer/mux changes to update audio routing between PCMs and
 * any DAI links.
 */
int snd_soc_dpcm_runtime_update(struct snd_soc_card *card)
{
	struct device *dev = snd_soc_card_to_dev(card);
	struct snd_soc_pcm_runtime *fe;
	int ret = 0;

	snd_soc_dpcm_mutex_lock(card);
	/* shutdown all old paths first */
	for_each_card_rtds(card, fe) {
		ret = soc_dpcm_fe_runtime_update(fe, 0);
		if (ret)
			goto out;
	}

	/* bring new paths up */
	for_each_card_rtds(card, fe) {
		ret = soc_dpcm_fe_runtime_update(fe, 1);
		if (ret)
			goto out;
	}

out:
	snd_soc_dpcm_mutex_unlock(card);

	return snd_soc_ret(dev, ret, "failed\n");
}
EXPORT_SYMBOL_GPL(snd_soc_dpcm_runtime_update);

static void dpcm_fe_dai_cleanup(struct snd_pcm_substream *fe_substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(fe_substream);
	struct snd_soc_dpcm *dpcm;
	int stream = fe_substream->stream;

	snd_soc_dpcm_mutex_assert_held(fe);

	/* mark FE's links ready to prune */
	for_each_dpcm_be(fe, stream, dpcm)
		dpcm->state = SND_SOC_DPCM_LINK_STATE_FREE;

	dpcm_be_disconnect(fe, stream);
}

static int dpcm_fe_dai_close(struct snd_pcm_substream *fe_substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(fe_substream);
	int ret;

	snd_soc_dpcm_mutex_lock(fe);
	ret = dpcm_fe_dai_shutdown(fe_substream);

	dpcm_fe_dai_cleanup(fe_substream);

	snd_soc_dpcm_mutex_unlock(fe);

	return soc_pcm_ret(fe, ret);
}

static int dpcm_fe_dai_open(struct snd_pcm_substream *fe_substream)
{
	struct snd_soc_pcm_runtime *fe = snd_soc_substream_to_rtd(fe_substream);
	struct snd_soc_dapm_widget_list *list;
	int ret;
	int stream = fe_substream->stream;

	snd_soc_dpcm_mutex_lock(fe);

	ret = dpcm_path_get(fe, stream, &list);
	if (ret < 0)
		goto open_end;

	/* calculate valid and active FE <-> BE dpcms */
	dpcm_add_paths(fe, stream, &list);

	/* There is no point starting up this FE if there are no BEs. */
	if (list_empty(&fe->dpcm[stream].be_clients)) {
		/* dev_err_once() for visibility, dev_dbg() for debugging UCM profiles. */
		dev_err_once(fe->dev, "ASoC: no backend DAIs enabled for %s, possibly missing ALSA mixer-based routing or UCM profile\n",
			     fe->dai_link->name);
		dev_dbg(fe->dev, "ASoC: no backend DAIs enabled for %s\n", fe->dai_link->name);

		ret = -EINVAL;
		goto put_path;
	}

	ret = dpcm_fe_dai_startup(fe_substream);
	if (ret < 0)
		dpcm_fe_dai_cleanup(fe_substream);

	dpcm_clear_pending_state(fe, stream);
put_path:
	dpcm_path_put(&list);
open_end:
	snd_soc_dpcm_mutex_unlock(fe);

	return soc_pcm_ret(fe, ret);
}

static int dpcm_fe_new_pcm(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_card *snd_card = snd_soc_card_to_snd_card(rtd->card);
	struct snd_pcm *pcm;
	char new_name[64];
	int ret = 0, playback = 0, capture = 0;

	if (dai_link->num_cpus > 1)
		return snd_soc_ret(rtd->dev, -EINVAL,
				   "DPCM doesn't support Multi CPU for Front-Ends yet\n");

	snprintf(new_name, sizeof(new_name), "%s (*)", dai_link->stream_name);

	ret = snd_soc_pcm_detect_playback_capture(rtd, &playback, &capture, true);
	if (ret < 0)
		goto err;

	ret = snd_pcm_new(snd_card, new_name, rtd->id, playback, capture, &pcm);
	if (ret < 0)
		goto err;

	rtd->pcm			= pcm;
	rtd->close_delayed_work_func	= snd_soc_close_delayed_work;

	pcm->nonatomic			= dai_link->nonatomic;
	pcm->private_data		= rtd;
	pcm->no_device_suspend		= true;

	/* ASoC PCM operations */
	rtd->ops.open		= dpcm_fe_dai_open;
	rtd->ops.hw_params	= dpcm_fe_dai_hw_params;
	rtd->ops.prepare	= dpcm_fe_dai_prepare;
	rtd->ops.trigger	= dpcm_fe_dai_trigger;
	rtd->ops.hw_free	= dpcm_fe_dai_hw_free;
	rtd->ops.close		= dpcm_fe_dai_close;
	rtd->ops.pointer	= snd_soc_pcm_pointer;

	ret = snd_soc_pcm_new_common(rtd, playback, capture);
err:
	return soc_pcm_ret(rtd, ret);
}

int snd_soc_pcm_new_nopcm(struct snd_soc_pcm_runtime *rtd, char *name_prefix,
			  bool use_dummy_codec,
			  bool delayed_work)
{
	struct snd_soc_dai_link *dai_link = rtd->dai_link;
	struct snd_card *snd_card = snd_soc_card_to_snd_card(rtd->card);
	struct snd_pcm *pcm;
	char new_name[64];
	int ret = 0, playback = 0, capture = 0;

	snprintf(new_name, sizeof(new_name), "%s(%s)", name_prefix, dai_link->stream_name);

	ret = snd_soc_pcm_detect_playback_capture(rtd, &playback, &capture, use_dummy_codec);
	if (ret < 0)
		goto err;

	ret = snd_pcm_new_internal(snd_card, new_name, rtd->id, playback, capture, &pcm);
	if (ret < 0)
		goto err;

	/* DAPM dai link stream work */
	/*
	 * Currently nothing to do for c2c links
	 * Since c2c links are internal nodes in the DAPM graph and
	 * don't interface with the outside world or application layer
	 * we don't have to do any special handling on close.
	 */
	if (delayed_work)
		rtd->close_delayed_work_func = snd_soc_close_delayed_work;

	rtd->pcm			= pcm;

	pcm->nonatomic			= dai_link->nonatomic;
	pcm->private_data		= rtd;
	pcm->no_device_suspend		= true;

	if (playback)
		pcm->streams[SNDRV_PCM_STREAM_PLAYBACK].substream->private_data	= rtd;
	if (capture)
		pcm->streams[SNDRV_PCM_STREAM_CAPTURE].substream->private_data	= rtd;

	ret = 0;
err:
	return soc_pcm_ret(rtd, ret);
}

static int dpcm_be_new_pcm(struct snd_soc_pcm_runtime *rtd)
{
	return snd_soc_pcm_new_nopcm(rtd, "", true, true);
}

struct snd_soc_rtd_ops rtd_ops_dpcm_fe = {
	.new_pcm	= dpcm_fe_new_pcm,
};

struct snd_soc_rtd_ops rtd_ops_dpcm_be = {
	.new_pcm	= dpcm_be_new_pcm,
};
