// SPDX-License-Identifier: GPL-2.0
//
// Renesas R-Car MSIOF (Clock-Synchronized Serial Interface with FIFO) I2S driver
//
// Copyright (C) 2024 Renesas Solutions Corp.
// Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//
// Based on fsi.c
// Copyright (c) 2009 Kuninori Morimoto <morimoto.kuninori@renesas.com>

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>

/* register */
#define SITMDR1		0000
#define SITMDR2		0004
#define SITMDR3		0008
#define SIRMDR1		0010
#define SIRMDR2		0014
#define SIRMDR3		0018
#define SITSCR		0020
#define SICTR		0028
#define SIFCTR		0030
#define SISTR		0040
#define SIIER		0044
#define SITFDR		0050
#define SIRFDR		0060

/* spec */
#define MSIOF_RATES	SNDRV_PCM_RATE_8000_192000
#define MSIOF_FMTS	(SNDRV_PCM_FMTBIT_S16_LE |\
			 SNDRV_PCM_FMTBIT_S24_LE)

struct msiof_priv {
	struct platform_device *pdev;
	spinlock_t lock;
	void __iomem *base;
	struct dma_chan *dma_chan[SNDRV_PCM_STREAM_LAST + 1];
};

static int msiof_is_play(struct snd_pcm_substream *substream)
{
	return substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
}

static const struct snd_soc_dai_ops msiof_dai_ops = {
/*
	.startup	= fsi_dai_startup,
	.shutdown	= fsi_dai_shutdown,
	.trigger	= fsi_dai_trigger,
	.set_fmt	= fsi_dai_set_fmt,
	.hw_params	= fsi_dai_hw_params,
	.auto_selectable_formats	= &fsi_dai_formats,
	.num_auto_selectable_formats	= 1,
*/
};

static struct snd_soc_dai_driver msiof_dai_driver = {
	.name = "msiof-dai",
	.playback = {
		.rates		= MSIOF_RATES,
		.formats	= MSIOF_FMTS,
		.channels_min	= 2,
		.channels_max	= 2,
	},
	.capture = {
		.rates		= MSIOF_RATES,
		.formats	= MSIOF_FMTS,
		.channels_min	= 2,
		.channels_max	= 2,
	},
	.ops = &msiof_dai_ops,
};

static int msiof_open(struct snd_soc_component *component,
		      struct snd_pcm_substream *substream)
{
	struct device *dev = component->dev;
	struct msiof_priv *priv = dev_get_drvdata(dev);
	struct dma_chan *chan;
	int is_play = msiof_is_play(substream);
	char *dma_names[] = {"rx", "tx"};

	chan = of_dma_request_slave_channel(dev->of_node, dma_names[is_play]);

	if (IS_ERR_OR_NULL(chan))
		return PTR_ERR(chan);

	priv->dma_chan[is_play] = chan;

	return snd_dmaengine_pcm_open(substream, chan);
}

static int msiof_close(struct snd_soc_component *component,
		       struct snd_pcm_substream *substream)
{
	struct msiof_priv *priv = dev_get_drvdata(component->dev);
	int is_play = msiof_is_play(substream);

	priv->dma_chan[is_play] = NULL;

	return snd_dmaengine_pcm_close_release_chan(substream);
}

static snd_pcm_uframes_t msiof_pointer(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	return snd_dmaengine_pcm_pointer(substream);
}

#define PREALLOC_BUFFER		(32 * 1024)
#define PREALLOC_BUFFER_MAX	(32 * 1024)
static int msiof_new(struct snd_soc_component *component,
		     struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV,
				       rtd->card->snd_card->dev,
				       PREALLOC_BUFFER, PREALLOC_BUFFER_MAX);
	return 0;
}

static const struct snd_soc_component_driver msiof_component_driver = {
	.name			= "msiof",
	.open			= msiof_open,
	.close			= msiof_close,
	.pointer		= msiof_pointer,
	.pcm_construct		= msiof_new,
};

static irqreturn_t msiof_interrupt(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int msiof_probe(struct platform_device *pdev)
{
	struct msiof_priv *priv;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int irq, ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	irq = platform_get_irq(pdev, 0);
	if (irq <= 0)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENODEV;

	priv->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	ret = devm_request_irq(dev, irq, msiof_interrupt, 0, dev_name(dev), priv);
	if (ret)
		return ret;

	priv->pdev	= pdev;
	spin_lock_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

	devm_pm_runtime_enable(dev);

	ret = devm_snd_soc_register_component(dev, &msiof_component_driver,
					      &msiof_dai_driver, 1);

	return ret;
}

static const struct of_device_id msiof_of_match[] = {
	{ .compatible = "renesas,msiof_sound-gen3", },
	{},
};
MODULE_DEVICE_TABLE(of, msiof_of_match);

static struct platform_driver msiof_driver = {
	.driver 	= {
		.name	= "msiof-pcm-audio",
//		.pm	= &fsi_pm_ops,
		.of_match_table = msiof_of_match,
	},
	.probe		= msiof_probe,
//	.remove_new	= msiof_remove,
};
module_platform_driver(msiof_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas R-Car MSIOF I2S audio driver");
MODULE_AUTHOR("Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>");
MODULE_ALIAS("platform:msiof-pcm-audio");
