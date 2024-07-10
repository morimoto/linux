// SPDX-License-Identifier: GPL-2.0
//
// Renesas R-Car MSIOF (Clock-Synchronized Serial Interface with FIFO) I2S driver
//
// Copyright (C) 2024 Renesas Solutions Corp.
// Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//
// Based on fsi.c
// Copyright (c) 2009 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>

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

struct msiof_priv {
	struct platform_device *pdev;
	spinlock_t lock;
	void __iomem *base;
};

static snd_pcm_uframes_t msiof_pointer(struct snd_soc_component *component,
				       struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	struct rsnd_dma *dma = rsnd_mod_to_dma(mod);
	struct rsnd_dmaen *dmaen = rsnd_dma_to_dmaen(dma);
	struct dma_tx_state state;
	enum dma_status status;
	unsigned int pos = 0;

	status = dmaengine_tx_status(dmaen->chan, dmaen->cookie, &state);
	if (status == DMA_IN_PROGRESS || status == DMA_PAUSED) {
		if (state.residue > 0 && state.residue <= dmaen->dma_len)
			pos = dmaen->dma_len - state.residue;
	}
	*pointer = bytes_to_frames(runtime, pos);

	return 0;
}

#define PREALLOC_BUFFER		(32 * 1024)
#define PREALLOC_BUFFER_MAX	(32 * 1024)
static int msiof_pcm_new(struct snd_soc_component *component,
			 struct snd_soc_pcm_runtime *rtd)
{
	snd_pcm_set_managed_buffer_all(
		rtd->pcm,
		SNDRV_DMA_TYPE_DEV,
		rtd->card->snd_card->dev,
		PREALLOC_BUFFER, PREALLOC_BUFFER_MAX);
	return 0;
}

static struct snd_soc_dai_driver msiof_dai = {
	.name			= "msiof-dai",
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

static const struct snd_soc_component_driver msiof_component = {
	.name		= "msiof",
	.open		= msiof_pcm_open,
	.pointer	= msiof_pointer,
	.pcm_construct	= msiof_pcm_new,
};

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

	pm_runtime_enable(dev);

	ret = devm_snd_soc_register_component(dev, &msiof_component,
					      msiof_dai, ARRAY_SIZE(msiof_dai));
	if (ret < 0) {
		dev_err(&pdev->dev, "cannot snd component register\n");
		goto exit_fsib;
	}

	return ret;


	return 0;
}

static void msiof_remove(struct platform_device *pdev)
{
	
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
	.remove_new	= msiof_remove,
};
module_platform_driver(msiof_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas R-Car MSIOF I2S audio driver");
MODULE_AUTHOR("Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>");
MODULE_ALIAS("platform:msiof-pcm-audio");
