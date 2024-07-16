// SPDX-License-Identifier: GPL-2.0
//
// Renesas R-Car MSIOF (Clock-Synchronized Serial Interface with FIFO) I2S driver
//
// Copyright (C) 2024 Renesas Solutions Corp.
// Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//
// Based on fsi.c
// Copyright (c) 2009 Kuninori Morimoto <morimoto.kuninori@renesas.com>
// Referenced to mca.c
// Copyright (C) The Asahi Linux Contributors

/* FIXME
 * Note
 *
 * MSIOF is used as SPI or I2S (Sound), and because of it, Datasheet indicates
 * "It supports Provider (= Master) Mode". But RX (= Capture) can't provide clock/frame.
 * This driver support Consumer Mode only.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <sound/dmaengine_pcm.h>
#include <sound/soc.h>

/* register */
#define SITMDR1		0x00
#define SITMDR2		0x04
#define SITMDR3		0x08
#define SIRMDR1		0x10
#define SIRMDR2		0x14
#define SIRMDR3		0x18
#define SITSCR		0x20
#define SICTR		0x28
#define SIFCTR		0x30
#define SISTR		0x40
#define SIIER		0x44
#define SITFDR		0x50
#define SIRFDR		0x60

/* SITMDR1/ SIRMDR1 */
#define TRMD		(1 << 31)	/* Transfer Mode (1 = Master mode) */
#define PCON		(1 << 30)	/* Transfer Signal Connection */
#define SYNCMD_SPI	(2 << 28)	/* Level mode/SPI */
#define SYNCMD_LR	(3 << 28)	/* L/R mode */
#define SYNCAC		(1 << 25)	/* Sync Polarity (Active-low) */
#define DTDL_1		(1 << 20)	/* 1-clock-cycle delay */
#define TXSTP		(1 <<  0)	/* Transmission/Reception Stop on FIFO */

/* SICTR */
#define TEDG		(1 << 27)	/* Transmit Timing (1 = falling edge) */
#define REDG		(1 << 26)	/* Receive  Timing (1 = rising  edge) */
#define TSCKE		(1 << 15)	/* Transmit Serial Clock Output Enable */
#define TFSE		(1 << 14)	/* Transmit Frame Sync Signal Output Enable */
#define TXE		(1 <<  9)	/* Transmit Enable */
#define RXE		(1 <<  8)	/* Receive Enable */
#define TXRST		(1 <<  1)	/* Transmit Reset */
#define RXRST		(1 <<  0)	/* Receive Reset */

/* SIIER */
#define TDMAE		(1 << 31)	/* Transmit Data DMA Transfer Req. Enable */
#define TDREQE		(1 << 28)	/* Transmit Data Transfer Request Enable */
#define RDMAE		(1 << 15)	/* Receive Data DMA Transfer Req. Enable */
#define RDREQE		(1 << 12)	/* Receive Data Transfer Request Enable */

/* spec */
#define MSIOF_RATES	SNDRV_PCM_RATE_8000_192000
#define MSIOF_FMTS	(SNDRV_PCM_FMTBIT_S24_LE |\
			 SNDRV_PCM_FMTBIT_S32_LE)

struct msiof_priv {
	struct device *dev;
	spinlock_t lock;
	void __iomem *base;
	resource_size_t phy_addr;
	int fifo_size[SNDRV_PCM_STREAM_LAST + 1];
	u32 flag;
#define MSIOF_FLAG_CLK_PROVIDER		(1 << 0)
#define MSIOF_FLAG_DATA_DELAY		(1 << 1)
};

#define msiof_flag_set(p, f) ((p)->flag |=  (f))
#define msiof_flag_has(p, f) ((p)->flag &   (f))
#define msiof_flag_del(p, f) ((p)->flag &= ~(f))

static int msiof_is_play(struct snd_pcm_substream *substream)
{
	return substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
}

static void snd_soc_component_update_and_wait(struct snd_soc_component *component, unsigned int reg,
					      u32 mask, u32 val, u32 expect)
{
	struct msiof_priv *priv = snd_soc_component_get_drvdata(component);

	snd_soc_component_update_bits(component, reg, mask, val);

	for (int i = 0; i < 128; i++) {
		if ((snd_soc_component_read(component, reg) & mask) == expect)
			return;
		udelay(NSEC_PER_USEC);
	}

	dev_warn(priv->dev, "write timeout [%02x] = %08x\n", reg, val);
}

static int msiof_hw_start(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct msiof_priv *priv = snd_soc_component_get_drvdata(component);
	int is_play = msiof_is_play(substream);
	int is_provider = msiof_flag_has(priv, MSIOF_FLAG_CLK_PROVIDER);
	u32 val;

// FIXME working check ?
	/* SICTR reset */
	val = is_play ? TXRST : RXRST;
	snd_soc_component_update_and_wait(component, SICTR, val, val, 0);

	/* SITMDR1 */
	if (is_play) {
		val = PCON | SYNCMD_LR | SYNCAC | TXSTP;
		if (is_provider)
			val |= TRMD;
		if (msiof_flag_has(priv, MSIOF_FLAG_DATA_DELAY))
			val |= DTDL_1;
		snd_soc_component_write(component, SITMDR1, val);
	}
	/* SIRMDR1 */
	else {
		val = SYNCMD_LR | SYNCAC;
		if (msiof_flag_has(priv, MSIOF_FLAG_DATA_DELAY))
			val |= DTDL_1;
		snd_soc_component_write(component, SIRMDR1, val);
	}

// SITMDR2
//	priv->fifo_size[x]

// SIFCTR

// FIXME
	if (is_provider || is_play)
		val = TDREQE | TDMAE;
	if (!is_play)
		val = RDREQE | RDMAE;
	snd_soc_component_update_bits(component, SIIER, val, val);

	/*
	 * see
	 *	Datasheet 59.3.6 [Transmit and Receive Procedures]
	 *	SICTR :: TSCKE
	 */
	if (is_provider)
		snd_soc_component_update_and_wait(component, SICTR, TSCKE, TSCKE, TSCKE);
// FIXME
// 多分 TXE/RXE はラストに書けばいい
// チェック不要 stop も同様
	if (is_provider || is_play) {
		val = TEDG | TXE;
		snd_soc_component_update_and_wait(component, SICTR, val,   val,   val);
	}
	if (!is_play) {
		val = REDG | RXE;
		snd_soc_component_update_and_wait(component, SICTR, val,   val,   val);
	}
	if (is_provider)
		snd_soc_component_update_and_wait(component, SICTR, TFSE,  TFSE,  TFSE);

	return 0;
}

static int msiof_hw_stop(struct snd_soc_component *component, struct snd_pcm_substream *substream)
{
	struct msiof_priv *priv = snd_soc_component_get_drvdata(component);
	int is_play = msiof_is_play(substream);
	int is_provider = msiof_flag_has(priv, MSIOF_FLAG_CLK_PROVIDER);
	u32 val;

// FIXME working check ?
	if (is_provider || is_play)
		val = TDREQE | TDMAE;
	if (!is_play)
		val = RDREQE | RDMAE;
	snd_soc_component_update_bits(component, SIIER, val, 0);

	/*
	 * see
	 *	Datasheet 59.3.6 [Transmit and Receive Procedures]
	 *	SICTR :: TSCKE
	 */
// FIXME working check ?
	if (is_provider)
		snd_soc_component_update_and_wait(component, SICTR, TFSE,  0, 0);
	if (is_provider || is_play)
		snd_soc_component_update_and_wait(component, SICTR, TXE,   0, 0);
	if (!is_play)
		snd_soc_component_update_and_wait(component, SICTR, RXE,   0, 0);
	if (is_provider)
		snd_soc_component_update_and_wait(component, SICTR, TSCKE, 0, 0);

	return 0;
}

static int msiof_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct msiof_priv *priv = snd_soc_dai_get_drvdata(dai);

	switch (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) {
	case SND_SOC_DAIFMT_BP_FP:
		msiof_flag_set(priv, MSIOF_FLAG_CLK_PROVIDER); /* provider */
		break;
	case SND_SOC_DAIFMT_BC_FC:
		msiof_flag_del(priv, MSIOF_FLAG_CLK_PROVIDER); /* consumer */
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		/* it supports NB_NF only */
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		msiof_flag_set(priv, MSIOF_FLAG_DATA_DELAY);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		msiof_flag_del(priv, MSIOF_FLAG_DATA_DELAY);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/* FIXME
 * Select below from Sound Card, not auto
 *	SND_SOC_DAIFMT_CBC_CFC
 *	SND_SOC_DAIFMT_CBP_CFP
 */
static const u64 msiof_dai_formats = SND_SOC_POSSIBLE_DAIFMT_I2S	|
				     SND_SOC_POSSIBLE_DAIFMT_LEFT_J	|
				     SND_SOC_POSSIBLE_DAIFMT_NB_NF;

static const struct snd_soc_dai_ops msiof_dai_ops = {
	.set_fmt			= msiof_dai_set_fmt,
	.auto_selectable_formats	= &msiof_dai_formats,
	.num_auto_selectable_formats	= 1,
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

static struct snd_pcm_hardware msiof_pcm_hardware = {
	.info =	SNDRV_PCM_INFO_INTERLEAVED	|
		SNDRV_PCM_INFO_MMAP		|
		SNDRV_PCM_INFO_MMAP_VALID,
	.buffer_bytes_max	= 64 * 1024,
	.period_bytes_min	= 32,
	.period_bytes_max	= 8192,
	.periods_min		= 1,
	.periods_max		= 32,
	.fifo_size		= 64,
};

static int msiof_open(struct snd_soc_component *component,
		      struct snd_pcm_substream *substream)
{
	struct device *dev = component->dev;
	struct dma_chan *chan;
	char *dma_names[] = {"rx", "tx"};
	int is_play = msiof_is_play(substream);
	int ret;

	chan = of_dma_request_slave_channel(dev->of_node, dma_names[is_play]);
	if (IS_ERR_OR_NULL(chan))
		return PTR_ERR(chan);

// snd_pcm_hw_constraint_list()

	ret = snd_dmaengine_pcm_open(substream, chan);
	if (ret < 0)
		goto open_err_dma;

	snd_soc_set_runtime_hwparams(substream, &msiof_pcm_hardware);

	ret = snd_pcm_hw_constraint_integer(substream->runtime, SNDRV_PCM_HW_PARAM_PERIODS);

open_err_dma:
	if (ret < 0)
		dma_release_channel(chan);

	return ret;
}

static int msiof_close(struct snd_soc_component *component,
		       struct snd_pcm_substream *substream)
{
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

static int msiof_trigger(struct snd_soc_component *component,
			 struct snd_pcm_substream *substream, int cmd)
{
	struct msiof_priv *priv = dev_get_drvdata(component->dev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&priv->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
		msiof_hw_start(component, substream);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		msiof_hw_stop(component, substream);
	}

	ret = snd_dmaengine_pcm_trigger(substream, cmd);

	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static int msiof_hw_params(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params)
{
	struct msiof_priv *priv = dev_get_drvdata(component->dev);
	struct dma_chan *chan = snd_dmaengine_pcm_get_chan(substream);
	struct dma_slave_config cfg = {};
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&priv->lock, flags);

	ret = snd_hwparams_to_dma_slave_config(substream, params, &cfg);
	if (ret < 0)
		goto hw_params_out;

	cfg.src_addr		= priv->phy_addr + SIRFDR;
	cfg.dst_addr		= priv->phy_addr + SITFDR;
	cfg.dst_addr_width	= DMA_SLAVE_BUSWIDTH_4_BYTES;
	cfg.src_addr_width	= DMA_SLAVE_BUSWIDTH_4_BYTES;

	ret = dmaengine_slave_config(chan, &cfg);
hw_params_out:
	spin_unlock_irqrestore(&priv->lock, flags);

	return ret;
}

static unsigned int msiof_read(struct snd_soc_component *component, unsigned int reg)
{
	struct msiof_priv *priv = dev_get_drvdata(component->dev);

// FIXME SITSCR is not needed ?
	if (reg == SITSCR)
		return ioread16(priv->base + reg);
	else
		return ioread32(priv->base + reg);
}

static int msiof_write(struct snd_soc_component *component, unsigned int reg, unsigned int val)
{
	struct msiof_priv *priv = dev_get_drvdata(component->dev);
// FIXME SITSCR is not needed ?
	if (reg == SITSCR)
		iowrite16(val, priv->base + reg);
	else
		iowrite32(val, priv->base + reg);

	return 0;
}

static const struct snd_soc_component_driver msiof_component_driver = {
	.name			= "msiof",
	.open			= msiof_open,
	.close			= msiof_close,
	.pointer		= msiof_pointer,
	.pcm_construct		= msiof_new,
	.trigger		= msiof_trigger,
	.hw_params		= msiof_hw_params,
	.write			= msiof_write,
	.read			= msiof_read,
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

	priv->dev	= dev;
	priv->phy_addr	= res->start;
	priv->fifo_size[SNDRV_PCM_STREAM_PLAYBACK] = 64; /* default */
	priv->fifo_size[SNDRV_PCM_STREAM_CAPTURE]  = 64; /* default */
	spin_lock_init(&priv->lock);
	platform_set_drvdata(pdev, priv);

	of_property_read_u32(dev->of_node, "renesas,tx-fifo-size",
			     &priv->fifo_size[SNDRV_PCM_STREAM_PLAYBACK]);
	of_property_read_u32(dev->of_node, "renesas,rx-fifo-size",
			     &priv->fifo_size[SNDRV_PCM_STREAM_CAPTURE]);

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
