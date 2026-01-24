// SPDX-License-Identifier: GPL-2.0-only
/*
 * Renesas R-Car MFIS (Multifunctional Interface) driver
 *
 * Copyright (C) 2025 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 * Wolfram Sang <wsa+renesas@sang-engineering.com>
 */
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mailbox_controller.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>


#define MFISWPCNTR	0x900
#define MFISWACNTR	0x904

struct mfis_priv;

struct mfis_reg {
	void __iomem *base;
	resource_size_t start;
	struct mfis_priv *priv;
};

/* for Mailbox */
struct mfis_mb_chan {
	unsigned int chan_map;
	unsigned int reg;
};

struct mfis_info {
	u32 unprotect_mask;
	u32 flags;
#define MFIS_INFO_MAILBOX_IN_COMMON	BIT(1)
};

struct mfis_priv {
	struct device *dev;
	struct mfis_reg common_reg;
	struct mfis_reg mbox_reg;
	const struct mfis_info *info;

	/*
	 * private data for Mailbox
	 *	m${name}
	 */
	struct mbox_controller mbox;
	struct mfis_mb_chan *mchan;
	unsigned int mnum;		// used channel
};

/****************************************************
 *
 *		Common
 *
 ****************************************************/
#if 0 /* is not used so far */
static int mfis_read(void *context, unsigned int reg, unsigned int *val)
{
	struct mfis_reg *mreg = context;

	*val = ioread32(mreg->base + reg);
	return 0;
}
#endif

#define MFIS_UNPROTECT_KEY 0xACCE0000
static int mfis_write(struct mfis_reg *mreg, unsigned int reg, unsigned int val)
{
	struct mfis_priv *priv = mreg->priv;
	u32 unprotect_mask = priv->info->unprotect_mask;
	u32 unprotect_code;

	/*
	 * [Gen4] key: 0xACCE0000, mask: 0x0000FFFF
	 * [Gen5] key: 0xACC00000, mask: 0x000FFFFF
	 */
	unprotect_code = (MFIS_UNPROTECT_KEY & ~unprotect_mask) |
			 ((mreg->start | reg) & unprotect_mask);

	iowrite32(unprotect_code, priv->common_reg.base + MFISWACNTR);
	iowrite32(val, mreg->base + reg);

	return 0;
}

/****************************************************
 *
 *			HW Spinlock
 *
 * TBD
 ****************************************************/

/****************************************************
 *
 *			Mailbox
 *
 * [Control Direction]
 *
 *	<MFIS>			<MFIS-SCP>
 *
 *	+--+	     +--+	+--+	     +---+
 *	|  | <-[I]-> |  |	|  | <-[I]-- |   |
 *	|AP|	     |RT|	|AP|	     |SCP|
 *	|  | <-[E]-> |  |	|  | --[E]-> |   |
 *	+--+	     +--+	+--+	     +---+
 *
 * 1ch has 2 controls (= [I], [E])
 *	(A) MFISASIICR
 *	(B) MFISASEICR
 *		  ^
 * The diff between [I] and [E] is whether use interrupt or not.
 * RX side control needs to enable interrupt.
 * It is not fixed in MFIS     (= I or E)
 * It is     fixed in MFIS-SCP (= I)
 ****************************************************/
#define MFIS_MB_NUM_DIRECTION 2
static const char mfis_mb_irq_direction_suffix[MFIS_MB_NUM_DIRECTION] = {
	'i', 'e',
};

#define MFIS_MASK_CHAN	GENMASK(31, 16)
#define MFIS_MASK_MAP	GENMASK(15, 0)

#define mfis_mb_chan_map(chan, map)	FIELD_PREP(MFIS_MASK_CHAN, chan) +\
					FIELD_PREP(MFIS_MASK_MAP,  map)
#define mfis_mb_get_chan(param)		FIELD_GET(MFIS_MASK_CHAN, param)
#define mfis_mb_get_map(param)		FIELD_GET(MFIS_MASK_MAP,  param)

#define mfis_mb_mbox_to_priv(_m)	container_of((_m), struct mfis_priv, mbox)

#define mfis_mb_chan_to_mchan(chan)	chan->con_priv
#define mfis_mb_chan_to_mbox(chan)	chan->mbox

static irqreturn_t mfis_mb_interrupt(int irq, void *data)
{
	struct mbox_chan *chan = data;
	struct mbox_controller *mbox = mfis_mb_chan_to_mbox(chan);
	struct mfis_priv *priv = mfis_mb_mbox_to_priv(mbox);
	struct mfis_mb_chan *mchan = mfis_mb_chan_to_mchan(chan);

	mbox_chan_received_data(chan, NULL);
	mfis_write(&priv->mbox_reg, mchan->reg, 0);

	return IRQ_HANDLED;
}

static int mfis_mb_send_data(struct mbox_chan *chan, void *data)
{
	struct mbox_controller *mbox = mfis_mb_chan_to_mbox(chan);
	struct mfis_priv *priv = mfis_mb_mbox_to_priv(mbox);
	struct mfis_mb_chan *mchan = mfis_mb_chan_to_mchan(chan);

	mfis_write(&priv->mbox_reg, mchan->reg, 1);

	return 0;
}

static bool mfis_mb_last_tx_done(struct mbox_chan *chan)
{
	return true;
}

static const struct mbox_chan_ops mfis_mb_chan_ops = {
	.send_data	= mfis_mb_send_data,
	.last_tx_done	= mfis_mb_last_tx_done
};

#define MFIS_MB_DIR_RX	0
#define MFIS_MB_DIR_TX	1
static struct mbox_chan *mfis_mb_of_xlate(struct mbox_controller *mbox,
					  const struct of_phandle_args *sp)
{
	struct mfis_priv *priv = mfis_mb_mbox_to_priv(mbox);
	struct mfis_mb_chan *mchan;
	struct mbox_chan *chan;
	unsigned int chan_map;
	unsigned int i;
	int ch, map, dir;

	/*
	 * check size
	 *
	 * priv->mnum:		Total consumed channels == next channel position
	 * mbox->num_chans:	Max size of available channels
	 */
	if (priv->mnum >= mbox->num_chans)
		return ERR_PTR(-EINVAL);

	/*
	 * ex) mboxes = <&mfis_scp x y z>;
	 */

	/* it must have 3 params (= x, y, z) */
	if (sp->args_count != 3)
		return ERR_PTR(-EINVAL);

	ch	 = sp->args[0];	/* x */
	map	 = sp->args[1];	/* y */
	dir	 = sp->args[2];	/* z */
	chan_map = mfis_mb_chan_map(ch, map);

	/* map (= y) must 0 (= i) or 1 (= e) */
	if (map >= MFIS_MB_NUM_DIRECTION)
		return ERR_PTR(-EINVAL);

	/* dir (= z) must 0 (= RX) or 1 (= TX) */
	if (dir >= MFIS_MB_NUM_DIRECTION)
		return ERR_PTR(-EINVAL);

	/*
	 * check duplicate use case
	 *
	 * "chan_map" (= pair of x / y) should be unique
	 */
	for (i = 0; i < priv->mnum; i++)
		if (priv->mchan[i].chan_map == chan_map)
			return ERR_PTR(-EINVAL);

	/*
	 * Next attach channel
	 *
	 * mchan: for R-Car MFIS driver
	 *  chan: for Mailbox Framwork
	 */
	mchan = priv->mchan + priv->mnum;
	chan  = mbox->chans + priv->mnum;

	/*
	 * RX only use IRQ
	 * see
	 *	[Control Direction]
	 */
	if (dir == MFIS_MB_DIR_RX) {
		struct device *dev = mbox->dev;
		char irqname[8];
		char suffix = mfis_mb_irq_direction_suffix[map];
		int irq, ret;

		/* "ch0i" or "ch0e" */
		scnprintf(irqname, sizeof(irqname), "ch%d%c", ch, suffix);

		/*
		 * get IRQ number and request it
		 */
		irq = of_irq_get_byname(dev->of_node, irqname);
		if (irq < 0)
			return ERR_PTR(-EINVAL);

		ret = request_irq(irq, mfis_mb_interrupt, 0, dev_name(dev), chan);
		if (ret < 0)
			return ERR_PTR(ret);
	}

	mchan->chan_map	= chan_map;
	mchan->reg	= (ch * 0x1000) + (map * 4);

	priv->mnum++;

	return chan;
}

static int mfis_mb_count_user(void)
{
	struct device_node *firmware = of_find_node_by_path("/firmware");
	struct device_node *node;
	int cnt = 0;

	/*
	 * It try to find "mboxes" user from "firmware" node,
	 * and count it
	 *
	 *	firmware {
	 *		scmi {
	 *			...
	 *			mboxes = <&mfis_scp 2 1 1>, <&mfis_scp 2 0 0>;
	 *			...
	 *		};
	 *	};
	 */
	for_each_available_child_of_node(firmware, node) {
		struct of_phandle_iterator it;
		int rc;

		of_for_each_phandle(&it, rc, node, "mboxes", "#mbox-cells", -1)
			cnt++;
	}

	return cnt;
}

static int mfis_mb_probe(struct mfis_priv *priv)
{
	struct device *dev = priv->dev;
	struct mfis_mb_chan *mchan;
	struct mbox_chan *chan;
	struct mbox_controller *mbox;
	int i;
	int chan_num = 0;

	chan_num = mfis_mb_count_user();

	chan  = devm_kcalloc(dev, chan_num, sizeof(*chan),  GFP_KERNEL);
	mchan = devm_kcalloc(dev, chan_num, sizeof(*mchan), GFP_KERNEL);
	if (!chan || !mchan)
		return -ENOMEM;

	mbox = &priv->mbox;

	for (i = 0; i < chan_num; i++) {
		chan[i].mbox		= mbox;
		chan[i].con_priv	= &mchan[i];
	}

	priv->mchan		= mchan;

	mbox->chans		= chan;
	mbox->num_chans		= chan_num;
	mbox->txdone_poll	= true;
	mbox->txdone_irq	= false;
	mbox->txpoll_period	= 1;
	mbox->ops		= &mfis_mb_chan_ops;
	mbox->dev		= dev;
	mbox->of_xlate		= mfis_mb_of_xlate;

	return mbox_controller_register(mbox);
}

/****************************************************
 *
 *		Common
 *
 ****************************************************/
static char *mfis_res_name_common = "common";
static char *mfis_res_name_mboxes = "mboxes";

static int mfis_reg_probe(struct platform_device *pdev, struct mfis_priv *priv,
			  struct mfis_reg *mreg, const char *name)
{
	struct resource *res;
	void __iomem *base;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);

	/* If there is no mailbox resource, registers are in the common space */
	if (!res && name == mfis_res_name_mboxes) {
		if (priv->info->flags & MFIS_INFO_MAILBOX_IN_COMMON)
			priv->mbox_reg = priv->common_reg;
	} else {
		base = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(base))
			return PTR_ERR(base);

		mreg->base	= base;
		mreg->start	= res->start;
		mreg->priv	= priv;
	}

	return 0;
}

static int mfis_probe(struct platform_device *pdev)
{
	struct mfis_priv *priv;
	struct device *dev = &pdev->dev;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev	= dev;
	priv->info	= of_device_get_match_data(dev);

	/* for reg-names = "common" */
	ret = mfis_reg_probe(pdev, priv, &priv->common_reg, mfis_res_name_common);
	if (ret)
		return ret;

	/* for reg-names = "mboxes" */
	ret = mfis_reg_probe(pdev, priv, &priv->mbox_reg, mfis_res_name_mboxes);
	if (ret)
		return ret;

	return mfis_mb_probe(priv);
}

struct mfis_info mfis_info_gen4 = {
	.unprotect_mask	= 0x0000ffff,
	.flags = MFIS_INFO_MAILBOX_IN_COMMON,
};

struct mfis_info mfis_info_gen5 = {
	.unprotect_mask	= 0x000fffff,
};

static const struct of_device_id mfis_mfd_of_match[] = {
	{ .compatible = "renesas,rcar-gen4-mfis",	.data = &mfis_info_gen4, },
	{ .compatible = "renesas,r8a78000-mfis-scp",	.data = &mfis_info_gen5, },
	{ .compatible = "renesas,r8a78000-mfis",	.data = &mfis_info_gen5, },
	{ .compatible = "renesas,rcar-gen5-mfis-scp",	.data = &mfis_info_gen5, },	/* will be changed */
	{ .compatible = "renesas,rcar-gen5-mfis",	.data = &mfis_info_gen5, },	/* will be changed */
	{}
};
MODULE_DEVICE_TABLE(of, mfis_mfd_of_match);

static struct platform_driver mfis_driver = {
	.driver = {
		.name = "rcar-mfis",
		.of_match_table = mfis_mfd_of_match,
	},
	.probe	= mfis_probe,
};
module_platform_driver(mfis_driver);

MODULE_AUTHOR("Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>");
MODULE_AUTHOR("Wolfram Sang <wsa+renesas@sang-engineering.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Renesas R-Car MFIS driver");
