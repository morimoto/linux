// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Renesas I2C via R4
 *
 * Copyright (C) 2022 Renesas Solutions Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * This file is based on the drivers/i2c/busses/i2c-rcar.c
 * Copyright (C) Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 * Copyright (C) Wolfram Sang <wsa@sang-engineering.com>
 * Copyright (C) Renesas Electronics Corporation
 */
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/i2c.h>
#include <linux/i2c-smbus.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/slab.h>

#define RCAR_MIN_DMA_LEN	8

#define ID_P_HAS_R4_DMA		BIT(27)
#define ID_P_HOST_NOTIFY	BIT(28)
#define ID_P_PM_BLOCKED		BIT(31)

struct rcar_i2c_priv {
	u32 flags;
	void __iomem *io;
	struct i2c_adapter adap;
	struct i2c_msg *msg;
	int msg_idx;
	int msg_max;
	int irq_ret;

	wait_queue_head_t wait;

	struct resource *res;
	struct dma_chan *dma_tx;
	struct dma_chan *dma_rx;
	struct scatterlist sg;
	enum dma_data_direction dma_direction;

	struct reset_control *rstc;
	int irq;

	struct i2c_client *host_notify_client;
	struct r4_i2c_priv *r4;
};

#define rU32 unsigned int
#define rU8  unsigned char
#include "r4/r4_i2c_v1.h"
#define R4_TO_PRIV(r4)	(struct rcar_i2c_priv *)r4->private
static u32 r4_i2c_read(struct r4_i2c_priv *r4, int reg)
{
	struct rcar_i2c_priv *priv = R4_TO_PRIV(r4);

	return readl(priv->io + reg);
}

static void r4_i2c_write(struct r4_i2c_priv *r4, int reg, u32 val)
{
	struct rcar_i2c_priv *priv = R4_TO_PRIV(r4);

	writel(val, priv->io + reg);
}

static void r4_i2c_udelay(struct r4_i2c_priv *r4, unsigned int time)
{
	udelay(time);
}

#define CONFIG_R4_I2C_RECOVERY
#define CONFIG_R4_I2C_ATOMIC
#define CONFIG_R4_I2C_PIO
#define CONFIG_R4_I2C_DMA
#include "r4/r4_i2c_v1_rcar_gen3.c"

static int r4_err_to_rcar_err(int ret)
{
	if (ret < 0) {
		switch (ret) {
		case -R4I2CERR_NOTSUPPORTED:
			ret = -ENOTSUPP;
			break;
		case -R4I2CERR_INVAL:
			ret = -EINVAL;
			break;
		case -R4I2CERR_BUSY:
			ret = -EBUSY;
			break;
		case -R4I2CERR_TIMEOUT:
			ret = -ETIMEDOUT;
			break;
		case -R4I2CERR_ARBLOST:
			ret = -EAGAIN;
			break;
		case -R4I2CERR_NACK:
			ret = -ENXIO;
			break;
		default:
			ret = -EIO;
		}
	}
	return ret;
}

#define rcar_i2c_priv_to_dev(p)		((p)->adap.dev.parent)

static int rcar_i2c_recovery(struct i2c_adapter *adap)
{
	struct rcar_i2c_priv *priv = i2c_get_adapdata(adap);
	int ret = r4_i2c_recovery(priv->r4);

	return r4_err_to_rcar_err(ret);
};

static struct i2c_bus_recovery_info rcar_i2c_rcvry = {
	.recover_bus = rcar_i2c_recovery,
};

static int rcar_i2c_request_r4_dma(struct rcar_i2c_priv *priv,
				   int is_read,
				   rU32 dma_addr,
				   rU32 dma_xfer_size)
{
	struct device *dev = rcar_i2c_priv_to_dev(priv);
	struct dma_chan *chan;
	struct dma_slave_config cfg;
	enum dma_slave_buswidth width;
	char *chan_name = (is_read) ? "rx" : "tx";
	int ret;

	switch (dma_xfer_size) {
	case 1:
		width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case 2:
		width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	case 4:
		width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	default:
		return -EINVAL;
	}

	memset(&cfg, 0, sizeof(cfg));
	if (is_read) {
		cfg.direction		= DMA_DEV_TO_MEM;
		cfg.src_addr		= priv->res->start + dma_addr;
		cfg.src_addr_width	= width;
	} else {
		cfg.direction		= DMA_MEM_TO_DEV;
		cfg.dst_addr		= priv->res->start + dma_addr;
		cfg.dst_addr_width	= width;
	}

	chan = dma_request_chan(dev, chan_name);
	if (IS_ERR(chan))
		return PTR_ERR(chan);

	ret = dmaengine_slave_config(chan, &cfg);
	if (ret < 0) {
		dma_release_channel(chan);
		return ret;
	}

	if (is_read)
		priv->dma_rx = chan;
	else
		priv->dma_tx = chan;

	return 0;
}

static void rcar_i2c_r4_dma_clean(struct rcar_i2c_priv *priv, bool terminate)
{
	struct dma_chan *chan = priv->dma_direction == DMA_FROM_DEVICE
		? priv->dma_rx : priv->dma_tx;

	if (priv->dma_direction == DMA_NONE)
		return;

	/* only allowed from thread context! */
	if (terminate)
		dmaengine_terminate_sync(chan);

	dma_unmap_single(chan->device->dev, sg_dma_address(&priv->sg),
			 sg_dma_len(&priv->sg), priv->dma_direction);

	priv->dma_direction = DMA_NONE;

	/* stop R4 DMA */
	r4_i2c_xfer_dma_stop(priv->r4);
}
static void rcar_i2c_r4_dma_callback(void *data)
{
	struct rcar_i2c_priv *priv = data;

	rcar_i2c_r4_dma_clean(priv, false);
}

static int rcar_i2c_r4_xfer_dma(struct rcar_i2c_priv *priv)
{
	struct dma_async_tx_descriptor *txdesc;
	struct i2c_msg *msg = priv->msg;
	bool read = msg->flags & I2C_M_RD;
	struct dma_chan *chan = read ? priv->dma_rx : priv->dma_tx;
	enum dma_data_direction dir = read ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	dma_addr_t dma_addr;
	dma_cookie_t cookie;
	int ret;
	u8 *buf;
	int len;

	/*
	 * Prepare DMA
	 */
	buf = priv->msg->buf;
	len = priv->msg->len;

	ret = r4_i2c_xfer_dma_setup(priv->r4,
				    priv->msg_idx, priv->msg_max,
				    i2c_8bit_addr_from_msg(priv->msg),
				    &buf, &len);
	if (ret < 0)
		return r4_err_to_rcar_err(ret);

	dma_addr = dma_map_single(chan->device->dev, buf, len, dir);
	if (dma_mapping_error(chan->device->dev, dma_addr))
		return -EIO;

	sg_dma_len(&priv->sg)		= len;
	sg_dma_address(&priv->sg)	= dma_addr;

	txdesc = dmaengine_prep_slave_sg(chan, &priv->sg, 1,
					 read ? DMA_DEV_TO_MEM : DMA_MEM_TO_DEV,
					 DMA_PREP_INTERRUPT | DMA_CTRL_ACK);
	if (!txdesc)
		return -EIO;

	txdesc->callback	= rcar_i2c_r4_dma_callback;
	txdesc->callback_param	= priv;

	cookie = dmaengine_submit(txdesc);
	if (dma_submit_error(cookie))
		return -EIO;

	/* start R4 DMA */
	ret = r4_i2c_xfer_dma_start(priv->r4);
	if (ret < 0)
		return r4_err_to_rcar_err(ret);

	dma_async_issue_pending(chan);

	priv->dma_direction = dir;

	return 0;
}

static int rcar_i2c_r4_xfer_pio(struct rcar_i2c_priv *priv)
{
	int ret;

	/* PIO */
	ret = r4_i2c_xfer_pio(priv->r4,
			      priv->msg_idx, priv->msg_max,
			      i2c_8bit_addr_from_msg(priv->msg),
			      priv->msg->buf,
			      priv->msg->len);

	return r4_err_to_rcar_err(ret);
}

static int rcar_i2c_r4_xfer(struct rcar_i2c_priv *priv)
{
	int ret;

	if (RCAR_MIN_DMA_LEN > priv->msg->len || !(priv->flags & ID_P_HAS_R4_DMA))
		ret = rcar_i2c_r4_xfer_pio(priv);
	else
		ret = rcar_i2c_r4_xfer_dma(priv);

	return ret;
}

static int rcar_i2c_r4_irq(struct rcar_i2c_priv *priv)
{
	int ret;

	if (RCAR_MIN_DMA_LEN > priv->msg->len || !(priv->flags & ID_P_HAS_R4_DMA))
		ret = r4_i2c_xfer_pio_irq(priv->r4);
	else
		ret = r4_i2c_xfer_dma_irq(priv->r4);

	return r4_err_to_rcar_err(ret);
}

static irqreturn_t rcar_i2c_r4_irq_handler(int irq, void *ptr)
{
	struct rcar_i2c_priv *priv = ptr;
	int ret;

	ret = rcar_i2c_r4_irq(priv);
	/*
	 * ret > 0 : done
	 * ret = 0 : continue
	 * ret < 0 : error
	 */
	if (!(ret > 0))
		goto out;

	priv->msg_idx++;
	priv->msg++;

	if (priv->msg_idx == priv->msg_max) {
		/* all message has xfered */
		ret = priv->msg_max;
		goto out;
	}

	/* transfer remaining message */
	ret = rcar_i2c_r4_xfer(priv);
out:
	priv->irq_ret = ret;
	if (priv->irq_ret)
		wake_up(&priv->wait);

	return IRQ_HANDLED;
}

static void rcar_i2c_release_dma(struct rcar_i2c_priv *priv)
{
	if (!IS_ERR(priv->dma_tx)) {
		dma_release_channel(priv->dma_tx);
		priv->dma_tx = ERR_PTR(-EPROBE_DEFER);
	}

	if (!IS_ERR(priv->dma_rx)) {
		dma_release_channel(priv->dma_rx);
		priv->dma_rx = ERR_PTR(-EPROBE_DEFER);
	}
}

/* I2C is a special case, we need to poll the status of a reset */
static int rcar_i2c_do_reset(struct rcar_i2c_priv *priv)
{
	int ret;

	/* do nothing */
	if (IS_ERR(priv->rstc))
		return 0;

	ret = reset_control_reset(priv->rstc);
	if (ret)
		return ret;

	return read_poll_timeout_atomic(reset_control_status, ret, ret == 0, 1,
					100, false, priv->rstc);
}

static int rcar_i2c_master_r4_xfer(struct i2c_adapter *adap,
				   struct i2c_msg *msgs,
				   int num)
{
	struct rcar_i2c_priv *priv = i2c_get_adapdata(adap);
	struct device *dev = rcar_i2c_priv_to_dev(priv);
	long time_left;
	int ret;

	pm_runtime_get_sync(dev);
	rcar_i2c_do_reset(priv);

	priv->msg	= msgs;
	priv->msg_idx	= 0;
	priv->msg_max	= num;
	priv->irq_ret	= 0;

	ret = rcar_i2c_r4_xfer(priv);
	if (ret < 0)
		goto out;

	time_left = wait_event_timeout(priv->wait,
				       priv->irq_ret,
				       num * adap->timeout);
	if (!time_left) {
		if (priv->flags & ID_P_HAS_R4_DMA)
			rcar_i2c_r4_dma_clean(priv, true);
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = priv->irq_ret;
out:
	pm_runtime_put(dev);

	if (ret < 0 && ret != -ENXIO)
		dev_err(dev, "error %d : %x\n", ret, priv->flags);

	return ret;
}

static int rcar_i2c_master_r4_atomic(struct i2c_adapter *adap,
				     struct i2c_msg *msgs, int num)
{
	struct rcar_i2c_priv *priv = i2c_get_adapdata(adap);
	struct device *dev = rcar_i2c_priv_to_dev(priv);
	int i;
	int ret;

	pm_runtime_get_sync(dev);

	for (i = 0; i < num; i++) {
		ret = r4_i2c_xfer_atomic(priv->r4,
					 i, num,
					 i2c_8bit_addr_from_msg(msgs),
					 msgs->buf, msgs->len);
		if (ret < 0)
			goto out;

		msgs++;
	}
	ret = num; /* success */
out:
	pm_runtime_put(dev);

	if (ret < 0) {
		ret = r4_err_to_rcar_err(ret);
		dev_err(dev, "error %d\n", ret);
	}

	return ret;
}

static u32 rcar_i2c_func(struct i2c_adapter *adap)
{
	struct rcar_i2c_priv *priv = i2c_get_adapdata(adap);

	/*
	 * This HW can't do:
	 * I2C_SMBUS_QUICK (setting FSB during START didn't work)
	 * I2C_M_NOSTART (automatically sends address after START)
	 * I2C_M_IGNORE_NAK (automatically sends STOP after NAK)
	 */
	u32 func = I2C_FUNC_I2C |
		   (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);

	if (priv->flags & ID_P_HOST_NOTIFY)
		func |= I2C_FUNC_SMBUS_HOST_NOTIFY;

	return func;
}

static struct i2c_algorithm rcar_i2c_algo = {
	.functionality	= rcar_i2c_func,
};

static const struct i2c_adapter_quirks rcar_i2c_quirks = {
	.flags = I2C_AQ_NO_ZERO_LEN,
};

static const struct of_device_id rcar_i2c_dt_ids[] = {
	{ .compatible = "renesas,i2c-r8a7795" },
	{ .compatible = "renesas,i2c-r8a7796" },
	{ .compatible = "renesas,rcar-gen3-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, rcar_i2c_dt_ids);

static int rcar_i2c_probe(struct platform_device *pdev)
{
	struct rcar_i2c_priv *priv;
	struct r4_i2c_priv *r4_target;
	struct i2c_adapter *adap;
	struct device *dev = &pdev->dev;
	unsigned long irqflags = 0;
	irqreturn_t (*irqhandler)(int irq, void *ptr);
	int ret;

	/* Otherwise logic will break because some bytes must always use PIO */
	BUILD_BUG_ON_MSG(RCAR_MIN_DMA_LEN < 3, "Invalid min DMA length");

	priv = devm_kzalloc(dev, sizeof(struct rcar_i2c_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->io = devm_platform_get_and_ioremap_resource(pdev, 0, &priv->res);
	if (IS_ERR(priv->io))
		return PTR_ERR(priv->io);

	init_waitqueue_head(&priv->wait);

	adap = &priv->adap;
	adap->nr = pdev->id;
	adap->algo = &rcar_i2c_algo;
	adap->class = I2C_CLASS_DEPRECATED;
	adap->retries = 3;
	adap->dev.parent = dev;
	adap->dev.of_node = dev->of_node;
	adap->quirks = &rcar_i2c_quirks;
	i2c_set_adapdata(adap, priv);
	strlcpy(adap->name, pdev->name, sizeof(adap->name));

	/* Init DMA */
	sg_init_table(&priv->sg, 1);
	priv->dma_direction = DMA_NONE;
	priv->dma_rx = priv->dma_tx = ERR_PTR(-EPROBE_DEFER);

	/* Target Version Check for R4 */
	r4_target = &r4_i2c_rcar_gen3;
	if (R4_I2C_VERSION_MATCH(r4_target, R4_I2C_VERSION_TARGET(1, 0))) {
		struct r4_i2c_priv *r4;
		unsigned int support;

		r4 = devm_kzalloc(dev, r4_target->alloc_size, GFP_KERNEL);
		if (!r4)
			return -ENOMEM;

		priv->r4 = r4;
		memcpy(r4, r4_target, sizeof(struct r4_i2c_priv));
		r4_i2c_probe(r4, priv);
		r4_i2c_setup_speed(r4, 100 * 1000); /* 100 KHz */

		support = r4_i2c_support(r4);
		if (support & R4_I2C_SUPPORT_RECOVERY)
			adap->bus_recovery_info = &rcar_i2c_rcvry;

		if (support & R4_I2C_SUPPORT_ATOMIC)
			rcar_i2c_algo.master_xfer_atomic = rcar_i2c_master_r4_atomic;

		if (support & R4_I2C_SUPPORT_PIO) {
			irqhandler = rcar_i2c_r4_irq_handler;
			rcar_i2c_algo.master_xfer = rcar_i2c_master_r4_xfer;

			if (support & R4_I2C_SUPPORT_DMA) {
				rU32 reg_dst;
				rU32 reg_src;
				int xfer_size;

				priv->flags |= ID_P_HAS_R4_DMA;

				r4_i2c_xfer_dma_info(priv->r4, &reg_dst, &reg_src, &xfer_size);
				ret = rcar_i2c_request_r4_dma(priv, 0, reg_dst, xfer_size);
				if (ret < 0)
					return ret;
				ret = rcar_i2c_request_r4_dma(priv, 1, reg_src, xfer_size);
				if (ret < 0)
					return ret;
			}
		}

	}

	/* Activate device for clock calculation */
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	priv->rstc = devm_reset_control_get_exclusive(&pdev->dev, NULL);
	if (!IS_ERR(priv->rstc)) {
		ret = reset_control_status(priv->rstc);
		if (ret < 0)
			priv->rstc = ERR_PTR(-ENOTSUPP);
	}

	/* Stay always active when multi-master to keep arbitration working */
	if (of_property_read_bool(dev->of_node, "multi-master"))
		priv->flags |= ID_P_PM_BLOCKED;
	else
		pm_runtime_put(dev);

	if (of_property_read_bool(dev->of_node, "smbus"))
		priv->flags |= ID_P_HOST_NOTIFY;

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto out_pm_disable;
	priv->irq = ret;
	ret = devm_request_irq(dev, priv->irq, irqhandler, irqflags, dev_name(dev), priv);
	if (ret < 0) {
		dev_err(dev, "cannot get irq %d\n", priv->irq);
		goto out_pm_disable;
	}

	platform_set_drvdata(pdev, priv);

	ret = i2c_add_numbered_adapter(adap);
	if (ret < 0)
		goto out_pm_disable;

	if (priv->flags & ID_P_HOST_NOTIFY) {
		priv->host_notify_client = i2c_new_slave_host_notify_device(adap);
		if (IS_ERR(priv->host_notify_client)) {
			ret = PTR_ERR(priv->host_notify_client);
			goto out_del_device;
		}
	}

	dev_info(dev, "probed\n");

	return 0;

 out_del_device:
	i2c_del_adapter(&priv->adap);
	if (priv->flags & ID_P_PM_BLOCKED)
		pm_runtime_put(dev);
 out_pm_disable:
	pm_runtime_disable(dev);
	return ret;
}

static int rcar_i2c_remove(struct platform_device *pdev)
{
	struct rcar_i2c_priv *priv = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	if (priv->r4)
		r4_i2c_remove(priv->r4);

	if (priv->host_notify_client)
		i2c_free_slave_host_notify_device(priv->host_notify_client);
	i2c_del_adapter(&priv->adap);
	rcar_i2c_release_dma(priv);
	if (priv->flags & ID_P_PM_BLOCKED)
		pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int rcar_i2c_suspend(struct device *dev)
{
	struct rcar_i2c_priv *priv = dev_get_drvdata(dev);

	i2c_mark_adapter_suspended(&priv->adap);
	return 0;
}

static int rcar_i2c_resume(struct device *dev)
{
	struct rcar_i2c_priv *priv = dev_get_drvdata(dev);

	i2c_mark_adapter_resumed(&priv->adap);
	return 0;
}

static const struct dev_pm_ops rcar_i2c_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(rcar_i2c_suspend, rcar_i2c_resume)
};

#define DEV_PM_OPS (&rcar_i2c_pm_ops)
#else
#define DEV_PM_OPS NULL
#endif /* CONFIG_PM_SLEEP */

static struct platform_driver rcar_i2c_driver = {
	.driver	= {
		.name	= "i2c-rcar",
		.of_match_table = rcar_i2c_dt_ids,
		.pm	= DEV_PM_OPS,
	},
	.probe		= rcar_i2c_probe,
	.remove		= rcar_i2c_remove,
};

module_platform_driver(rcar_i2c_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Renesas R-Car I2C bus driver");
MODULE_AUTHOR("Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>");
