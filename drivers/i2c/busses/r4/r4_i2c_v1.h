/* SPDX-License-Identifier: MIT
 *
 * i2c_v1.h
 *
 * Copyright (c) 2022 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */
#ifndef __I2C_V1_H
#define __I2C_V1_H

/***********************************************************************
[Original Code]

	https://github.com/morimoto/renesas-r3-project.git

[Confirm the R4 version you are using]

	It will indicate controlled driver, or it was custom.

	ex)
	> ${renesas-r3-project}/script/version-check.sh ${your-path-to-code}/r4_i2c_v1*
	r4_i2c_v1.h                   : custom driver
	r4_i2c_v1_rcar_gen3.c         : v1.0.0-rc1

[Required function at OS]

	rU32
	rU8
	rU32  r4_i2c_read(struct r4_i2c_priv *r4, int reg);
	void  r4_i2c_write(struct r4_i2c_priv *r4, int reg, rU32 val);
	void  r4_i2c_udelay(struct r4_i2c_priv *r4, unsigned int time);

[CONFIGs]

	You need to select necessary features.

	CONFIG_R4_I2C_RECOVERY
	CONFIG_R4_I2C_ATOMIC
	CONFIG_R4_I2C_PIO
	CONFIG_R4_I2C_DMA

[Version check]

	You can confirm R4 driver version by using R4_I2C_VERSION_MATCH().

[Required priv data at OS]

	OS need to have/alloc [struct r4_i2c_priv] which is interface
	of OS, and it also be interface for [R4 driver].
	The size is indicated at [r4_i2c_priv.alloc_size]
	The data relationship is like below.

	+----------------+
	| r4_i2c_priv    |
	|    .private    | OS can use .private
	+----------------+
	| R4 driver data |
	+----------------+

[Prepare to use]

	You need to initialize [r4_i2c_priv] as 0,
	and need to copy [r4_i2c_priv] from each R4 driver.
	You can set OS data by r4_i2c_probe(), and get it via
	r4->private on [Required function at OS].

[Pseudo Code for Preparing]

	// see [Required function at OS]
	//     [Require to use]
	static u32 r4_i2c_read(struct r4_i2c_priv *r4, int reg)
	{
		struct os_private *priv = r4->private;
		...
	}

	// see [Version check]
	if (!R4_I2C_VERSION_MATCH(r4_target, R4_I2C_VERSION_TARGET(1, 0))) {
		// version not match
		return error;
	}

	struct r4_i2c_priv *r4;
	struct os_private *priv = xxx;

	// see [Required priv data at OS]
	if (OS_can_use_alloc()) {
		r4 = alloc(r4_target.alloc_size);
	} else {
		struct r4_i2c_data {
			struct r4_i2c_priv	r4;
			struct r4_i2c_gen3_priv	gen3;
		} r4_data[x];

		r4 = &r4_data[x].r4;
	}

	// see [Prepare to use]
	memset(r4, 0, r4_target->alloc_size)
	memcpy(r4,    r4_target, sizeof(struct r4_i2c_priv));
	r4_i2c_probe(r4, priv);


[HOW TO USE ATOMIC transfer]

	r4_i2c_setup_speed(r4, xxx);

	<POWER ON here>
	for (i = 0; i < num; i++)
		r4_i2c_xfer_atomic(r4, i, num, address,
				   msg[i].buf, msg[i].len);
	<POWER OFF here>

[HOW TO USE PIO transfer]

	r4_i2c_setup_speed(r4, xxx);

	idx = 0;
	msg = xxx;

	<POWER ON here>
	r4_i2c_xfer_pio(r4, idx, num, address,
			msg[idx].buf, msg[idx].len);
	wait_done();
	<POWER OFF here>

	//
	// In I2C IRQ hander
	//
	ret = r4_i2c_xfer_pio_irq(r4);
	if (ret < 0) {
		// error
	} if (ret == 0) {
		// still transfering
		// nothing to do on OS side
	} else {
		// current message transfering done.
		// send remaining message.
		idx++;
		if (idx >= num) {
			// all message has transfered
			wakeup();
			return;
		}
		r4_i2c_xfer_pio(r4, idx, num, address,
				msg[idx].buf, msg[idx].len);
	}

[HOW TO USE DMA transfer]

	i2c_transfer_by_dma() {
		buf = &msg[idx].buf; // use local val. it might be updated
		len = &msg[idx].len; // use local val. it might be updated

		// get DMA transfer related information from R4
		// buf/len will be overwrote
		r4_i2c_xfer_dma_setup(r4, idx, num, address, buf, len);

		<Prepare DMA here via
		 dma_dst/dma_src/dma_size/buf/len>
		<starat DMA here>

		// start I2C trasnfer
		r4_i2c_xfer_dma_start();
	}

	r4_i2c_setup_speed(r4, xxx);

	idx = 0;
	msg = xxx;

	// get DMA transfer related information from R4
	dma_dst;
	dma_src;
	dma_size;
	r4_i2c_xfer_dma_info(r4, &dma_dst, &dma_src, &dma_size);

	<POWER ON here>
	i2c_transfer_by_dma();
	wait_done();
	<POWER OFF here>

	//
	// In [DMA IRQ] hander
	//
	r4_i2c_xfer_dma_stop(r4);

	//
	// In [I2C IRQ] hander
	//
	ret = r4_i2c_xfer_dma_irq(r4);
	if (ret < 0) {
		// error
	} if (ret == 0) {
		// still transfering
	} else {
		// current message transfering done.
		// send remaining message.
		idx++;
		if (idx >= num) {
			// all message has transfered
			wakeup();
			return;
		}
		i2c_transfer_by_dma()
	}
***********************************************************************/

struct r4_i2c_priv {
	unsigned int version;
	unsigned int alloc_size;
	unsigned int support;	/* R4_I2C_SUPPORT_xx */

	void *private;		/* private data for OS */

	/* v1.0.x */
	int  (*probe)(struct r4_i2c_priv *r4);
	int  (*remove)(struct r4_i2c_priv *r4);
	int  (*setup_speed)(struct r4_i2c_priv *r4, int bus_speed);
	int  (*recovery)(struct r4_i2c_priv *r4);
	int  (*xfer_atomic)(struct r4_i2c_priv *r4, int step, int step_max,
			    rU32 address, rU8 *buf, int len);
	int  (*xfer_pio)(struct r4_i2c_priv *r4, int step, int step_max,
			    rU32 address, rU8 *buf, int len);
	int  (*xfer_pio_irq)(struct r4_i2c_priv *r4);
	int  (*xfer_dma_irq)(struct r4_i2c_priv *r4);
	int  (*xfer_dma_setup)(struct r4_i2c_priv *r4, int step, int step_max,
			       rU32 address, rU8 **buf, int *len);
	int  (*xfer_dma_enable)(struct r4_i2c_priv *r4, int enable);
	int  (*xfer_dma_info)(struct r4_i2c_priv *r4,
			      rU32 *reg_dst, rU32 *reg_src,
			      int *xfer_size);
};

#define R4_I2C_VERSION(a, b, c, d)	((a & 0xFF) << 24 | (b & 0xFF) << 16 | (c & 0xFF) << 8 | (d & 0xFF))
#define R4_I2C_VERSION_TARGET(a, b)	((a & 0xFF) << 24 | (b & 0xFF))
#define R4_I2C_VERSION_MAJOR(r4)	(((r4)->version >> 24) & 0xFF)
#define R4_I2C_VERSION_MINOR(r4)	(((r4)->version >> 16) & 0xFF)
#define R4_I2C_VERSION_BUGFIX(r4)	(((r4)->version >>  8) & 0xFF)
#define R4_I2C_VERSION_RC(r4)		(((r4)->version      ) & 0xFF)
#define R4_I2C_VERSION_MATCH(r4, v)	(((r4)->version & 0xFFFF0000) == ((v) & 0xFFFF0000))

#define R4_I2C_SUPPORT_RECOVERY		(1 << 0)	/* auto detected */
#define R4_I2C_SUPPORT_ATOMIC		(1 << 1)	/* auto detected */
#define R4_I2C_SUPPORT_PIO		(1 << 2)	/* auto detected */
#define R4_I2C_SUPPORT_DMA		(1 << 3)	/* auto detected */
inline static rU32 r4_i2c_support(struct r4_i2c_priv *r4) {

	rU32 support = 0;

	/* auto detect */
	if (r4->recovery)
		support |= R4_I2C_SUPPORT_RECOVERY;
	if (r4->xfer_atomic)
		support |= R4_I2C_SUPPORT_ATOMIC;
	if (r4->xfer_pio &&
	    r4->xfer_pio_irq)
		support |= R4_I2C_SUPPORT_PIO;
	if (r4->xfer_dma_irq	&&
	    r4->xfer_dma_setup	&&
	    r4->xfer_dma_enable	&&
	    r4->xfer_dma_info)
		support |= R4_I2C_SUPPORT_DMA;

	/*
	 * each driver is possible to have extra
	 * support info via r4->support
	 */
	return support | r4->support;
}

enum r4_i2c_err {
	R4I2CERR_NOTUSED = 0,

/* v1.0.x */
	R4I2CERR_NOTSUPPORTED,
	R4I2CERR_INVAL,
	R4I2CERR_BUSY,
	R4I2CERR_TIMEOUT,
	R4I2CERR_ARBLOST,
	R4I2CERR_NACK,

	RCSI2CERR_MAX,
};

#define R4_I2C_SIMPLE_FUNC(name, defret)		\
inline static int r4_i2c_##name(struct r4_i2c_priv *r4)	\
{							\
	int ret = defret;				\
							\
	if (r4->name)					\
		ret = r4->name(r4);			\
							\
	return ret;					\
}
R4_I2C_SIMPLE_FUNC(remove,		0)
R4_I2C_SIMPLE_FUNC(recovery,		-R4I2CERR_NOTSUPPORTED)
R4_I2C_SIMPLE_FUNC(xfer_pio_irq,	-R4I2CERR_NOTSUPPORTED)
R4_I2C_SIMPLE_FUNC(xfer_dma_irq,	-R4I2CERR_NOTSUPPORTED)

inline static int r4_i2c_probe(struct r4_i2c_priv *r4, void *private)
{
	int ret = 0;

	/*
	 * *DON'T* setup register in r4->probe,
	 * there is no guarantee the Power was enabled.
	 */
	if (r4->probe)
		ret = r4->probe(r4);

	/*
	 * r4->private which is for OS is set here
	 */
	if (ret == 0)
		r4->private = private;

	return ret;
}

inline static int r4_i2c_setup_speed(struct r4_i2c_priv *r4, int bus_speed)
{
	return r4->setup_speed(r4, bus_speed);
}

#define R4_I2C_XFER_FUNC(name)							\
inline static int r4_i2c_xfer_##name(struct r4_i2c_priv *r4,			\
				     int step, int step_max,			\
				     rU32 address, rU8 *buf, int len)		\
{										\
	if (r4->xfer_##name)							\
		return r4->xfer_##name(r4, step, step_max, address, buf, len);	\
										\
	return -R4I2CERR_NOTSUPPORTED;						\
}
R4_I2C_XFER_FUNC(atomic)
R4_I2C_XFER_FUNC(pio)

/*
 * DMA transfer settings itself is OS side task,
 * but some device might have transfer buffer/len limitation.
 * Get DMA info via buf/len.
 */
inline static int r4_i2c_xfer_dma_setup(struct r4_i2c_priv *r4,
					int step, int step_max,
					rU32 address, rU8 **buf, int *len)
{
	if (r4->xfer_dma_setup)
		return r4->xfer_dma_setup(r4, step, step_max, address, buf, len);

	return -R4I2CERR_NOTSUPPORTED;
}

inline static int r4_i2c_xfer_dma_info(struct r4_i2c_priv *r4,
				       rU32 *reg_dst, rU32 *reg_src,
				       int *xfer_size)
{
	if (r4->xfer_dma_info)
		return r4->xfer_dma_info(r4, reg_dst, reg_src, xfer_size);

	return -R4I2CERR_NOTSUPPORTED;
}

#define r4_i2c_xfer_dma_start(r4)	r4_i2c_xfer_dma_enable(r4, 1)
#define r4_i2c_xfer_dma_stop(r4)	r4_i2c_xfer_dma_enable(r4, 0)
inline static int r4_i2c_xfer_dma_enable(struct r4_i2c_priv *r4, int enable)
{
	if (r4->xfer_dma_enable)
		return r4->xfer_dma_enable(r4, enable);

	return -R4I2CERR_NOTSUPPORTED;
}

#endif /* __I2C_V1_H */
