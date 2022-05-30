// SPDX-License-Identifier: MIT
//
// i2c_v1.c
//
// Copyright (c) 2022 Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
//
// [Original Code]
//
//	https://github.com/morimoto/renesas-r3-project.git
//
// [Confirm the R4 version you are using]
//
// It will indicate controlled driver, or it was custom.
//
//	ex)
//	> ${renesas-r3-project}/script/version-check.sh ${your-path-to-code}/r4_i2c_v1*
//	r4_i2c_v1.h                   : custom driver
//	r4_i2c_v1_rcar_gen3.c         : v1.0.0-rc1
//

#define VER_MINOR	0
#define VER_BUGFIX	0
#define VER_RC		1

#define ICSCR		0x00
#define ICMCR		0x04
#define ICSSR		0x08
#define ICMSR		0x0C
#define ICSIER		0x10
#define ICMIER		0x14
#define ICCCR		0x18
#define ICSAR		0x1C
#define ICMAR		0x20
#define ICRXTX		0x24
#define ICFBSCR		0x38
#define ICDMAER		0x3c

/* ICSCR */
#define SDBS		(1 << 3)

/* ICMCR */
#define MDBS		(1 << 7)
#define FSCL		(1 << 6)
#define FSDA		(1 << 5)
#define OBPC		(1 << 4)
#define MIE		(1 << 3)
#define TSBE		(1 << 2)
#define FSB		(1 << 1)
#define ESG		(1 << 0)
#define PHASE_START	(MDBS | MIE | ESG)
#define PHASE_DATA	(MDBS | MIE)
#define PHASE_STOP	(MDBS | MIE | FSB)


/* ICMSR / ICMIE */
#define MNR		(1 << 6)
#define MAL		(1 << 5)
#define MST		(1 << 4)
#define MDE		(1 << 3)
#define MDT		(1 << 2)
#define MDR		(1 << 1)
#define MAT		(1 << 0)

#define WAIT_BUSY	(MNR|MAL|MST|MDE|    MDR|MAT)
#define STATE_ACK_SEND	(MNR|MAL|MST|    MDT|MDR)
#define STATE_ACK_RECV	(MNR|MAL|MST|MDE|MDT)

#define ENABLE_SEND	(MNR|MAL|MST|MDE	|MAT)
#define ENABLE_RECV	(MNR|MAL|MST|	     MDR|MAT)

/* ICFBSCR */
#define TCYC17		0x0f

/* ICDMAER */
#define RSDMAE		(1 << 3)
#define TSDMAE		(1 << 2)
#define RMDMAE		(1 << 1)
#define TMDMAE		(1 << 0)

struct r4_i2c_gen3_priv {
	rU32 icccr;

	int step;
	int step_max;
	rU32 address;
	rU8 *buf;
	int len;
	int pos;

	rU32 flags;
};

/* flags */
#define FLAG_READ	(1 << 0)
#define FLAG_READ0	(1 << 1)

#define R4_TO_RPIV(r4) (struct r4_i2c_gen3_priv *)(r4 + 1)

/********************************************************

	Common functions

*********************************************************/
static int r4_gen3_bus_barrier(struct r4_i2c_priv *r4)
{
	int i;

	for (i = 0; i < 200; i++) {
		if (!(r4_i2c_read(r4, ICMCR) & FSDA))
			return 0;
		r4_i2c_udelay(r4, 5);
	}

	return -R4I2CERR_BUSY;
}

static int r4_gen3_is_read(struct r4_i2c_gen3_priv *priv)
{
	return (priv->flags & FLAG_READ);
}

static int r4_gen3_setup_speed(struct r4_i2c_priv *r4, int bus_speed)
{
	struct r4_i2c_gen3_priv *priv = R4_TO_RPIV(r4);
	rU32 cdf, scgd;

	/*
	 * Table 57.3 Recommended Settings for CDF and SCGD
	 *
	 * Recommended register values for open drain buffer
	 * Recommended register values for LVTTL buffer (low drive only)
	 *
	 *	Module Clock Frequency : 133.33 MHz
	 *	Bus Speed 100 kHz : CDF = 6, SCGD = 21
	 *	Bus Speed 400 kHz : CDF = 6, SCGD =  3
	 */
	switch (bus_speed) {
	case 100000:
		cdf  =  6;
		scgd = 21;
		break;
	case 400000:
		cdf  = 6;
		scgd = 3;
		break;
	default:
		return -R4I2CERR_INVAL;
	}

	priv->icccr = (scgd << 3) | cdf;

	return 0;
}

#define r4_gen3_irq_enable(r4)	r4_gen3_irq_ctrl(r4, 1)
#define r4_gen3_irq_disable(r4)	r4_gen3_irq_ctrl(r4, 0)
static void r4_gen3_irq_ctrl(struct r4_i2c_priv *r4, int enable)
{
	struct r4_i2c_gen3_priv *priv = R4_TO_RPIV(r4);
	rU32 val = 0;

	if (enable)
		val = r4_gen3_is_read(priv) ?
			ENABLE_RECV : ENABLE_SEND;
	r4_i2c_write(r4, ICMIER, val);
}

static int r4_gen3_status_update(struct r4_i2c_priv *r4, rU32 msr)
{
	int ret;

	/* Arbitration lost */
	if (msr & MAL) {
		ret = -R4I2CERR_ARBLOST;
		goto clear;
	}

	/* Nack */
	if (msr & MNR) {
		ret = -R4I2CERR_NACK;
		goto clear;
	}

	/* Stop */
	if (msr & MST) {
		ret = 1; /* done */
		goto clear;
	}

	/* Continue */
	return 0;

clear:
	r4_gen3_irq_disable(r4);
	r4_i2c_write(r4, ICMSR, 0);

	return ret;
}

static void r4_gen3_xfer_init(struct r4_i2c_priv *r4,
			      int step, int step_max,
			      rU32 address, rU8 *buf, int len)
{
	struct r4_i2c_gen3_priv *priv = R4_TO_RPIV(r4);

	priv->buf	= buf;
	priv->len	= len;
	priv->step	= step;
	priv->step_max	= step_max;
	priv->address	= address;
	priv->pos	= 0;
	priv->flags	= 0;

	if (address & 0x1)
		priv->flags |= (FLAG_READ | FLAG_READ0);
}

static int r4_gen3_xfer_start(struct r4_i2c_priv *r4)
{
	struct r4_i2c_gen3_priv *priv = R4_TO_RPIV(r4);

	if (priv->step >= priv->step_max)
		return -R4I2CERR_INVAL;

	if (priv->step == 0) {
		int ret;

		/* check bus status not to lost bus busy info */
		ret = r4_gen3_bus_barrier(r4);
		if (ret < 0)
			return ret;

		/*
		 * Reset Slave
		 *
		 * FIXME
		 * not yet supported
		 */
		r4_i2c_write(r4, ICSIER, 0);
		r4_i2c_write(r4, ICSCR,  SDBS);
		r4_i2c_write(r4, ICSAR,  0);
		r4_i2c_write(r4, ICSSR,  0);

		/* Reset Master */
		r4_i2c_write(r4, ICMIER, 0);
		r4_i2c_write(r4, ICMCR,  MDBS);
		r4_i2c_write(r4, ICMAR,  0);
		r4_i2c_write(r4, ICMSR,  0);

		/* setup clock */
		r4_i2c_write(r4, ICCCR,  priv->icccr);

		/* 17*Tcyc delay 1st bit between SDA and SCL */
		r4_i2c_write(r4, ICFBSCR, TCYC17);
	}

	/* setup address */
	r4_i2c_write(r4, ICMAR, priv->address);

	/* start */
	r4_i2c_write(r4, ICMSR, 0);
	r4_i2c_write(r4, ICMCR, PHASE_START);

	return 0;
}

static void r4_gen3_xfer_send(struct r4_i2c_priv *r4)
{
	struct r4_i2c_gen3_priv *priv = R4_TO_RPIV(r4);

	/*
	 * (address sent by r4_gen3_xfer_start())
	 * IRQ	[DATA]	[SEND]	[ACK]
	 * IRQ		[SEND]	[ACK]
	 * IRQ		[SEND]	[ACK]
	 *	...
	 * IRQ	[STOP]		[CLEAR]
	 */
	if (priv->pos == priv->len) {
		r4_i2c_write(r4, ICMCR, PHASE_STOP);			/* [STOP]  */
		r4_i2c_write(r4, ICMSR, 0);				/* [CLEAR] */
		return;
	}
	if (priv->pos == 0)
		r4_i2c_write(r4, ICMCR, PHASE_DATA);			/* [DATA] */

	r4_i2c_write(r4, ICRXTX, priv->buf[priv->pos++]);		/* [SEND] */

	r4_i2c_write(r4, ICMSR, STATE_ACK_SEND);			/* [ACK] */
}

static void r4_gen3_xfer_recv(struct r4_i2c_priv *r4)
{
	struct r4_i2c_gen3_priv *priv = R4_TO_RPIV(r4);

	/*
	 * len == 1
	 *
	 * (address sent by r4_gen3_xfer_start())
	 * IRQ	[STOP]		[ACK]	(A) priv->pos check is not enough
	 * IRQ		[READ]	[CLEAR]
	 *
	 * len > 1
	 *
	 * (address sent by r4_gen3_xfer_start())
	 * IRQ	[DATA]		[ACK]
	 * IRQ		[READ]	[ACK]
	 * IRQ		[READ]	[ACK]
	 *	...
	 * IRQ	[STOP]	[READ]	[ACK]	(B) priv->pos check is enough
	 * IRQ		[READ]	[CLEAR]
	 */
	if (((priv->len == 1) && r4_gen3_is_read(priv)) ||		/* (A) */
	    ((priv->len  > 1) && (priv->pos + 1 == priv->len)))		/* (B) */
		r4_i2c_write(r4, ICMCR, PHASE_STOP);				/* [STOP] */
	else if (priv->flags & FLAG_READ0)
		r4_i2c_write(r4, ICMCR, PHASE_DATA);				/* [DATA] */

	if (priv->flags & FLAG_READ0)
		priv->flags &= ~FLAG_READ0;
	else if ((priv->pos < priv->len))
		priv->buf[priv->pos++] = r4_i2c_read(r4, ICRXTX) & 0xFF;	/* [READ] */

	if (priv->pos < priv->len)
		r4_i2c_write(r4, ICMSR, STATE_ACK_RECV);			/* [ACK] */
	else
		r4_i2c_write(r4, ICMSR, 0);					/* [CLEAR] */
}

static int r4_gen3_xfer_continue(struct r4_i2c_priv *r4, rU32 msr)
{
	struct r4_i2c_gen3_priv *priv = R4_TO_RPIV(r4);
	int ret;

	/*
	 * ret > 0 : done
	 * ret = 0 : continue
	 * ret < 0 : error
	 */
	ret = r4_gen3_status_update(r4, msr);
	if (ret)
		return ret;

	if (r4_gen3_is_read(priv))
		r4_gen3_xfer_recv(r4);
	else
		r4_gen3_xfer_send(r4);

	/* Continue */
	return 0;
}

/********************************************************

	RECOVERY

*********************************************************/
#ifdef CONFIG_R4_I2C_RECOVERY
static int r4_gen3_recovery(struct r4_i2c_priv *r4)
{
	rU32 def = MDBS | OBPC;
	int i;

	/* send 9 SCL */
	for (i = 0; i < 9; i++) {
		r4_i2c_write(r4, ICMCR, def | FSDA | FSCL);	r4_i2c_udelay(r4, 5);
		r4_i2c_write(r4, ICMCR, def | FSDA);		r4_i2c_udelay(r4,5);
	}

	/* send stop condition */
	r4_i2c_udelay(r4, 5);
	r4_i2c_write(r4, ICMCR, def | FSDA);		r4_i2c_udelay(r4, 5);
	r4_i2c_write(r4, ICMCR, def);			r4_i2c_udelay(r4, 5);
	r4_i2c_write(r4, ICMCR, def |		FSCL);	r4_i2c_udelay(r4, 5);
	r4_i2c_write(r4, ICMCR, def | FSDA |	FSCL);	r4_i2c_udelay(r4, 5);

	/*
	 * FIXME
	 *
	 * Need to check recovery fail
	 */

	return 0;
}
#define R4_I2C_RECOVERY\
	.recovery		= r4_gen3_recovery,
#else
#define R4_I2C_RECOVERY
#endif

/********************************************************

	ATOMIC

*********************************************************/
#ifdef CONFIG_R4_I2C_ATOMIC
static rU32 r4_gen3_xfer_atomic_busy_wait_irq(struct r4_i2c_priv *r4)
{
	rU32 condition = WAIT_BUSY;
	rU32 msr;
	int timeout = 256;
	int i;

	/* wait ready */
	for (i = 0; i < timeout; i++) {
		msr = r4_i2c_read(r4, ICMSR);
		if (msr & condition)
			return msr;
		r4_i2c_udelay(r4, 5);
	}

	return 0;
}

static int r4_gen3_xfer_atomic(struct r4_i2c_priv *r4,
			       int step, int step_max,
			       rU32 address, rU8 *buf, int len)
{
	rU32 msr;
	int ret;

	r4_gen3_xfer_init(r4, step, step_max, address, buf, len);

	ret = r4_gen3_xfer_start(r4);
	if (ret < 0)
		return ret;

	while(1) {
		msr = r4_gen3_xfer_atomic_busy_wait_irq(r4);
		if (!msr)
			return -R4I2CERR_TIMEOUT;

		ret = r4_gen3_xfer_continue(r4, msr);
		/*
		 * ret > 0 : done
		 * ret = 0 : continue
		 * ret < 0 : error
		 */
		if (ret < 0)
			return ret;
		if (ret)
			break;
	}

	/* return 0 means "success" as atomic function */
	return 0;
}
#define R4_I2C_ATOMIC\
	.xfer_atomic		= r4_gen3_xfer_atomic,
#else
#define R4_I2C_ATOMIC
#endif

/********************************************************

	PIO

*********************************************************/
#ifdef CONFIG_R4_I2C_PIO
static int r4_gen3_xfer_pio_irq(struct r4_i2c_priv *r4)
{
	rU32 msr = r4_i2c_read(r4, ICMSR);

	return r4_gen3_xfer_continue(r4, msr);
}

static int r4_gen3_xfer_pio(struct r4_i2c_priv *r4,
			    int step, int step_max,
			    rU32 address, rU8 *buf, int len)
{
	int ret;

	r4_gen3_xfer_init(r4, step, step_max, address, buf, len);

	ret = r4_gen3_xfer_start(r4);
	if (ret < 0)
		return ret;

	r4_gen3_irq_enable(r4);

	return 0;
}
#define R4_I2C_PIO\
	.xfer_pio		= r4_gen3_xfer_pio,\
	.xfer_pio_irq		= r4_gen3_xfer_pio_irq,
#else
#define R4_I2C_PIO
#endif

/********************************************************

	DMA

*********************************************************/
#ifdef CONFIG_R4_I2C_DMA
#define r4_gen3_dma_enable(r4)	r4_gen3_dma_ctrl(r4, 1)
#define r4_gen3_dma_disable(r4)	r4_gen3_dma_ctrl(r4, 0)
static void r4_gen3_dma_ctrl(struct r4_i2c_priv *r4, int enable)
{
	struct r4_i2c_gen3_priv *priv = R4_TO_RPIV(r4);
	rU32 val = 0;

	if (enable)
		val = r4_gen3_is_read(priv) ? RMDMAE : TMDMAE;

	r4_i2c_write(r4, ICDMAER, val);
}

static int r4_gen3_xfer_dma_info(struct r4_i2c_priv *r4,
				 rU32 *reg_dst, rU32 *reg_src, int *xfer_size)
{
	*reg_dst	= ICRXTX;
	*reg_src	= ICRXTX;
	*xfer_size	= 1;

	return 0;
}

static int r4_gen3_xfer_dma_irq(struct r4_i2c_priv *r4)
{
	struct r4_i2c_gen3_priv *priv = R4_TO_RPIV(r4);
	rU32 msr = r4_i2c_read(r4, ICMSR);
	int ret;

	/*
	 * ret > 0 : done
	 * ret = 0 : continue
	 * ret < 0 : error
	 */
	ret = r4_gen3_xfer_continue(r4, msr);
	if (ret)
		return ret;

	/*
	 * Setup priv->pos for next IRQ (= after DMA)
	 * at first time
	 */
	if ( r4_gen3_is_read(priv) && (priv->pos == 0))	/* (A) */
		priv->pos = priv->len - 2;
	if (!r4_gen3_is_read(priv) && (priv->pos == 1))	/* (B) */
		priv->pos = priv->len;

	return 0;
}

static int r4_gen3_xfer_dma_setup(struct r4_i2c_priv *r4,
				  int step, int step_max,
				  rU32 address, rU8 **buf, int *len)
{
	r4_gen3_xfer_init(r4, step, step_max, address, *buf, *len);

	/*
	 * Indicate buf/len info to OS to prepare DMA
	 */
	if (address & 0x1) {
		/*
		 * READ
		 *
		 * Last two bytes need to use PIO.
		 * Thus, DMA transfer size is (len - 2)
		 * see
		 *	r4_gen3_xfer_dma_irq()
		 *	r4_gen3_xfer_recv()
		 */
		*len = *len - 2;
	} else {
		/*
		 * WRITE
		 *
		 * It need to use PIO for first 1 byte.
		 * Thus, DMA transfer is
		 *	buf = (buf + 1)
		 *	len = (len - 1)
		 * see
		 *	r4_gen3_xfer_dma_setup()
		 *	r4_gen3_xfer_send()
		 */
		*buf = *buf + 1;
		*len = *len - 1;
	}
	return 0;
}

static int r4_gen3_xfer_dma_enable(struct r4_i2c_priv *r4, int enable)
{
	if (enable) {
		int ret;

		ret = r4_gen3_xfer_start(r4);
		if (ret < 0)
			return ret;

		r4_gen3_irq_enable(r4);
		r4_gen3_dma_enable(r4);
	} else {
		r4_gen3_dma_disable(r4);
	}

	return 0;
}
#define R4_I2C_DMA\
	.xfer_dma_irq		= r4_gen3_xfer_dma_irq,\
	.xfer_dma_setup		= r4_gen3_xfer_dma_setup,\
	.xfer_dma_info		= r4_gen3_xfer_dma_info,\
	.xfer_dma_enable	= r4_gen3_xfer_dma_enable,
#else
#define R4_I2C_DMA
#endif

/*
 * see
 *	[PRIV RELATIONSHIP]
 */
struct r4_i2c_priv r4_i2c_rcar_gen3 = {
	.version		= R4_I2C_VERSION(1, VER_MINOR, VER_BUGFIX, VER_RC),
	.alloc_size		= sizeof(struct r4_i2c_priv) + sizeof(struct r4_i2c_gen3_priv),
	.setup_speed		= r4_gen3_setup_speed,
	R4_I2C_RECOVERY
	R4_I2C_ATOMIC
	R4_I2C_PIO
	R4_I2C_DMA
};
