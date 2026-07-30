/* SPDX-License-Identifier: GPL-2.0
 *
 * soc-def.h
 *
 * Copyright (C) 2026 Renesas Electronics Corp.
 * Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 */
#ifndef __SOC_DEF_H
#define __SOC_DEF_H

/*
 * Convenience kcontrol builders
 */
#define SOC_DOUBLE_S_VALUE(xreg, shift_left, shift_right, xmin, xmax, xsign_bit, \
			   xinvert, xautodisable)			\
	((unsigned long)&(struct soc_mixer_control)			\
	 {.reg = xreg, .rreg = xreg, .shift = shift_left,		\
	  .rshift = shift_right, .min = xmin, .max = xmax,		\
	  .sign_bit = xsign_bit, .invert = xinvert, .autodisable = xautodisable})
#define SOC_DOUBLE_VALUE(xreg, shift_left, shift_right, xmin, xmax, xinvert, xautodisable) \
	SOC_DOUBLE_S_VALUE(xreg, shift_left, shift_right, xmin, xmax, 0, xinvert, \
			   xautodisable)
#define SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, xinvert, xautodisable) \
	SOC_DOUBLE_VALUE(xreg, xshift, xshift, xmin, xmax, xinvert, xautodisable)
#define SOC_DOUBLE_R_S_VALUE(xlreg, xrreg, xshift, xmin, xmax, xsign_bit, xinvert) \
	((unsigned long)&(struct soc_mixer_control)			\
	 {.reg = xlreg, .rreg = xrreg, .shift = xshift, .rshift = xshift, \
	  .max = xmax, .min = xmin, .sign_bit = xsign_bit,		\
	  .invert = xinvert})
#define SOC_DOUBLE_R_VALUE(xlreg, xrreg, xshift, xmin, xmax, xinvert)	\
	SOC_DOUBLE_R_S_VALUE(xlreg, xrreg, xshift, xmin, xmax, 0, xinvert)

#define SOC_SINGLE(xname, reg, shift, max, invert)			\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,	\
		.put = snd_soc_put_volsw,				\
		.private_value = SOC_SINGLE_VALUE(reg, shift, 0, max, invert, 0) }
#define SOC_SINGLE_RANGE(xname, xreg, xshift, xmin, xmax, xinvert)	\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,	\
		.put = snd_soc_put_volsw,				\
		.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, xinvert, 0) }
#define SOC_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array)	\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,	\
		.put = snd_soc_put_volsw,				\
		.private_value = SOC_SINGLE_VALUE(reg, shift, 0, max, invert, 0) }
#define SOC_SINGLE_SX_TLV(xname, xreg, xshift, xmin, xmax, tlv_array)	\
	{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p  = (tlv_array),					\
		.info = snd_soc_info_volsw_sx,				\
		.get = snd_soc_get_volsw_sx,				\
		.put = snd_soc_put_volsw_sx,				\
		.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, 0, 0) }
#define SOC_SINGLE_RANGE_TLV(xname, xreg, xshift, xmin, xmax, xinvert, tlv_array) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = snd_soc_get_volsw, .put = snd_soc_put_volsw,	\
		.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, xinvert, 0) }
#define SOC_DOUBLE(xname, reg, shift_left, shift_right, max, invert)	\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,	\
		.put = snd_soc_put_volsw,				\
		.private_value = SOC_DOUBLE_VALUE(reg, shift_left, shift_right, \
						  0, max, invert, 0) }
#define SOC_DOUBLE_STS(xname, reg, shift_left, shift_right, max, invert) \
	{								\
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,	\
		.access = SNDRV_CTL_ELEM_ACCESS_READ |			\
		SNDRV_CTL_ELEM_ACCESS_VOLATILE,				\
		.private_value = SOC_DOUBLE_VALUE(reg, shift_left, shift_right,	\
						  0, max, invert, 0) }
#define SOC_DOUBLE_R(xname, reg_left, reg_right, xshift, xmax, xinvert) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.info = snd_soc_info_volsw,				\
		.get = snd_soc_get_volsw, .put = snd_soc_put_volsw,	\
		.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
						    0, xmax, xinvert) }
#define SOC_DOUBLE_R_RANGE(xname, reg_left, reg_right, xshift, xmin,	\
			   xmax, xinvert)				\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.info = snd_soc_info_volsw,				\
		.get = snd_soc_get_volsw, .put = snd_soc_put_volsw,	\
		.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, \
						    xshift, xmin, xmax, xinvert) }
#define SOC_DOUBLE_TLV(xname, reg, shift_left, shift_right, max, invert, tlv_array) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,	\
		.put = snd_soc_put_volsw,				\
		.private_value = SOC_DOUBLE_VALUE(reg, shift_left, shift_right, \
						  0, max, invert, 0) }
#define SOC_DOUBLE_SX_TLV(xname, xreg, shift_left, shift_right, xmin, xmax, tlv_array) \
	{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p  = (tlv_array),					\
		.info = snd_soc_info_volsw_sx,				\
		.get = snd_soc_get_volsw_sx,				\
		.put = snd_soc_put_volsw_sx,				\
		.private_value = SOC_DOUBLE_VALUE(xreg, shift_left, shift_right, \
						  xmin, xmax, 0, 0) }
#define SOC_DOUBLE_RANGE_TLV(xname, xreg, xshift_left, xshift_right, xmin, xmax, \
			     xinvert, tlv_array)			\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = snd_soc_get_volsw, .put = snd_soc_put_volsw,	\
		.private_value = SOC_DOUBLE_VALUE(xreg, xshift_left, xshift_right, \
						  xmin, xmax, xinvert, 0) }
#define SOC_DOUBLE_R_TLV(xname, reg_left, reg_right, xshift, xmax, xinvert, tlv_array) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = snd_soc_get_volsw, .put = snd_soc_put_volsw,	\
		.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
						    0, xmax, xinvert) }
#define SOC_DOUBLE_R_RANGE_TLV(xname, reg_left, reg_right, xshift, xmin, \
			       xmax, xinvert, tlv_array)		\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = snd_soc_get_volsw, .put = snd_soc_put_volsw,	\
		.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, \
						    xshift, xmin, xmax, xinvert) }
#define SOC_DOUBLE_R_SX_TLV(xname, xreg, xrreg, xshift, xmin, xmax, tlv_array) \
	{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p  = (tlv_array),					\
		.info = snd_soc_info_volsw_sx,				\
		.get = snd_soc_get_volsw_sx,				\
		.put = snd_soc_put_volsw_sx,				\
		.private_value = SOC_DOUBLE_R_VALUE(xreg, xrreg, xshift, xmin, xmax, 0) }
#define SOC_DOUBLE_R_S_TLV(xname, reg_left, reg_right, xshift, xmin, xmax, xsign_bit, xinvert, tlv_array) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = snd_soc_get_volsw, .put = snd_soc_put_volsw,	\
		.private_value = SOC_DOUBLE_R_S_VALUE(reg_left, reg_right, xshift, \
						      xmin, xmax, xsign_bit, xinvert) }
#define SOC_SINGLE_S_TLV(xname, xreg, xshift, xmin, xmax, xsign_bit, xinvert, tlv_array) \
	SOC_DOUBLE_R_S_TLV(xname, xreg, xreg, xshift, xmin, xmax, xsign_bit, xinvert, tlv_array)
#define SOC_SINGLE_S8_TLV(xname, xreg, xmin, xmax, tlv_array)		\
	{	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p  = (tlv_array),					\
		.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,	\
		.put = snd_soc_put_volsw,				\
		.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.reg = xreg, .rreg = xreg,				\
		 .min = xmin, .max = xmax,				\
		 .sign_bit = 7,} }
#define SOC_DOUBLE_S8_TLV(xname, xreg, xmin, xmax, tlv_array)		\
	{	.iface  = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p  = (tlv_array),					\
		.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,	\
		.put = snd_soc_put_volsw,				\
		.private_value = SOC_DOUBLE_S_VALUE(xreg, 0, 8, xmin, xmax, 7, 0, 0) }
#define SOC_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xitems, xtexts)	\
	{	.reg = xreg, .shift_l = xshift_l, .shift_r = xshift_r,	\
		.items = xitems, .texts = xtexts,			\
		.mask = xitems ? roundup_pow_of_two(xitems) - 1 : 0}
#define SOC_ENUM_SINGLE(xreg, xshift, xitems, xtexts)		\
	SOC_ENUM_DOUBLE(xreg, xshift, xshift, xitems, xtexts)
#define SOC_ENUM_SINGLE_EXT(xitems, xtexts)		\
	{	.items = xitems, .texts = xtexts }
#define SOC_VALUE_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmask, xitems, xtexts, xvalues) \
	{	.reg = xreg, .shift_l = xshift_l, .shift_r = xshift_r,	\
		.mask = xmask, .items = xitems, .texts = xtexts, .values = xvalues}
#define SOC_VALUE_ENUM_SINGLE(xreg, xshift, xmask, xitems, xtexts, xvalues) \
	SOC_VALUE_ENUM_DOUBLE(xreg, xshift, xshift, xmask, xitems, xtexts, xvalues)
#define SOC_VALUE_ENUM_SINGLE_AUTODISABLE(xreg, xshift, xmask, xitems, xtexts, xvalues) \
	{	.reg = xreg, .shift_l = xshift, .shift_r = xshift,	\
		.mask = xmask, .items = xitems, .texts = xtexts,	\
		.values = xvalues, .autodisable = 1}
#define SOC_ENUM_SINGLE_VIRT(xitems, xtexts)			\
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, xitems, xtexts)
#define SOC_ENUM(xname, xenum)						\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.info = snd_soc_info_enum_double,			\
		.get = snd_soc_get_enum_double, .put = snd_soc_put_enum_double, \
		.private_value = (unsigned long)&xenum }
#define SOC_SINGLE_EXT(xname, xreg, xshift, xmax, xinvert,		\
		       xhandler_get, xhandler_put)			\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.info = snd_soc_info_volsw,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = SOC_SINGLE_VALUE(xreg, xshift, 0, xmax, xinvert, 0) }
#define SOC_DOUBLE_EXT(xname, reg, shift_left, shift_right, max, invert, \
		       xhandler_get, xhandler_put)			\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.info = snd_soc_info_volsw,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value =					\
		SOC_DOUBLE_VALUE(reg, shift_left, shift_right, 0, max, invert, 0) }
#define SOC_DOUBLE_R_EXT(xname, reg_left, reg_right, xshift, xmax, xinvert, \
			 xhandler_get, xhandler_put)			\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.info = snd_soc_info_volsw,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
						    0, xmax, xinvert) }
#define SOC_SINGLE_EXT_TLV(xname, xreg, xshift, xmax, xinvert,		\
			   xhandler_get, xhandler_put, tlv_array)	\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = SOC_SINGLE_VALUE(xreg, xshift, 0, xmax, xinvert, 0) }
#define SOC_SINGLE_RANGE_EXT_TLV(xname, xreg, xshift, xmin, xmax, xinvert, \
				 xhandler_get, xhandler_put, tlv_array) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmin, xmax, xinvert, 0) }
#define SOC_DOUBLE_EXT_TLV(xname, xreg, shift_left, shift_right, xmax, xinvert,	\
			   xhandler_get, xhandler_put, tlv_array)	\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = SOC_DOUBLE_VALUE(xreg, shift_left, shift_right, \
						  0, xmax, xinvert, 0) }
#define SOC_DOUBLE_R_EXT_TLV(xname, reg_left, reg_right, xshift, xmax, xinvert,	\
			     xhandler_get, xhandler_put, tlv_array)	\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = SOC_DOUBLE_R_VALUE(reg_left, reg_right, xshift, \
						    0, xmax, xinvert) }
#define SOC_DOUBLE_R_S_EXT_TLV(xname, reg_left, reg_right, xshift, xmin, xmax, \
			       xsign_bit, xinvert, xhandler_get, xhandler_put, \
			       tlv_array)				\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |		\
		SNDRV_CTL_ELEM_ACCESS_READWRITE,			\
		.tlv.p = (tlv_array),					\
		.info = snd_soc_info_volsw,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = SOC_DOUBLE_R_S_VALUE(reg_left, reg_right, xshift, \
						      xmin, xmax, xsign_bit, xinvert) }
#define SOC_SINGLE_S_EXT_TLV(xname, xreg, xshift, xmin, xmax,		\
			     xsign_bit, xinvert, xhandler_get, xhandler_put, \
			     tlv_array)					\
	SOC_DOUBLE_R_S_EXT_TLV(xname, xreg, xreg, xshift, xmin, xmax,	\
			       xsign_bit, xinvert, xhandler_get, xhandler_put, \
			       tlv_array)
#define SOC_SINGLE_BOOL_EXT(xname, xdata, xhandler_get, xhandler_put)	\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.info = snd_soc_info_bool_ext,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = xdata }
#define SOC_SINGLE_BOOL_EXT_ACC(xname, xdata, xhandler_get, xhandler_put, xaccess) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.access = xaccess,					\
		.info = snd_soc_info_bool_ext,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = xdata }
#define SOC_ENUM_EXT(xname, xenum, xhandler_get, xhandler_put)		\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.info = snd_soc_info_enum_double,			\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = (unsigned long)&xenum }
#define SOC_VALUE_ENUM_EXT(xname, xenum, xhandler_get, xhandler_put)	\
	SOC_ENUM_EXT(xname, xenum, xhandler_get, xhandler_put)

#define SOC_ENUM_EXT_ACC(xname, xenum, xhandler_get, xhandler_put, xaccess) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.access = xaccess,					\
		.info = snd_soc_info_enum_double,			\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = (unsigned long)&xenum }

#define SND_SOC_BYTES(xname, xbase, xregs)				\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.info = snd_soc_bytes_info, .get = snd_soc_bytes_get,	\
		.put = snd_soc_bytes_put, .private_value =		\
		((unsigned long)&(struct soc_bytes)			\
		 {.base = xbase, .num_regs = xregs }) }
#define SND_SOC_BYTES_E(xname, xbase, xregs, xhandler_get, xhandler_put) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.info = snd_soc_bytes_info, .get = xhandler_get,	\
		.put = xhandler_put, .private_value =			\
		((unsigned long)&(struct soc_bytes)			\
		 {.base = xbase, .num_regs = xregs }) }
#define SND_SOC_BYTES_E_ACC(xname, xbase, xregs, xhandler_get, xhandler_put, xaccess) \
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.access = xaccess,					\
		.info = snd_soc_bytes_info, .get = xhandler_get,	\
		.put = xhandler_put, .private_value =			\
		((unsigned long)&(struct soc_bytes)			\
		 {.base = xbase, .num_regs = xregs }) }

#define SND_SOC_BYTES_MASK(xname, xbase, xregs, xmask)			\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.info = snd_soc_bytes_info, .get = snd_soc_bytes_get,	\
		.put = snd_soc_bytes_put, .private_value =		\
		((unsigned long)&(struct soc_bytes)			\
		 {.base = xbase, .num_regs = xregs,			\
		  .mask = xmask }) }

/*
 * SND_SOC_BYTES_EXT is deprecated, please USE SND_SOC_BYTES_TLV instead
 */
#define SND_SOC_BYTES_EXT(xname, xcount, xhandler_get, xhandler_put)	\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.info = snd_soc_bytes_info_ext,				\
		.get = xhandler_get, .put = xhandler_put,		\
		.private_value = (unsigned long)&(struct soc_bytes_ext) \
		{.max = xcount} }
#define SND_SOC_BYTES_TLV(xname, xcount, xhandler_get, xhandler_put)	\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,	\
		.access = SNDRV_CTL_ELEM_ACCESS_TLV_READWRITE |		\
		SNDRV_CTL_ELEM_ACCESS_TLV_CALLBACK,			\
		.tlv.c = (snd_soc_bytes_tlv_callback),			\
		.info = snd_soc_bytes_info_ext,				\
		.private_value = (unsigned long)&(struct soc_bytes_ext) \
		{.max = xcount, .get = xhandler_get, .put = xhandler_put, } }
#define SOC_SINGLE_XR_SX(xname, xregbase, xregcount, xnbits,		\
			 xmin, xmax, xinvert)				\
	{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),	\
		.info = snd_soc_info_xr_sx, .get = snd_soc_get_xr_sx,	\
		.put = snd_soc_put_xr_sx,				\
		.private_value = (unsigned long)&(struct soc_mreg_control) \
		{.regbase = xregbase, .regcount = xregcount, .nbits = xnbits, \
		 .invert = xinvert, .min = xmin, .max = xmax} }

#define SOC_SINGLE_STROBE(xname, xreg, xshift, xinvert)		\
	SOC_SINGLE_EXT(xname, xreg, xshift, 1, xinvert,		\
		       snd_soc_get_strobe, snd_soc_put_strobe)

/*
 * Simplified versions of above macros, declaring a struct and calculating
 * ARRAY_SIZE internally
 */
#define SOC_ENUM_DOUBLE_DECL(name, xreg, xshift_l, xshift_r, xtexts)	\
	const struct soc_enum name = SOC_ENUM_DOUBLE(xreg, xshift_l, xshift_r, \
						     ARRAY_SIZE(xtexts), xtexts)
#define SOC_ENUM_SINGLE_DECL(name, xreg, xshift, xtexts)		\
	SOC_ENUM_DOUBLE_DECL(name, xreg, xshift, xshift, xtexts)
#define SOC_ENUM_SINGLE_EXT_DECL(name, xtexts)				\
	const struct soc_enum name = SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(xtexts), xtexts)
#define SOC_VALUE_ENUM_DOUBLE_DECL(name, xreg, xshift_l, xshift_r, xmask, xtexts, xvalues) \
	const struct soc_enum name = SOC_VALUE_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmask, \
							   ARRAY_SIZE(xtexts), xtexts, xvalues)
#define SOC_VALUE_ENUM_SINGLE_DECL(name, xreg, xshift, xmask, xtexts, xvalues) \
	SOC_VALUE_ENUM_DOUBLE_DECL(name, xreg, xshift, xshift, xmask, xtexts, xvalues)

#define SOC_VALUE_ENUM_SINGLE_AUTODISABLE_DECL(name, xreg, xshift, xmask, xtexts, xvalues) \
	const struct soc_enum name = SOC_VALUE_ENUM_SINGLE_AUTODISABLE(xreg, \
								       xshift, xmask, ARRAY_SIZE(xtexts), xtexts, xvalues)

#define SOC_ENUM_SINGLE_VIRT_DECL(name, xtexts)				\
	const struct soc_enum name = SOC_ENUM_SINGLE_VIRT(ARRAY_SIZE(xtexts), xtexts)

enum snd_soc_trigger_order {
						/* start			stop		     */
	SND_SOC_TRIGGER_ORDER_DEFAULT	= 0,	/* Link->Component->DAI		DAI->Component->Link */
	SND_SOC_TRIGGER_ORDER_LDC,		/* Link->DAI->Component		Component->DAI->Link */

	SND_SOC_TRIGGER_ORDER_MAX,
};

/* SoC PCM stream information */
struct snd_soc_pcm_stream {
	const char *stream_name;
	u64 formats;			/* SNDRV_PCM_FMTBIT_* */
	u32 subformats;			/* for S32_LE format, SNDRV_PCM_SUBFMTBIT_* */
	unsigned int rates;		/* SNDRV_PCM_RATE_* */
	unsigned int rate_min;		/* min rate */
	unsigned int rate_max;		/* max rate */
	unsigned int channels_min;	/* min channels */
	unsigned int channels_max;	/* max channels */
	unsigned int sig_bits;		/* number of bits of content */
};


#endif /* __SOC_DEF_H */
