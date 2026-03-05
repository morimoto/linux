/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (C) 2026 Renesas Electronics Corp.
 */

#ifndef __DT_BINDINGS_RESET_RENESAS_R8A78000_SCMI_SCP_V4_32_0_H__
#define __DT_BINDINGS_RESET_RENESAS_R8A78000_SCMI_SCP_V4_32_0_H__

/*
 * The Reset IDs for SCP FW SDKv4.32.0 extend those for v4.31.0.
 */

#include "renesas,r8a78000-scmi-scp-v4.31.0.h"

#undef R8A78000_SCMI_RESET_COUNT

#define R8A78000_SCMI_RESET_U266_PCIE4		815
#define R8A78000_SCMI_RESET_U266_PCIE6		816
#define R8A78000_SCMI_RESET_COUNT		817

#endif /* __DT_BINDINGS_RESET_RENESAS_R8A78000_SCMI_SCP_V4_32_0_H__ */
