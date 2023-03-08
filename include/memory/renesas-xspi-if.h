/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Renesas RPC-IF core driver
 *
 * Copyright (C) 2018~2019 Renesas Solutions Corp.
 * Copyright (C) 2019 Macronix International Co., Ltd.
 * Copyright (C) 2019-2020 Cogent Embedded, Inc.
 */

#ifndef __RENESAS_XSPI_IF_H
#define __RENESAS_XSPI_IF_H

#include <linux/pm_runtime.h>
#include <linux/types.h>

#include <memory/renesas-rpc-if.h>

int xspi_sw_init(struct rpcif *rpc, struct device *dev);
int xspi_hw_init(struct rpcif *rpc, bool hyperflash);
void xspi_prepare(struct rpcif *rpc, const struct rpcif_op *op, u64 *offs,
		   size_t *len);
int xspi_manual_xfer(struct rpcif *xspi);
ssize_t xspi_dirmap_read(struct rpcif *xspi, u64 offs, size_t len, void *buf);
ssize_t xspi_dirmap_write(struct rpcif *xspi, u64 offs, size_t len, const void *buf);

static inline void xspi_enable_rpm(struct rpcif *xspi)
{
	pm_runtime_enable(xspi->dev);
}

static inline void xspi_disable_rpm(struct rpcif *xspi)
{
	pm_runtime_disable(xspi->dev);
}

#endif // __RENESAS_RPC_IF_H
