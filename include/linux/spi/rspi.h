/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Renesas SPI driver
 *
 * Copyright (C) 2012  Renesas Solutions Corp.
 */

#ifndef __LINUX_SPI_RENESAS_SPI_H__
#define __LINUX_SPI_RENESAS_SPI_H__

enum {
	RSPI_SPI_MASTER,
	RSPI_SPI_SLAVE,
};

struct rspi_plat_data {
	unsigned int dma_tx_id;
	unsigned int dma_rx_id;

	u16 num_chipselect;
};

#endif
