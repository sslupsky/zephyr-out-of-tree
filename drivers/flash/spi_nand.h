/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_NAND_H__
#define __SPI_NAND_H__

#include <sys/util.h>

#define SPI_NAND_MAX_ID_LEN	3

struct spi_nand_config {
	/* JEDEC id from devicetree */
	u8_t id[SPI_NAND_MAX_ID_LEN];

	/* Indicates support for BE32K */
	bool has_be32k;

	/* Size from devicetree, in bytes */
	u32_t size;
};

/* Status register bits */
#define SPI_NAND_WIP_BIT         BIT(0)  /* Write in progress */
#define SPI_NAND_WEL_BIT         BIT(1)  /* Write enable latch */

/* Flash opcodes */
#define SPI_NAND_CMD_WRSR        0x01    /* Write status register */
#define SPI_NAND_CMD_RDSR        0x05    /* Read status register */
#define SPI_NAND_CMD_READ        0x03    /* Read data */
#define SPI_NAND_CMD_WREN        0x06    /* Write enable */
#define SPI_NAND_CMD_WRDI        0x04    /* Write disable */
#define SPI_NAND_CMD_PP          0x02    /* Page program */
#define SPI_NAND_CMD_SE          0x20    /* Sector erase */
#define SPI_NAND_CMD_BE_32K      0x52    /* Block erase 32KB */
#define SPI_NAND_CMD_BE          0xD8    /* Block erase */
#define SPI_NAND_CMD_CE          0xC7    /* Chip erase */
#define SPI_NAND_CMD_RDID        0x9F    /* Read JEDEC ID */
#define SPI_NAND_CMD_ULBPR       0x98    /* Global Block Protection Unlock */
#define SPI_NAND_CMD_DPD         0xB9    /* Deep Power Down */
#define SPI_NAND_CMD_RDPD        0xAB    /* Release from Deep Power Down */

/* Page, sector, and block size are standard, not configurable. */
#define SPI_NAND_PAGE_SIZE    0x0100U
#define SPI_NAND_SECTOR_SIZE  0x1000U
#define SPI_NAND_BLOCK_SIZE   0x10000U

/* Some devices support erase operations on 32 KiBy blocks.
 * Support is indicated by the has-be32k property.
 */
#define SPI_NAND_BLOCK32_SIZE 0x8000

/* Test whether offset is aligned. */
#define SPI_NAND_IS_PAGE_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_PAGE_SIZE - 1U)) == 0)
#define SPI_NAND_IS_SECTOR_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_SECTOR_SIZE - 1U)) == 0)
#define SPI_NAND_IS_BLOCK_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_BLOCK_SIZE - 1U)) == 0)
#define SPI_NAND_IS_BLOCK32_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_BLOCK32_SIZE - 1U)) == 0)

#endif /*__SPI_NAND_H__*/
