/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_NAND_H__
#define __SPI_NAND_H__

#include <sys/util.h>

#define SPI_NAND_INVALID_PAGE 0xffffffff

#define SPI_NAND_ID_LEN			2

struct spi_nand_config {
	/* JEDEC id from devicetree */
	u8_t id[SPI_NAND_ID_LEN];

	/* Indicates support for BE32K */
	bool has_be32k;

	/* spi mode */
	u8_t spi_mode;

	/* Size from devicetree, in bytes */
	u32_t size;
};

/* Status register bits */
#define SPI_NAND_STATUS_OIP_BIT         BIT(0)  /* Write in progress */
#define SPI_NAND_STATUS_WEL_BIT         BIT(1)  /* Write enable latch */
#define SPI_NAND_STATUS_ERASEF_BIT	BIT(2)
#define SPI_NAND_STATUS_PROGF_BIT	BIT(3)
#define SPI_NAND_STATUS_ECC_BIT	 	(BIT(5) | BIT(4))
#define SPI_NAND_STATUS_ECC_POS	 	4

#define SPI_NAND_CTRL_HSE_BIT		BIT(1)
#define SPI_NAND_CTRL_BBI_BIT		BIT(2)
#define SPI_NAND_CTRL_ECCE_BIT		BIT(4)
#define SPI_NAND_CTRL_IDRE_BIT		BIT(6)
#define SPI_NAND_CTRL_PRTE_BIT		BIT(7)

#define SPI_NAND_LOCK_BL_BIT		(BIT(5) | BIT(4) | BIT(3))
#define SPI_NAND_LOCK_BL_POS		3

/* Flash opcodes */
#define SPI_NAND_CMD_WRFR        0x1F    /* Write feature register */
#define SPI_NAND_CMD_RDFR        0x0F    /* Read feature register */
#define SPI_NAND_CMD_RDPB        0x03    /* Read page buffer data */
#define SPI_NAND_CMD_WREN        0x06    /* Write enable */
#define SPI_NAND_CMD_WRDI        0x04    /* Write disable */
#define SPI_NAND_CMD_LDPBSPI     0x02    /* Load page buffer from spi ("Page load") */
#define SPI_NAND_CMD_LDPBRDSPI	 0x84	 /* Load page buffer random data from spi ("Page load random data") */
#define SPI_NAND_CMD_LDPBFCA	 0x13	 /* Load page buffer from flash cell array */
#define SPI_NAND_CMD_WRPBFCA	 0x10    /* Write page buffer to the flash cell array */
#define SPI_NAND_CMD_SECERASE    0x20    /* Sector erase */
#define SPI_NAND_CMD_BE_32K      0x52    /* Block erase 32KB */
#define SPI_NAND_CMD_BLKERASE    0xD8    /* Block erase */
#define SPI_NAND_CMD_CHIPERASE   0xC7    /* Chip erase */
#define SPI_NAND_CMD_RDID        0x9F    /* Read JEDEC ID */
#define SPI_NAND_CMD_ULBPR       0x98    /* Global Block Protection Unlock */
#define SPI_NAND_CMD_DPD         0xB9    /* Deep Power Down */
#define SPI_NAND_CMD_RDPD        0xAB    /* Release from Deep Power Down */
#define SPI_NAND_PROTECT	 0x2A    /* Protect Execute */
#define SPI_NAND_CMD_RESET	 0xFF	 /* Reset operation */

#define SPI_NAND_ROW_ADDR_SIZE		3
#define SPI_NAND_BLOCK_ADDR_SIZE	2
#define SPI_NAND_COLUMN_ADDR_SIZE 	2
#define SPI_NAND_PAGE_ADDR_SIZE		1
#define SPI_NAND_FEATURE_ADDR_SIZE 	1

#define SPI_NAND_ERASE_TIMEOUT		100000	/* erase time out (us)   */
#define SPI_NAND_PROG_TIMEOUT		100000	/* program time out (us) */
#define SPI_NAND_READ_TIMEOUT		100000	/* read time out (us) */
#define SPI_NAND_RESET_TIMEOUT		100000	/* reset time out */

#define SPI_NAND_MIN_ERASE_TIME 	1000    /* min erase time (us) */
#define SPI_NAND_MIN_PROG_TIME		225     /* min programming time (us) */
#define SPI_NAND_MIN_READ_TIME		35      /* min read time (us) */
#define SPI_NAND_POWER_ON_TIME		1100    /* min power up time (us) */

#define SPI_NAND_OPCODE_LEN		1
#define SPI_NAND_ADDR_SIZE_MAX		3
#define SPI_NAND_WAIT_MAX		2
#define SPI_NAND_HEADER_SIZE_MAX	(SPI_NAND_OPCODE_LEN + SPI_NAND_ADDR_SIZE_MAX + SPI_NAND_WAIT_MAX)

enum SPI_NAND_FEATURE_TABLE_ADDRESS_e {
	SPI_NAND_FT_ADDR_LOCK       = 0xA0,
	SPI_NAND_FT_ADDR_CTRL       = 0xB0,
	SPI_NAND_FT_ADDR_STATUS     = 0xC0,
	SPI_NAND_FT_ADDR_BFD_CTRL   = 0x10,
	SPI_NAND_FT_ADDR_BFD_STATUS = 0x20,
	SPI_NAND_FT_ADDR_MBF        = 0x30,
	SPI_NAND_FT_ADDR_BFCR0      = 0x40,
	SPI_NAND_FT_ADDR_BFCR1      = 0x50,
	SPI_NAND_FT_ADDR_BFCR2      = 0x60,
	SPI_NAND_FT_ADDR_BFCR3      = 0x70
};
/*
struct SPI_NAND_FT_REG_LOCK_t {
	u8_t BRWD 	: 1;
	u8_t  		: 1;
	u8_t BL 	: 3;
	u8_t  		: 3;
};

struct SPI_NAND_FT_REG_CTRL_t {
	u8_t PRT_E 	: 1;
	u8_t IDR_E 	: 1;
	u8_t  		: 1;
	u8_t ECC_E 	: 1;
	u8_t  		: 1;
	u8_t BBI 	: 1;
	u8_t HSE 	: 1;
	u8_t  		: 1;
};

struct SPI_NAND_FT_REG_STATUS_t {
	u8_t  		: 2;
	u8_t ECC 	: 2;
	u8_t PRG_F 	: 1;
	u8_t ERS_F 	: 1;
	u8_t WEL 	: 1;
	u8_t OIP 	: 1;
};

struct SPI_NAND_FT_REG_BFDCTRL_t {
	u8_t FLIPS 	: 4;
	u8_t  		: 4;
};

struct SPI_NAND_FT_REG_BFS_t {
	u8_t SECTOR_7 	: 1;
	u8_t SECTOR_6 	: 1;
	u8_t SECTOR_5 	: 1;
	u8_t SECTOR_4 	: 1;
	u8_t SECTOR_3 	: 1;
	u8_t SECTOR_2 	: 1;
	u8_t SECTOR_1 	: 1;
	u8_t SECTOR_0 	: 1;
};

struct SPI_NAND_FT_REG_BFD_t {
	SPI_NAND_FT_REG_BFDCTRL_t 	CTRL;
	SPI_NAND_FT_REG_BFS_t 	STATUS;
};

struct SPI_NAND_FT_REG_MBF_t {
	u8_t COUNT 	: 4;
	u8_t  		: 1;
	u8_t SECTOR 	: 3;
};

struct SPI_NAND_FT_BFR_t {
	u8_t SECTOR1 	: 4;
	u8_t SECTOR0 	: 4;
	u8_t SECTOR3 	: 4;
	u8_t SECTOR2 	: 4;
	u8_t SECTOR5 	: 4;
	u8_t SECTOR4 	: 4;
	u8_t SECTOR7 	: 4;
	u8_t SECTOR6 	: 4;
};

struct SPI_NAND_FEATURE_TABLE_t {
	SPI_NAND_FT_REG_LOCK_t 	LOCK;
	SPI_NAND_FT_REG_CTRL_t 	CTRL;
	SPI_NAND_FT_REG_STATUS_t 	STATUS;
	SPI_NAND_FT_REG_BFD_t 	BFD;
	SPI_NAND_FT_REG_MBF_t 	MBF;
	SPI_NAND_FT_BFR_t 		BFR;
};

struct SPI_NAND_PARAMETER_PAGE_t {
	u8_t signature[16];                   ///<  "NAND"
	u8_t reserved[16];
	u8_t device_manufacturer[12];        ///<  "TOSHIBA"
	u8_t device_model[20];               ///<  "TC58CVG2S0HRAIG"
	u8_t manufacturerID;                 ///<  98h
	u8_t reserved2[15];
	u32_t page_bytes;                    ///<  00001000h
	u16_t page_spare_bytes;              ///<  0080h
	u32_t partial_page_bytes;            ///<  00000200h
	u16_t partial_page_spare_bytes;      ///<  0010h
	u32_t block_pages;                   ///<  00000040h
	u32_t unit_blocks;                   ///<  00000800h
	u8_t units;                          ///<  01h
	u8_t reserved3;
	u8_t cell_bits;                      ///<  01h
	u16_t unit_bb;                       ///<  0028h
	u16_t endurance;                     ///<  0501h
	u8_t valid_blocks;
	u8_t reserved4[2];
	u8_t page_partial_writes;            ///<  04h
	u8_t reserved5;
	u8_t ecc_bits;
	u8_t reserved6[15];
	u8_t pin_capacitance;
	u8_t reserved7[4];
	u16_t page_program_time;             ///<  0258h
	u16_t block_erase_time;              ///<  1b58h
	u16_t page_read_time;                ///<  0118h
	u8_t reserved8[115];
	u16_t crc;                           ///<  e1f5h
};
*/
/* Page, sector, and block size are standard, not configurable. */
#define SPI_NAND_PAGE_SIZE    0x1000U
#define SPI_NAND_SECTOR_SIZE  0x200U
#define SPI_NAND_BLOCK_SIZE   0x40000U
#define SPI_NAND_PARTIAL_PAGE_WRITES 	4
#define SPI_NAND_PARTIAL_PAGE_SIZE (SPI_NAND_PAGE_SIZE / SPI_NAND_PARTIAL_PAGE_WRITES)

/* Some devices support erase operations on 32 KiBy blocks.
 * Support is indicated by the has-be32k property.
 */
#define SPI_NAND_BLOCK32_SIZE 0x8000

/* Test whether offset is aligned. */
#define SPI_NAND_IS_PAGE_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_PAGE_SIZE - 1U)) == 0)
#define SPI_NAND_IS_SECTOR_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_SECTOR_SIZE - 1U)) == 0)
#define SPI_NAND_IS_BLOCK_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_BLOCK_SIZE - 1U)) == 0)
#define SPI_NAND_IS_BLOCK32_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_BLOCK32_SIZE - 1U)) == 0)

static inline u32_t page_addr_of(u32_t addr)
{
    return addr & ~(SPI_NAND_PAGE_SIZE - 1U);
}

static inline u32_t page_offset_of(u32_t addr)
{
    return addr & (SPI_NAND_PAGE_SIZE - 1U);
}

#endif /*__SPI_NAND_H__*/
