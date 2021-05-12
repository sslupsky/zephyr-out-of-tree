/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SPI_NAND_H__
#define __SPI_NAND_H__

#include <sys/util.h>

#define SPI_NAND_INVALID_PAGE 0xffffffff

#define SPI_NAND_JEDEC_ID_LEN		DT_INST_PROP_LEN(0, jedec_id)
#define SPI_NAND_SIZE			DT_INST_PROP(0, size)
#define SPI_NAND_PAGE_SIZE		DT_INST_PROP(0, page_bytes)
#define SPI_NAND_PARTIAL_PAGE_SIZE	DT_INST_PROP(0, partial_page_bytes)
#define SPI_NAND_SECTOR_SIZE		DT_INST_PROP(0, sector_bytes)
#define SPI_NAND_PAGES_PER_BLOCK	DT_INST_PROP(0, pages_per_block)
#define SPI_NAND_BLOCK_SIZE		(SPI_NAND_PAGE_SIZE * SPI_NAND_PAGES_PER_BLOCK)
#define SPI_NAND_BLOCKS			DT_INST_PROP(0, blocks)
#define SPI_NAND_MAX_PAGE_PROG_TIME	DT_INST_PROP(0, max_page_prog_time)
#define SPI_NAND_MAX_PAGE_READ_TIME	DT_INST_PROP(0, max_page_read_time)
#define SPI_NAND_MAX_BLOCK_ERASE_TIME	DT_INST_PROP(0, max_block_erase_time)
#define SPI_NAND_PAGE_PROG		DT_INST_PROP(0, page_prog)
#define SPI_NAND_MAX_RESET_TIME		(10000U)	/* reset time out */

/* Test whether offset is aligned. */
#define SPI_NAND_IS_PAGE_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_PAGE_SIZE - 1U)) == 0)
#define SPI_NAND_IS_SECTOR_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_SECTOR_SIZE - 1U)) == 0)
#define SPI_NAND_IS_BLOCK_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_BLOCK_SIZE - 1U)) == 0)


struct spi_nand_config {
	/* JEDEC id from devicetree */
	u8_t id[SPI_NAND_JEDEC_ID_LEN];

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
	u8_t signature[4];                   ///<  ASCII text, "NAND"
	u8_t onfi_version[2];			///<  ONFI Revision Number
	u8_t features[2];
	u8_t optional_commands[2];
	u8_t reserved[22];
	u8_t device_manufacturer[12];        ///<  ASCII text, ex: "TOSHIBA"
	u8_t device_model[20];               ///<  ASCII text, ex: "TC58CVG2S0HRAIG"
	u8_t manufacturerID;                 ///<  Toshiba = 98h,  Alliance = 52h
	u8_t date_code[2];
	u8_t reserved2[13];
	u32_t page_bytes;                    ///<  00001000h
	u16_t page_spare_bytes;              ///<  0080h
	u32_t partial_page_bytes;            ///<  00000200h
	u16_t partial_page_spare_bytes;      ///<  0010h
	u32_t block_pages;                   ///<  00000040h
	u32_t logical_unit_blocks;           ///<  00000800h
	u8_t logical_units;                  ///<  01h
	u8_t address_cycles;
	u8_t cell_bits;                      ///<  01h
	u16_t logical_unit_bad_blocks;       ///<  0028h
	u16_t block_endurance;               ///<  0501h
	u8_t valid_blocks;
	u8_t valid_block_endurance;
	u8_t page_partial_writes;            ///<  04h
	u8_t partial_write_attributes;
	u8_t ecc_bits;
	u8_t interleaved_address_bits;
	u8_t interleaved_operation_attributes;
	u8_t reserved3[13];
	u8_t pin_capacitance;
	u8_t timing_mode[2];
	u8_t cache_timing_mode[2];
	u16_t page_program_time;             ///<  0258h
	u16_t block_erase_time;              ///<  1b58h
	u16_t page_read_time;                ///<  0118h
	u16_t column_setup_time;
	u8_t reserved4[23];
	u16_t vendor_revision;
	u8_t vendor_data[88];
	u16_t crc;                           ///<  e1f5h
};
*/

/*
 *  SPI NAND Parameter Page Data Address
 *
 *  References:
 *  Alliance Memory SPI NAND Flash July 2020 Rev 1.0 Table 10-3 (page 38)
 *  Toshiba / Kioxia TC58CVG2S0HxAIx Rev 1.1 2016-11-08 Table 19 (page 30)
 *  Kioxia TC58CVG2SOHRAIJ datasheet 20191001 - Table 19 (page 35)
 */
#define SPI_NAND_PARAMETER_SIGNATURE			0
#define SPI_NAND_PARAMETER_ONFI_REVISION		4
#define SPI_NAND_PARAMETER_FEATURES			6
#define SPI_NAND_PARAMETER_OPTIONAL_CMDS		8
#define SPI_NAND_PARAMETER_DEVICE_MFG			32
#define SPI_NAND_PARAMETER_DEVICE_MODEL			44
#define SPI_NAND_PARAMETER_JEDEC_ID			64
#define SPI_NAND_PARAMETER_DATE_CODE			65
#define SPI_NAND_PARAMETER_PAGE_BYTES			80
#define SPI_NAND_PARAMETER_PAGE_SPARE_BYTES		84
#define SPI_NAND_PARAMETER_PARTIAL_PAGE_BYTES		86
#define SPI_NAND_PARAMETER_PARTIAL_PAGE_SPARE_BYTES	90
#define SPI_NAND_PARAMETER_PAGES_PER_BLOCK		92
#define SPI_NAND_PARAMETER_BLOCKS_PER_LUN		96
#define SPI_NAND_PARAMETER_LUNS				100
#define SPI_NAND_PARAMETER_ADDRESS_CYCLES		101
#define SPI_NAND_PARAMETER_BITS_PER_CELL		102
#define SPI_NAND_PARAMETER_BAD_BLOCKS_PER_LUN		103
#define SPI_NAND_PARAMETER_BLOCK_ENDURANCE		105
#define SPI_NAND_PARAMETER_VALID_BLOCKS			107
#define SPI_NAND_PARAMETER_VALID_BLOCK_ENDURANCE	108
#define SPI_NAND_PARAMETER_PROGRAMS_PER_PAGE		110
#define SPI_NAND_PARAMETER_PARTIAL_PROGRAM_ATTRIBUTES	111
#define SPI_NAND_PARAMETER_ECC_BITS			112
#define SPI_NAND_PARAMETER_INTERLEAVED_ADDRESS_BITS	113
#define SPI_NAND_PARAMETER_INTERLEAVED_OPERATION_ATTRIBUTES	114
#define SPI_NAND_PARAMETER_PIN_CAPACITANCE		128
#define SPI_NAND_PARAMETER_TIMING_MODE			129
#define SPI_NAND_PARAMETER_CACHE_TIMING_MODE		131
#define SPI_NAND_PARAMETER_PAGE_PROGRAM_TIME		133
#define SPI_NAND_PARAMETER_BLOCK_ERASE_TIME		135
#define SPI_NAND_PARAMETER_PAGE_READ_TIME		137
#define SPI_NAND_PARAMETER_COLUMN_SETUP_TIME		139
#define SPI_NAND_PARAMETER_VENDOR_REVISION		164
#define SPI_NAND_PARAMETER_VENDOR_DATA			166
#define SPI_NAND_PARAMETER_CRC				254

#endif /*__SPI_NAND_H__*/
