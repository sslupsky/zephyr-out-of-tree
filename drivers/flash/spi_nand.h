/*
 * Copyright (c) 2018 Savoir-Faire Linux.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file spi_nand.h
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-05
 * 
 * @copyright Copyright (c) 2022
 * SPDX-License-Identifier: Apache-2.0
 * 
 *    _____                 _                _        _          
 *   / ____|               (_)              | |      (_)         
 *  | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
 *   \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
 *   ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 *  |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
 *                                                               
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
#define SPI_NAND_PARAMETER_PAGE_ADDRESS	DT_INST_PROP(0, parameter_page_address)

/* Test whether offset is aligned. */
#define SPI_NAND_IS_PAGE_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_PAGE_SIZE - 1U)) == 0)
#define SPI_NAND_IS_SECTOR_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_SECTOR_SIZE - 1U)) == 0)
#define SPI_NAND_IS_BLOCK_ALIGNED(_ofs) (((_ofs) & (SPI_NAND_BLOCK_SIZE - 1U)) == 0)

/* Status register bits */
#define SPI_NAND_STATUS_OIP_BIT         BIT(0)  /* Write in progress */
#define SPI_NAND_STATUS_WEL_BIT         BIT(1)  /* Write enable latch */
#define SPI_NAND_STATUS_ERASEF_BIT	BIT(2)
#define SPI_NAND_STATUS_PROGF_BIT	BIT(3)
#define SPI_NAND_STATUS_ECC_BIT	 	(BIT(5) | BIT(4))
#define SPI_NAND_STATUS_ECC_POS	 	4
#define SPI_NAND_STATUS_ERROR_BITS (	\
	   SPI_NAND_STATUS_ERASEF_BIT	\
	|| SPI_NAND_STATUS_PROGF_BIT	\
	|| SPI_NAND_STATUS_ECC_BIT	\
)

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
#define SPI_NAND_POWER_UP_WAIT		DT_INST_PROP(0, power_up_wait)  /* tPUW min power up time (us) */
#define SPI_NAND_TVSL			DT_INST_PROP(0, tvsl)         	/* tVSL vcc rising power up time (us) */

#define SPI_NAND_OPCODE_LEN		1
#define SPI_NAND_ADDR_SIZE_MAX		3
#define SPI_NAND_WAIT_MAX		2
#define SPI_NAND_HEADER_SIZE_MAX	(SPI_NAND_OPCODE_LEN + SPI_NAND_ADDR_SIZE_MAX + SPI_NAND_WAIT_MAX)

enum SPI_NAND_FEATURE_TABLE_ADDRESS_e {
	SPI_NAND_FT_ADDR_LOCK		= 0xA0,
	SPI_NAND_FT_ADDR_CTRL		= 0xB0,
	SPI_NAND_FT_ADDR_STATUS		= 0xC0,
	SPI_NAND_FT_ADDR_BFD		= 0x10,
	SPI_NAND_FT_ADDR_BFS		= 0x20,
	SPI_NAND_FT_ADDR_MBF		= 0x30,
	SPI_NAND_FT_ADDR_BFR0		= 0x40,
	SPI_NAND_FT_ADDR_BFR1		= 0x50,
	SPI_NAND_FT_ADDR_BFR2		= 0x60,
	SPI_NAND_FT_ADDR_BFR3		= 0x70,
} __packed;
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

#define SPI_NAND_PARAMETER_SIGNATURE_LEN		4
#define SPI_NAND_PARAMETER_ONFI_REVISION_LEN		2
#define SPI_NAND_PARAMETER_FEATURES_LEN			2
#define SPI_NAND_PARAMETER_OPTIONAL_CMDS_LEN		2
#define SPI_NAND_PARAMETER_DEVICE_MFG_LEN		12
#define SPI_NAND_PARAMETER_DEVICE_MODEL_LEN		20
#define SPI_NAND_PARAMETER_JEDEC_ID_LEN			1
#define SPI_NAND_PARAMETER_DATE_CODE_LEN		2
#define SPI_NAND_PARAMETER_PAGE_BYTES_LEN		4
#define SPI_NAND_PARAMETER_PAGE_SPARE_BYTES_LEN		2
#define SPI_NAND_PARAMETER_PARTIAL_PAGE_BYTES_LEN	4
#define SPI_NAND_PARAMETER_PARTIAL_PAGE_SPARE_BYTES_LEN	2
#define SPI_NAND_PARAMETER_PAGES_PER_BLOCK_LEN		4
#define SPI_NAND_PARAMETER_BLOCKS_PER_LUN_LEN		4
#define SPI_NAND_PARAMETER_LUNS_LEN			1
#define SPI_NAND_PARAMETER_ADDRESS_CYCLES_LEN		1
#define SPI_NAND_PARAMETER_BITS_PER_CELL_LEN		1
#define SPI_NAND_PARAMETER_BAD_BLOCKS_PER_LUN_LEN	2
#define SPI_NAND_PARAMETER_BLOCK_ENDURANCE_LEN		2
#define SPI_NAND_PARAMETER_VALID_BLOCKS_LEN		1
#define SPI_NAND_PARAMETER_VALID_BLOCK_ENDURANCE_LEN	2
#define SPI_NAND_PARAMETER_PROGRAMS_PER_PAGE_LEN	1
#define SPI_NAND_PARAMETER_PARTIAL_PROGRAM_ATTRIBUTES_LEN	1
#define SPI_NAND_PARAMETER_ECC_BITS_LEN			1
#define SPI_NAND_PARAMETER_INTERLEAVED_ADDRESS_BITS_LEN	1
#define SPI_NAND_PARAMETER_INTERLEAVED_OPERATION_ATTRIBUTES_LEN	1
#define SPI_NAND_PARAMETER_PIN_CAPACITANCE_LEN		1
#define SPI_NAND_PARAMETER_TIMING_MODE_LEN		2
#define SPI_NAND_PARAMETER_CACHE_TIMING_MODE_LEN	2
#define SPI_NAND_PARAMETER_PAGE_PROGRAM_TIME_LEN	2
#define SPI_NAND_PARAMETER_BLOCK_ERASE_TIME_LEN		2
#define SPI_NAND_PARAMETER_PAGE_READ_TIME_LEN		2
#define SPI_NAND_PARAMETER_COLUMN_SETUP_TIME_LEN	2
#define SPI_NAND_PARAMETER_VENDOR_REVISION_LEN		2
#define SPI_NAND_PARAMETER_VENDOR_DATA_LEN		88
#define SPI_NAND_PARAMETER_CRC_LEN			2

struct spi_nand_register {
	enum SPI_NAND_FEATURE_TABLE_ADDRESS_e addr;
	char *name;
};

/**
 * @typedef spi_nand_reg_read_t
 * @brief Callback API upon reading a sensor's register
 *
 * See spi_nand_reg_read() for argument description
 */
typedef 
int (*spi_nand_reg_read_t)(struct device *dev, u8_t reg, u8_t *val);

/**
 * @typedef spi_nand_reg_write_t
 * @brief Callback API upon writing a sensor's register
 *
 * See spi_nand_reg_write() for argument description
 */
typedef
int (*spi_nand_reg_write_t)(struct device *dev, u8_t reg, u8_t val);


struct spi_nand_driver_api {
	spi_nand_reg_read_t reg_read;
	spi_nand_reg_write_t reg_write;
};

struct spi_nand_config {
	/* JEDEC id from devicetree */
	u8_t id[SPI_NAND_JEDEC_ID_LEN];

	/* spi mode */
	u8_t spi_mode;

	/* Size from devicetree, in bytes */
	u32_t size;
	struct spi_nand_driver_api api;
};

#if DT_INST_NODE_HAS_PROP(0, t_enter_dpd)
#define T_DP_MS ceiling_fraction(DT_INST_PROP(0, t_enter_dpd), NSEC_PER_MSEC)
#else /* T_ENTER_DPD */
#define T_DP_MS 0
#endif /* T_ENTER_DPD */
#if DT_INST_NODE_HAS_PROP(0, t_exit_dpd)
#define T_RES1_MS ceiling_fraction(DT_INST_PROP(0, t_exit_dpd), NSEC_PER_MSEC)
#endif /* T_EXIT_DPD */
#if DT_INST_NODE_HAS_PROP(0, dpd_wakeup_sequence)
#define T_DPDD_MS ceiling_fraction(DT_INST_PROP(0, dpd_wakeup_sequence_0), NSEC_PER_MSEC)
#define T_CRDP_MS ceiling_fraction(DT_INST_PROP(0, dpd_wakeup_sequence_1), NSEC_PER_MSEC)
#define T_RDP_MS ceiling_fraction(DT_INST_PROP(0, dpd_wakeup_sequence_2), NSEC_PER_MSEC)
#else /* DPD_WAKEUP_SEQUENCE */
#define T_DPDD_MS 0
#endif /* DPD_WAKEUP_SEQUENCE */

/**
 * struct spi_nand_data - Structure for defining the SPI NAND access
 * @spi: The SPI device
 * @spi_cfg: The SPI configuration
 * @cs_ctrl: The GPIO pin used to emulate the SPI CS if required
 * @sem: The semaphore to access to the flash
 */
struct spi_nand_data {
	struct device *spi;
	struct spi_config spi_cfg;
#if DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
	struct spi_cs_control cs_ctrl;
#endif /* DT_INST_SPI_DEV_HAS_CS_GPIOS(0) */
#if DT_INST_NODE_HAS_PROP(0, has_dpd)
	/* Low 32-bits of uptime counter at which device last entered
	 * deep power-down.
	 */
	u32_t ts_enter_dpd;
#endif /* DT_INST_NODE_HAS_PROP(0, has_dpd) */
	char signature[SPI_NAND_PARAMETER_SIGNATURE_LEN];
	char manufacturer[SPI_NAND_PARAMETER_DEVICE_MFG_LEN];
	char model[SPI_NAND_PARAMETER_DEVICE_MODEL_LEN];
	uint32_t page_size;
	uint32_t partial_page_size;
	uint32_t pages_per_block;
	uint32_t blocks_per_lun;
	uint8_t  luns_per_device;
	uint8_t  programs_per_page;
	uint8_t  jedec_id;
	uint8_t  bits_per_cell;
	uint16_t block_endurance;
	uint16_t page_prog_time;
	uint16_t block_erase_time;
	uint16_t page_read_time;
	struct k_sem sem;
};

/* Forward declaration of private "C" functions */
/* used in spi_nand_shell.c */
void spi_nand_get_registers(struct device *dev, u8_t *status, u8_t *ctrl, u8_t *lock);
int spi_nand_read_id(struct device *dev,
				  const struct spi_nand_config *const flash_id);
int spi_nand_read_parameter_page(struct device *dev);

#endif /*__SPI_NAND_H__*/
