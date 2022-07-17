/**
 * @file fram_mb85rs.h
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-16
 * 
 * @copyright Copyright (c) 2022
 * 
 *    _____                 _                _        _          
 *   / ____|               (_)              | |      (_)         
 *  | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
 *   \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
 *   ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 *  |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
 *                                                               
 */

/*
 * SPDX-License-Identifier: Apache-2.0
 */

/* MB85RS instruction set */
#define FRAM_MB85RS_CMD_WRSR  0x01U /* Write STATUS register        */
#define FRAM_MB85RS_CMD_WRITE 0x02U /* Write data to memory array   */
#define FRAM_MB85RS_CMD_READ  0x03U /* Read data from memory array  */
#define FRAM_MB85RS_CMD_WRDI  0x04U /* Reset the write enable latch */
#define FRAM_MB85RS_CMD_RDSR  0x05U /* Read STATUS register         */
#define FRAM_MB85RS_CMD_WREN  0x06U /* Set the write enable latch   */
#define FRAM_MB85RS_CMD_FSTRD 0x0bU /* Fast Read */
#define FRAM_MB85RS_CMD_SLEEP 0xb9U /* Sleep / Deep Power Down */
#define FRAM_MB85RS_CMD_RDID  0x9fU /* Read Device ID */

/* MB85RS status register bits */
#define FRAM_MB85RS_STATUS_WIP  BIT(1) /* Write In Process (RO) - These devices have no write delay so this bit always reads 0 */
#define FRAM_MB85RS_STATUS_WEL  BIT(1) /* Write Enable Latch (RO) */
#define FRAM_MB85RS_STATUS_BP0  BIT(2) /* Block Protection 0 (RW) */
#define FRAM_MB85RS_STATUS_BP1  BIT(3) /* Block Protection 1 (RW) */
#define FRAM_MB85RS_STATUS_WPEN BIT(7) /* Write Protect (RW) */

struct fram_mb85rsx_device_id {
	uint8_t manufacturer_id;
	uint8_t continuation_code;
	uint8_t product_id[2];
};

struct fram_mb85rsx_config {
	const char *bus_dev_name;
	uint16_t bus_addr;
	uint32_t max_freq;
	const char *spi_cs_dev_name;
	uint8_t spi_cs_pin;
	gpio_pin_t wp_gpio_pin;
	gpio_dt_flags_t wp_gpio_flags;
	const char *wp_gpio_name;
	size_t size;
	size_t pagesize;
	uint8_t addr_width;
	bool readonly;
	uint16_t timeout;
	fram_api_read read_fn;
	fram_api_write write_fn;
};

struct fram_mb85rsx_data {
	struct device *bus_dev;
	struct fram_mb85rsx_device_id id;
#ifdef CONFIG_FRAM_MB85RS
	struct spi_config spi_cfg;
	struct spi_cs_control spi_cs;
#endif /* CONFIG_FRAM_MB85RS */
	struct device *wp_gpio_dev;
	struct k_mutex lock;
};

struct fram_mb85rsx_device_id fram_mb85rsx_id(struct device *dev);