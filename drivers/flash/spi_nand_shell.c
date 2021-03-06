/**
 * @file spi_nand_shell.c
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-06-14
 * 
 * @copyright Copyright (c) 2020
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

#define DT_DRV_COMPAT jedec_spi_nand

#include <errno.h>
#include <zephyr/types.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <zephyr.h>
#include <time.h>

#include <device.h>
#include <devicetree.h>
#include <settings/settings.h>

#include <storage/flash_map.h>
#include <drivers/flash.h>
#include <drivers/spi.h>

#include <shell/shell.h>
#include "spi_nand.h"

#include <getopt.h>

static const struct shell *ctx_shell;
static struct device *spi_nand_dev;

static const struct spi_nand_register spi_nand_registers[] = {
	{.addr = SPI_NAND_FT_ADDR_LOCK, .name = "lock"},
	{.addr = SPI_NAND_FT_ADDR_CTRL, .name = "ctrl"},
	{.addr = SPI_NAND_FT_ADDR_STATUS, .name = "status"},
	{.addr = SPI_NAND_FT_ADDR_BFD, .name = "bfd"},
	{.addr = SPI_NAND_FT_ADDR_BFS, .name = "bfs"},
	{.addr = SPI_NAND_FT_ADDR_MBF, .name = "mbf"},
	{.addr = SPI_NAND_FT_ADDR_BFR0, .name = "bfr0"},
	{.addr = SPI_NAND_FT_ADDR_BFR1, .name = "bfr1"},
	{.addr = SPI_NAND_FT_ADDR_BFR2, .name = "bfr2"},
	{.addr = SPI_NAND_FT_ADDR_BFR3, .name = "bfr3"},
};

__unused
static void spi_nand_ready(int err)
{
	if (err) {
		shell_error(ctx_shell, "NAND init failed (err %d)", err);
		return;
	}

	shell_print(ctx_shell, "NAND initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
}

static int cmd_init(const struct shell *shell, size_t argc, char *argv[])
{
	struct spi_nand_data *drv_data;

	ctx_shell = shell;

	if (argc == 2) {
		spi_nand_dev = device_get_binding(argv[1]);
	} else {
		spi_nand_dev = device_get_binding(DT_INST_LABEL(0));
	}

	if (!spi_nand_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = spi_nand_dev->driver_data;

	/* TODO:  add spi_nand_enable() to api ? */
	// ret = spi_nand_enable(spi_nand_ready);

	return 0;
}

/***********************/

static int cmd_info_version(const struct shell *shell, size_t argc, char *argv[])
{
	struct device *dev;
	struct spi_nand_data *drv_data;

	if (argc == 2) {
		dev = device_get_binding(argv[1]);
	} else {
		dev = device_get_binding(DT_INST_LABEL(0));
	}

	if (!dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = dev->driver_data;

	return 0;
}

static int cmd_info_id(const struct shell *shell, size_t argc, char *argv[])
{
	struct device *dev;
	struct spi_nand_data *drv_data;
	const struct spi_nand_config *params;
	int ret;

	if (argc == 2) {
		dev = device_get_binding(argv[1]);
	} else {
		dev = device_get_binding(DT_INST_LABEL(0));
	}

	if (!dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	shell_print(shell, "Device: %s", dev->name);
	drv_data = dev->driver_data;
	params = dev->config_info;

	ret = spi_nand_read_id(dev, params);
	if (ret != 0) {
		shell_error(shell, "jedec id match failed");
		return -EIO;
	} else {
		shell_print(shell, "jedec id match");
	}

	return 0;
}

static int cmd_info_params(const struct shell *shell, size_t argc, char *argv[])
{
	struct device *dev;
	struct spi_nand_data *drv_data;
	const struct spi_nand_config *params;
	char buf[MAX(sizeof(drv_data->signature),MAX(sizeof(drv_data->manufacturer),sizeof(drv_data->model))) + 1];
	int ret;

	if (argc == 2) {
		dev = device_get_binding(argv[1]);
	} else {
		dev = device_get_binding(DT_INST_LABEL(0));
	}

	if (!dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	shell_print(shell, "Device: %s", dev->name);
	drv_data = dev->driver_data;
	params = dev->config_info;

	ret = spi_nand_read_id(dev, params);
	if (ret != 0) {
		shell_error(shell, "jedec id match failed");
		return -EIO;
	} else {
		shell_print(shell, "jedec id match");
	}

	ret = spi_nand_read_parameter_page(dev);
	if (ret < 0) {
		shell_error(shell, "could not read parameter page");
		return -EIO;
	} else {
		shell_print(shell, "read parameter page success");
	}

	snprintk(buf, MIN(sizeof(buf),sizeof(drv_data->signature)+1), "%4s", drv_data->signature);
	shell_print(shell, "Signature: %s", buf);
	snprintk(buf, MIN(sizeof(buf),sizeof(drv_data->manufacturer)+1), "%12s", drv_data->manufacturer);
	shell_print(shell, "Manufacturer: %s", buf);
	snprintk(buf, MIN(sizeof(buf),sizeof(drv_data->model)+1), "%20s", drv_data->model);
	shell_print(shell, "Model: %s", buf);
	shell_print(shell, "JEDEC ID: 0x%02x", drv_data->jedec_id);
	shell_print(shell, "Bits per cell: %d", drv_data->bits_per_cell);
	shell_print(shell, "Page size: %d", drv_data->page_size);
	shell_print(shell, "Pages per block: %d", drv_data->pages_per_block);
	shell_print(shell, "Blocks per LUN: %d", drv_data->blocks_per_lun);
	shell_print(shell, "LUN's per device: %d", drv_data->luns_per_device);
	shell_print(shell, "Block endurance: %d", drv_data->block_endurance);
	shell_print(shell, "Partial page size: %d", drv_data->partial_page_size);
	shell_print(shell, "Programs per page: %d", drv_data->programs_per_page);
	shell_print(shell, "Page program time: %d", drv_data->page_prog_time);
	shell_print(shell, "Block erase time: %d", drv_data->block_erase_time);
	shell_print(shell, "Page read time: %d", drv_data->page_read_time);

	return 0;
}

static int cmd_info_timers(const struct shell *shell, size_t argc, char *argv[])
{
	struct device *dev;
	struct spi_nand_data *drv_data;
	const struct spi_nand_config *params;

	if (argc == 2) {
		dev = device_get_binding(argv[1]);
	} else {
		dev = device_get_binding(DT_INST_LABEL(0));
	}

	if (!dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	shell_print(shell, "Device: %s", dev->name);
	drv_data = dev->driver_data;
	params = dev->config_info;

	/*
	 *  Note:  shell_print() does not support 64-bit integers.
	 */
	shell_print(shell, "Cummulative time:");
	shell_print(shell, "  page read= %d cycles\n"
			   "  page prog= %d cycles\n"
			   "  block erase= %d cycles",
		    (uint32_t) drv_data->timer[SPI_NAND_TIMER_PAGE_READ_TIME],
		     (uint32_t) drv_data->timer[SPI_NAND_TIMER_PAGE_PROG_TIME],
		     (uint32_t) drv_data->timer[SPI_NAND_TIMER_BLOCK_ERASE_TIME]);
	shell_print(shell, "Cummulative count:");
	shell_print(shell, "  page read= %d\n"
			   "  page prog= %d\n"
			   "  block erase= %d",
		    drv_data->counter[SPI_NAND_TIMER_PAGE_READ_TIME],
		    drv_data->counter[SPI_NAND_TIMER_PAGE_PROG_TIME],
		    drv_data->counter[SPI_NAND_TIMER_BLOCK_ERASE_TIME]);
	shell_print(shell, "Average time:");
	shell_print(shell, "  page read= %d cycles\n"
			   "  page prog= %d cycles\n"
			   "  block erase= %d cycles",
		     (uint32_t) (drv_data->timer[SPI_NAND_TIMER_PAGE_READ_TIME] / drv_data->counter[SPI_NAND_TIMER_PAGE_READ_TIME]),
		     (uint32_t) (drv_data->timer[SPI_NAND_TIMER_PAGE_PROG_TIME] / drv_data->counter[SPI_NAND_TIMER_PAGE_PROG_TIME]),
		     (uint32_t) (drv_data->timer[SPI_NAND_TIMER_BLOCK_ERASE_TIME] / drv_data->counter[SPI_NAND_TIMER_BLOCK_ERASE_TIME]));


	return 0;
}


/***********************/

static int cmd_nand_sleep(const struct shell *shell)
{
	struct spi_nand_data *drv_data;
	int ret = 0;

	if (!spi_nand_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = spi_nand_dev->driver_data;

	return ret;
}

static int cmd_nand_status(const struct shell *shell, size_t argc, char *argv[])
{
	struct device *dev;
	struct spi_nand_data *drv_data;
	uint8_t status, ctrl, lock;

	if (argc == 2) {
		dev = device_get_binding(argv[1]);
	} else {
		dev = device_get_binding(DT_INST_LABEL(0));
	}

	if (!dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = dev->driver_data;

	/*  this is a private spi_nand function call  */
	spi_nand_get_registers(dev, &status, &ctrl, &lock);
	shell_print(shell, "Device: %s", dev->name);
	shell_print(shell, "status: 0x%02x, ctrl: 0x%02x, lock: 0x%02x", status, ctrl, lock);

	return 0;
}

static int cmd_nand_mode(const struct shell *shell, size_t argc, char **argv)
{
	struct spi_nand_data *drv_data;
	int ret = 0;

	if (!spi_nand_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = spi_nand_dev->driver_data;

	return ret;
}

/***********************/

static int cmd_debug_register(const struct shell *shell, size_t argc, char **argv)
{
	struct spi_nand_data *drv_data;
	const struct spi_nand_config *cfg;
	u8_t reg = 0;
	u8_t val = 0;
	int ret = 0;

	if (!spi_nand_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	shell_print(shell, "Device: %s", spi_nand_dev->name);

	drv_data = spi_nand_dev->driver_data;
	cfg = spi_nand_dev->config_info;

	if (argc == 2) {
		if (strncmp(argv[1], "all", sizeof("all")) == 0) {
			for (reg = 0; reg < ARRAY_SIZE(spi_nand_registers); reg++) {
				if (cfg->api.reg_read) {
					cfg->api.reg_read(spi_nand_dev, spi_nand_registers[reg].addr, &val);
					shell_print(shell, "REG%02X (%s):  0x%02x", spi_nand_registers[reg].addr, spi_nand_registers[reg].name, val);
				}
			}
		} else {
			reg = strtol(argv[1], NULL, 0);
			if (cfg->api.reg_read && reg >=0 && reg <= ARRAY_SIZE(spi_nand_registers)) {
				cfg->api.reg_read(spi_nand_dev, spi_nand_registers[reg].addr, &val);
				shell_print(shell, "REG%02X (%s):  0x%02x", spi_nand_registers[reg].addr, spi_nand_registers[reg].name, val);
			}
		}
	} else if (argc == 3) {
		reg = strtol(argv[1], NULL, 0);
		val = strtol(argv[2], NULL, 0);
		if (cfg->api.reg_write && reg >=0 && reg <= ARRAY_SIZE(spi_nand_registers)) {
			cfg->api.reg_write(spi_nand_dev, spi_nand_registers[reg].addr, val);
			shell_print(shell, "REG%02X (%s):  0x%02x", spi_nand_registers[reg].addr, spi_nand_registers[reg].name, val);
		}
	}

	return ret;
}

/**
 * @brief Read data from flash and print to screen
 * 
 * 	  This function reads a block of data from the flash partition
 *        and dumps the data to the screen in hex and ascii format
 * 
 * @param shell 
 * @param argc 
 * @param argv <flash area partition id> <count> <memory offset into partition>
 * @return int 
 */
static int cmd_debug_read(const struct shell *shell, size_t argc, char **argv)
{
	const struct flash_area *pfa;
	uint8_t id;
	off_t offset;
	int count;
	int ret;

	if (argc > 1) {
		id = strtol(argv[1], NULL, 0);
	} else {
		shell_error(shell, "partition id required");
		return -ENODEV;
	}

	if (argc > 2) {
		count = strtol(argv[1], NULL, 0);
		if (count <= 0) {
			count = 256;
		}
	} else {
		count = 256;
	}

	if (argc > 3) {
		offset = strtol(argv[2], NULL, 0);
	} else {
		offset = 0;
	}

	ret = flash_area_open(id, &pfa);
	if (ret < 0) {
		shell_error(shell, "could not open partition, id=%u, ret=%d",
		       id, ret);
		return -ENODEV;
	}

	shell_print(shell, "partition: %u at 0x%x on %s for %u bytes",
	       id, (unsigned int)pfa->fa_off,
	       pfa->fa_dev_name,
	       (unsigned int)pfa->fa_size);

	shell_print(shell,"raw flash data addr: 0x%08x, len: 0x%08x", offset, count);
	while (count > 0) {
		ssize_t read;
		u8_t buf[16];
		int i;

		read = MIN(count, (int) sizeof(buf));
		ret = flash_area_read(pfa, offset, buf, read);
		if (ret < 0) {
			break;
		}

		shell_fprintf(shell, SHELL_NORMAL, "%08X  ", offset);

		for (i = 0; i < read; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%02X ", buf[i]);
		}
		for (; i < (int) sizeof(buf); i++) {
			shell_fprintf(shell, SHELL_NORMAL, "   ");
		}
		i = sizeof(buf) - i;
		shell_fprintf(shell, SHELL_NORMAL, "%*c", i*3, ' ');

		for (i = 0; i < read; i++) {
			shell_fprintf(shell, SHELL_NORMAL, "%c", buf[i] < 32 ||
				      buf[i] > 127 ? '.' : buf[i]);
		}

		shell_print(shell, "");

		offset += read;
		count -= read;
	}
	flash_area_close(pfa);
	return 0;
}

/**
 * @brief Erase a partition.  This function fetches the partition parameters
 *        from the flash area and erases the data on the partition.
 * 
 * @param shell 
 * @param argc 
 * @param argv <flash area partition id>
 * @return int 
 */
static int cmd_erase(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	const struct flash_area *pfa;
	uint8_t id;
	int ret;

	if (argc > 1) {
		id = strtol(argv[1], NULL, 0);
	} else {
		shell_error(shell, "area (partition) id required");
		return -ENODEV;
	}

	ret = flash_area_open(id, &pfa);
	if (ret < 0) {
		shell_error(shell, "could not open partition, id=%u, ret=%d",
		       id, ret);
		return -ENODEV;
	}

	shell_print(shell, "partition: %u at 0x%x on %s for %u bytes",
	       id, (unsigned int)pfa->fa_off,
	       pfa->fa_dev_name,
	       (unsigned int)pfa->fa_size);

	shell_print(shell, "erasing flash");
	ret = flash_area_erase(pfa, 0, pfa->fa_size);
	if (ret < 0) {
		shell_print(shell, "flash erase error, ret=%d", ret);
		goto done;
	} else {
		shell_print(shell, "done");
	}

	flash_area_close(pfa);
done:
	return ret;
}

/***********************/

static int cmd_spi_nand_poll(const struct shell *shell)
{
	struct spi_nand_data *drv_data;

	if (!spi_nand_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}
	
	drv_data = spi_nand_dev->driver_data;

	return 0;
}

static int cmd_spi_nand_save(const struct shell *shell)
{
	struct spi_nand_data *drv_data;

	if (!spi_nand_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}
	
	drv_data = spi_nand_dev->driver_data;

	return 0;
}

static int cmd_spi_nand_load(const struct shell *shell)
{
	struct spi_nand_data *drv_data;

	if (!spi_nand_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}
	
	drv_data = spi_nand_dev->driver_data;

	return 0;
}

/***********************/

static int cmd_spi_nand_info(const struct shell *shell, size_t argc, char *argv[])
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

static int cmd_spi_nand_pm(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

static int cmd_spi_nand_debug(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

/***********************/

static int cmd_nand(const struct shell *shell, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}


/***********************/

__unused
static int cmd_skeleton(const struct shell *shell)
{
	struct spi_nand_data *drv_data;
	int ret = 0;

	if (!spi_nand_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = spi_nand_dev->driver_data;

	return ret;
}

__unused
static int cmd_getopt_skeleton(const struct shell *shell, size_t argc, char **argv)
{
	int ch, ret;
	int c_option, n_option;
	bool q_option = false;
	int longoption1, longoption2;

	static struct option long_options[] = {
		{"longOption1",	required_argument,	0,	0},
		{"longOption2",	no_argument,		0,	0},
		{0,		0,			0,	0},
	};

	ARG_UNUSED(long_options);
	ARG_UNUSED(longoption1);
	ARG_UNUSED(longoption2);

	/* reset getopt() with optind=0 so that successive calls
	 * to getopts() parses the args.  Otherwise getops()
	 * remembers the last parse and returns -1 when
	 * tail is run more than once.
	 * see linux man page
	 * http://man7.org/linux/man-pages/man3/getopt.3.html
	 */
	optind = 0;

	while ((ch = getopt(argc, argv, "c:n:q")) != -1 ) {
		switch (ch) {
		case 'c':
			c_option = strtol(optarg, NULL, 0);
			break;
		case 'n':
			n_option = strtol(optarg, NULL, 0);
			break;
		case 'q':
			q_option = true;
			break;
		case '?':
		default:
			return -ENOTSUP;
		}
	}
	argc -= optind;
	argv += optind;

	return ret;
}

#define HELP_NONE "[none]"

SHELL_STATIC_SUBCMD_SET_CREATE(spi_nand_info_cmds,
	SHELL_CMD(id, NULL, HELP_NONE, cmd_info_id),
	SHELL_CMD(version, NULL, HELP_NONE, cmd_info_version),
	SHELL_CMD(params, NULL, "params - display nand parameter data", cmd_info_params),
	SHELL_CMD(timers, NULL, "timers - display nand timer data", cmd_info_timers),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(spi_nand_pm_cmds,
	SHELL_CMD(mode, NULL, HELP_NONE, cmd_nand_mode),
	SHELL_CMD(sleep, NULL, HELP_NONE, cmd_nand_sleep),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(spi_nand_debug_cmds,
	SHELL_CMD_ARG(reg, NULL, HELP_NONE, cmd_debug_register, 2, 0),
	SHELL_CMD_ARG(read, NULL, "read [count [offset]] - read raw data from nand", cmd_debug_read, 2, 1),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(spi_nand_cmds,
	SHELL_CMD(debug, &spi_nand_debug_cmds, HELP_NONE, cmd_spi_nand_debug),
	SHELL_CMD(info, &spi_nand_info_cmds, HELP_NONE, cmd_spi_nand_info),
	SHELL_CMD_ARG(init, NULL, HELP_NONE, cmd_init, 1, 1),
	SHELL_CMD(load, NULL, HELP_NONE, cmd_spi_nand_load),
	SHELL_CMD(pm, &spi_nand_pm_cmds, HELP_NONE, cmd_spi_nand_pm),
	SHELL_CMD(poll, NULL, HELP_NONE, cmd_spi_nand_poll),
	SHELL_CMD(save, NULL, HELP_NONE, cmd_spi_nand_save),
	SHELL_CMD(erase, NULL, HELP_NONE, cmd_erase),
	SHELL_CMD(status, NULL, HELP_NONE, cmd_nand_status),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(nand, &spi_nand_cmds, "NAND shell commands", cmd_nand);
