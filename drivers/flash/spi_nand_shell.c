/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-06-14
 * 
 * @copyright Copyright (c) 2020
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
	spi_nand_read_parameter_page(dev);
	shell_print(shell, "Signature: %.*s", sizeof(drv_data->signature), drv_data->signature);
	shell_print(shell, "Manufacturer: %.*s", sizeof(drv_data->manufacturer), drv_data->manufacturer);
	shell_print(shell, "Model: %.*s", sizeof(drv_data->model), drv_data->model);

	return 0;
}

static int cmd_info_params(const struct shell *shell, size_t argc, char *argv[])
{
	struct device *dev;
	struct spi_nand_data *drv_data;
	char buf[21];

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
	spi_nand_read_parameter_page(dev);
	snprintf(buf, sizeof(buf), "%.*s", sizeof(drv_data->signature), drv_data->signature);
	shell_print(shell, "Signature: %s", buf);
	snprintf(buf, sizeof(buf), "%.*s", sizeof(drv_data->manufacturer), drv_data->manufacturer);
	shell_print(shell, "Manufacturer: %s", buf);
	snprintf(buf, sizeof(buf), "%.*s", sizeof(drv_data->model), drv_data->model);
	shell_print(shell, "Model: %s", buf);
	shell_print(shell, "Page size: %d", drv_data->page_size);
	shell_print(shell, "Pages per block: %d", drv_data->pages_per_block);
	shell_print(shell, "Blocks per LUN: %d", drv_data->blocks_per_lun);
	shell_print(shell, "LUN's per device: %d", drv_data->luns_per_device);
	shell_print(shell, "Block endurance: %d", drv_data->block_endurance);
	shell_print(shell, "Partial page size: %d", drv_data->partial_page_size);
	shell_print(shell, "Programs per page: %d", drv_data->programs_per_page);

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

	spi_nand_get_registers(dev, &status, &ctrl, &lock);
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
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(spi_nand_pm_cmds,
	SHELL_CMD(mode, NULL, HELP_NONE, cmd_nand_mode),
	SHELL_CMD(sleep, NULL, HELP_NONE, cmd_nand_sleep),
	SHELL_CMD(status, NULL, HELP_NONE, cmd_nand_status),
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
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(nand, &spi_nand_cmds, "NAND shell commands", cmd_nand);
