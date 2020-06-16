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


#include <errno.h>
#include <zephyr/types.h>
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

#include <shell/shell.h>
#include <drivers/pmic.h>
#include "bq24195.h"

#include <getopt.h>

const struct shell *ctx_shell;
static struct device *pmic_dev;

__unused
static void pmic_ready(int err)
{
	if (err) {
		shell_error(ctx_shell, "PMIC init failed (err %d)", err);
		return;
	}

	shell_print(ctx_shell, "PMIC initialized");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
}

static int cmd_init(const struct shell *shell, size_t argc, char *argv[])
{
	struct bq24195_data *drv_data;
	int ret;

	ctx_shell = shell;

	if (argc == 2) {
		pmic_dev = device_get_binding(argv[1]);
		if (!pmic_dev) {
			shell_error(shell, "device not found: %s", argv[1]);
			return -EIO;
		}
	}

	drv_data = pmic_dev->driver_data;

	/* TODO:  add pmic_enable() to gnss api ? */
	// ret = pmic_enable(pmic_ready);

	// bq24195_trigger_set(dev, &txready_trigger, bq24195_txready_handler);

	ret = 0;
	if (ret) {
		shell_error(shell, "PMIC init failed (err %d)", ret);
	}

	return ret;
}

/***********************/

static int cmd_info_version(const struct shell *shell)
{
	struct bq24195_data *drv_data;
	u8_t ver[30] = "";
	int ret = 0;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = pmic_dev->driver_data;

	pmic_attr_get(pmic_dev, PMIC_CHAN_NONE, PMIC_ATTR_INFO_VERSION, ver);
	shell_print(shell, "device version: %s", ver);

	return ret;
}

static int cmd_info_id(const struct shell *shell)
{
	struct bq24195_data *drv_data;
	u8_t id;
	int ret = 0;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = pmic_dev->driver_data;

	pmic_attr_get(pmic_dev, PMIC_CHAN_NONE, PMIC_ATTR_INFO_ID, &id);
	shell_print(shell, "device id: 0x%02x", id);

	return ret;
}


/***********************/

static int cmd_pm_sleep(const struct shell *shell)
{
	struct bq24195_data *drv_data;
	int ret = 0;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = pmic_dev->driver_data;

	return ret;
}

static int cmd_pm_status(const struct shell *shell)
{
	struct bq24195_data *drv_data;
	int ret = 0;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = pmic_dev->driver_data;

	return ret;
}

static int cmd_pm_mode(const struct shell *shell, size_t argc, char **argv)
{
	struct bq24195_data *drv_data;
	int ret = 0;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = pmic_dev->driver_data;

	return ret;
}

/***********************/

static int cmd_debug_register(const struct shell *shell, size_t argc, char **argv)
{
	struct bq24195_data *drv_data;
	const struct bq24195_dev_config *cfg;
	static const char* reg_names[] = {
		"Input Source Control",
		"Power-On Configuration",
		"Charge Current Control",
		"Pre-Charge/Termination Current Control",
		"Charge Voltage Control",
		"Charge Termination/Timer Control",
		"Thermal Regulation Control",
		"Operation Control",
		"System Status",
		"Fault",
		"Vendor",
	};
	u8_t reg = 0;
	u8_t val = 0;
	int ret = 0;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = pmic_dev->driver_data;
	cfg = pmic_dev->config->config_info;

	if (argc == 2) {
		reg = strtol(argv[1], NULL, 0);
		if (cfg->api.reg_read) {
			cfg->api.reg_read(pmic_dev, reg, &val, sizeof(val));
		}
	} else if (argc == 3) {
		reg = strtol(argv[1], NULL, 0);
		val = strtol(argv[2], NULL, 0);
		if (cfg->api.reg_write) {
			cfg->api.reg_write(pmic_dev, reg, val);
		}
	}

	shell_print(shell, "REG%02X (%s):  0x%02x", reg, reg_names[reg], val);
	return ret;
}

/***********************/

static int cmd_pmic_poll(const struct shell *shell)
{
	struct bq24195_data *drv_data;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}
	
	drv_data = pmic_dev->driver_data;

	return 0;
}

static int cmd_pmic_save(const struct shell *shell)
{
	struct bq24195_data *drv_data;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}
	
	drv_data = pmic_dev->driver_data;

	return 0;
}

static int cmd_pmic_load(const struct shell *shell)
{
	struct bq24195_data *drv_data;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}
	
	drv_data = pmic_dev->driver_data;

	return 0;
}

/***********************/

static int cmd_pmic_info(const struct shell *shell, size_t argc, char *argv[])
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

static int cmd_pmic_pm(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argv);

	if (argc == 1) {
		shell_help(shell);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(shell, "%s unknown parameter: %s", argv[0], argv[1]);

	return -EINVAL;
}

static int cmd_pmic_debug(const struct shell *shell, size_t argc, char **argv)
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

static int cmd_pmic(const struct shell *shell, size_t argc, char **argv)
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
	struct bq24195_data *drv_data;
	int ret = 0;

	if (!pmic_dev) {
		shell_error(shell, "device not found");
		return -EIO;
	}

	drv_data = pmic_dev->driver_data;

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

SHELL_STATIC_SUBCMD_SET_CREATE(pmic_info_cmds,
	SHELL_CMD(id, NULL, HELP_NONE, cmd_info_id),
	SHELL_CMD(version, NULL, HELP_NONE, cmd_info_version),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(pmic_pm_cmds,
	SHELL_CMD(mode, NULL, HELP_NONE, cmd_pm_mode),
	SHELL_CMD(sleep, NULL, HELP_NONE, cmd_pm_sleep),
	SHELL_CMD(status, NULL, HELP_NONE, cmd_pm_status),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(pmic_debug_cmds,
	SHELL_CMD_ARG(reg, NULL, HELP_NONE, cmd_debug_register, 2, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(pmic_cmds,
	SHELL_CMD(debug, &pmic_debug_cmds, HELP_NONE, cmd_pmic_debug),
	SHELL_CMD(info, &pmic_info_cmds, HELP_NONE, cmd_pmic_info),
	SHELL_CMD_ARG(init, NULL, HELP_NONE, cmd_init, 2, 0),
	SHELL_CMD(load, NULL, HELP_NONE, cmd_pmic_load),
	SHELL_CMD(pm, &pmic_pm_cmds, HELP_NONE, cmd_pmic_pm),
	SHELL_CMD(poll, NULL, HELP_NONE, cmd_pmic_poll),
	SHELL_CMD(save, NULL, HELP_NONE, cmd_pmic_save),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(pmic, &pmic_cmds, "PMIC shell commands", cmd_pmic);
