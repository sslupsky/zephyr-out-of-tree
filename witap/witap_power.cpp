/**
 * @file power.cpp
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-07
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include <zephyr.h>
#include <kernel.h>
#include <soc.h>
#include <power/power.h>
#include <logging/log.h>
#include <shell/shell.h>

#include "witap_types.h"
#include "witap_power.hpp"

LOG_MODULE_DECLARE(myApp, CONFIG_MYAPP_LOG_LEVEL);

#define SECS_TO_TICKS		CONFIG_SYS_CLOCK_TICKS_PER_SEC

#ifndef CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_1
#define CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_1 1
#endif

#ifndef CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_2
#define CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_2 1
#endif

/* PM Policy based on SoC/Platform residency requirements */
static const s32_t pm_min_residency[] = {
#ifdef CONFIG_SYS_POWER_SLEEP_STATES
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_1 * SECS_TO_TICKS / MSEC_PER_SEC,
#endif

#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_2 * SECS_TO_TICKS / MSEC_PER_SEC,
#endif

#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	CONFIG_SYS_PM_MIN_RESIDENCY_SLEEP_3 * SECS_TO_TICKS / MSEC_PER_SEC,
#endif
#endif /* CONFIG_SYS_POWER_SLEEP_STATES */

#ifdef CONFIG_SYS_POWER_DEEP_SLEEP_STATES
#ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_1
	CONFIG_SYS_PM_MIN_RESIDENCY_DEEP_SLEEP_1 * SECS_TO_TICKS / MSEC_PER_SEC,
#endif

#ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_2
	CONFIG_SYS_PM_MIN_RESIDENCY_DEEP_SLEEP_2 * SECS_TO_TICKS / MSEC_PER_SEC,
#endif

#ifdef CONFIG_HAS_SYS_POWER_STATE_DEEP_SLEEP_3
	CONFIG_SYS_PM_MIN_RESIDENCY_DEEP_SLEEP_3 * SECS_TO_TICKS / MSEC_PER_SEC,
#endif
#endif /* CONFIG_SYS_POWER_DEEP_SLEEP_STATES */
};

atomic_t device_busy = 0;

extern "C" void witap_pm_busy_set(witap_pm_devices dev)
{
	atomic_set_bit(&device_busy, dev);
	return;
}

extern "C" void witap_pm_busy_clear(witap_pm_devices dev)
{
	atomic_clear_bit(&device_busy, dev);
	return;
}

extern "C" enum power_states sys_pm_policy_next_state(s32_t ticks)
{
	int i;

	if (!force_sleep) {
		if (usbState == USB_DC_CONNECTED ||
		    usbState == USB_DC_CONFIGURED ||
		    usbState == USB_DC_RESUME ||
		    usbState == USB_DC_RESET) {
			return SYS_POWER_STATE_ACTIVE;
		}

		if (application_boot) {
			return SYS_POWER_STATE_ACTIVE;
		}

		if (device_busy) {
			return SYS_POWER_STATE_ACTIVE;
		}
	}

	if ((ticks != (s32_t) K_TICKS_FOREVER) && (ticks < pm_min_residency[0])) {
		LOG_DBG("active: < min residency, ticks: %d", ticks);
		return SYS_POWER_STATE_ACTIVE;
	}

	for (i = ARRAY_SIZE(pm_min_residency) - 1; i >= 0; i--) {
#ifdef CONFIG_SYS_PM_STATE_LOCK
		if (!sys_pm_ctrl_is_state_enabled((enum power_states)(i))) {
			continue;
		}
#endif
		if ((ticks == (s32_t) K_TICKS_FOREVER) ||
		    (ticks >= pm_min_residency[i])) {
			LOG_DBG("sleep: %d, ticks: %d", i, ticks);
			return (enum power_states)(i);
		}
	}

	LOG_ERR("active: no policy");
	return SYS_POWER_STATE_ACTIVE;
}

static int cmd_sleep(const struct shell *shell, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(shell, "disconnect usb in 2 seconds ...");
	k_delayed_work_submit(&usb_disconnect_work, K_MSEC(2000));

	return 0;
}

SHELL_CMD_ARG_REGISTER(sleep, NULL, "enable power management", cmd_sleep, 1, 0);
