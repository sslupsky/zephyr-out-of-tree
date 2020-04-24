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

#include "witap_types.h"

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

extern "C" enum power_states sys_pm_policy_next_state(s32_t ticks)
{
	int i;

	if (usbState != USB_DC_DISCONNECTED) {
		return SYS_POWER_STATE_ACTIVE;
	}

	if (application_boot) {
		return SYS_POWER_STATE_ACTIVE;
	}

	// if ( device_any_busy_check() ) {
	//     LOG_DBG("Device busy");
	//     return SYS_POWER_STATE_ACTIVE;
	// }

	if ((ticks != K_TICKS_FOREVER) && (ticks < pm_min_residency[0])) {
		LOG_DBG("active: < min residency, ticks: %d", ticks);
		return SYS_POWER_STATE_ACTIVE;
	}

	for (i = ARRAY_SIZE(pm_min_residency) - 1; i >= 0; i--) {
#ifdef CONFIG_SYS_PM_STATE_LOCK
		if (!sys_pm_ctrl_is_state_enabled((enum power_states)(i))) {
			continue;
		}
#endif
		if ((ticks == K_TICKS_FOREVER) ||
		    (ticks >= pm_min_residency[i])) {
			LOG_DBG("sleep: %d, ticks: %d", i, ticks);
			return (enum power_states)(i);
		}
	}

	LOG_ERR("active: no policy");
	return SYS_POWER_STATE_ACTIVE;
}
