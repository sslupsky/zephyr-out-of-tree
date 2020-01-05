/*
 * Copyright (c) 2019, Steven Slupsky
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <power/power.h>
#include <soc.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(soc, CONFIG_SOC_LOG_LEVEL);

/*
 * Waits for RTC bus synchronization.
 */
static inline void rtc_sync(void)
{
	while (RTC->MODE0.STATUS.reg & RTC_STATUS_SYNCBUSY) {
		/* Wait for bus synchronization... */
	}
}

/*
 * Power state map:
 * SYS_POWER_STATE_SLEEP_1: IDLE
 * SYS_POWER_STATE_SLEEP_2: STANDBY
 * SYS_POWER_STATE_SLEEP_3: Stop
 */

/* Invoke Low Power/System Off specific Tasks */
void sys_set_power_state(enum power_states state)
{
	LOG_DBG("SoC entering power state %d", state);

	/* FIXME: When this function is entered the Kernel has disabled
	 * interrupts using BASEPRI register. This is incorrect as it prevents
	 * waking up from any interrupt which priority is not 0. Work around the
	 * issue and disable interrupts using PRIMASK register as recommended
	 * by ARM.
	 */

	/* Set PRIMASK */
	__disable_irq();
	/* Set BASEPRI to 0 */
	irq_unlock(0);

	switch (state) {
#ifdef CONFIG_SYS_POWER_SLEEP_STATES
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_1
	case SYS_POWER_STATE_SLEEP_1:
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
        PM->SLEEP.reg = 2;
        __DSB();
        __WFI();
		break;
#endif /* CONFIG_HAS_SYS_POWER_STATE_SLEEP_1 */
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_2
	case SYS_POWER_STATE_SLEEP_2:
        // Disable systick interrupt:  See https://www.avrfreaks.net/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
        // If SysTick is not enabled then no need to mask the interrupt
#ifdef CONFIG_CORTEX_M_SYSTICK
        SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
#endif
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
        __DSB();
        __WFI();
        // Re-enable systick interrupt if SysTick is enabled
#ifdef CONFIG_CORTEX_M_SYSTICK
        SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;	
#endif
		break;
#endif /* CONFIG_HAS_SYS_POWER_STATE_SLEEP_2 */
#ifdef CONFIG_HAS_SYS_POWER_STATE_SLEEP_3
	case SYS_POWER_STATE_SLEEP_3:
		break;
#endif /* CONFIG_HAS_SYS_POWER_STATE_SLEEP_3 */
#endif /* CONFIG_SYS_POWER_SLEEP_STATES */
	default:
		LOG_DBG("Unsupported power state %u", state);
		break;
	}

	LOG_DBG("SoC leaving power state %d", state);

	/* Clear PRIMASK */
	__enable_irq();
}

/* Handle SOC specific activity after Low Power Mode Exit */
void _sys_pm_power_state_exit_post_ops(enum power_states state)
{
	ARG_UNUSED(state);
}
