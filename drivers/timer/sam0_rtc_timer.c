/*
 * Copyright (c) 2018 omSquare s.r.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam0_rtc

/**
 * @file
 * @brief Atmel SAM0 series RTC-based system timer
 *
 * This system timer implementation supports both tickless and ticking modes.
 * In tickless mode, RTC counts continually in 32-bit mode and timeouts are
 * scheduled using the RTC comparator. In ticking mode, RTC is configured to
 * generate an interrupt every tick.
 */

#include <soc.h>
#include <drivers/clock_control.h>
#include <drivers/timer/system_timer.h>
#include <sys_clock.h>
#include <device.h>
#include <drivers/gpio.h>
#include <posix/time.h>
#include <sys/timeutil.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(rtc_sam0, LOG_LEVEL_DBG);

/* RTC registers. */
#define RTC0 ((RtcMode0 *) DT_INST_REG_ADDR(0))

#ifdef MCLK
#define RTC_CLOCK_HW_CYCLES_PER_SEC SOC_ATMEL_SAM0_OSC32K_FREQ_HZ
#else
#define RTC_CLOCK_HW_CYCLES_PER_SEC ATMEL_SAM0_DT_RTC_FREQ_HZ(0)
#endif

/* Number of sys timer cycles per on tick. */
#define CYCLES_PER_TICK (RTC_CLOCK_HW_CYCLES_PER_SEC \
			 / CONFIG_SYS_CLOCK_TICKS_PER_SEC)

/* Maximum number of ticks. */
#define MAX_TICKS (UINT32_MAX / CYCLES_PER_TICK - 2)

#ifdef CONFIG_TICKLESS_KERNEL

/*
 * Due to the nature of clock synchronization, reading from or writing to some
 * RTC registers takes approximately six RTC_GCLK cycles.
 *
 * The actual delay is 6 GCLK + 3 APB cycles. Generally, APB >> GCLK.
 * However, if for some reason, the clocks were the same, then the delay would
 * be equivalent to 9 cycles.
 *
 * We will assume APB >> GCLK and "round up" to 7 GCLK cycles.
 * The worst case update time after sleep requires 2 sync busy synchronizations.
 * Thus, the threshold is 7 x 2 = 14 cycles.
 *
 * This constant defines a safe threshold for the comparator.
 */
#define TICK_THRESHOLD 14

BUILD_ASSERT(CYCLES_PER_TICK > TICK_THRESHOLD,
	     "CYCLES_PER_TICK must be greater than TICK_THRESHOLD for "
	     "tickless mode");

#else /* !CONFIG_TICKLESS_KERNEL */

/*
 * For some reason, RTC does not generate interrupts when COMP == 0,
 * MATCHCLR == 1 and PRESCALER == 0. So we need to check that CYCLES_PER_TICK
 * is more than one.
 */
BUILD_ASSERT(CYCLES_PER_TICK > 1,
	     "CYCLES_PER_TICK must be greater than 1 for ticking mode");

#endif /* CONFIG_TICKLESS_KERNEL */

/* Helper macro to get the correct GCLK GEN based on configuration. */
#define GCLK_GEN(n) GCLK_EVAL(n)
#define GCLK_EVAL(n) GCLK_CLKCTRL_GEN_GCLK##n

/* Tick/cycle count of the last announce call. */
static volatile uint32_t rtc_last;
static volatile int64_t rtc_boot_uptime;
static volatile int64_t uptime;

#ifndef CONFIG_TICKLESS_KERNEL

/* Current tick count. */
static volatile uint32_t rtc_counter;

/* Tick value of the next timeout. */
static volatile uint32_t rtc_timeout;

#endif /* CONFIG_TICKLESS_KERNEL */

#if DT_NODE_EXISTS(DT_NODELABEL(debug0))
extern struct device *debug0_dev;
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(debug1))
extern struct device *debug1_dev;
#endif

/*
 * Waits for RTC bus synchronization.
 */
static inline void rtc_sync(void)
{
#if DT_NODE_EXISTS(DT_NODELABEL(debug1))
	if (debug1_dev) {
		// gpio_pin_set(debug1_dev, DT_ALIAS_DEBUG1_GPIOS_PIN, 1);
	}
#endif
	/* Wait for bus synchronization... */
#ifdef RTC_STATUS_SYNCBUSY
	while (RTC0->STATUS.reg & RTC_STATUS_SYNCBUSY) {
	}
#else
	while (RTC0->SYNCBUSY.reg) {
	}
#endif
#if DT_NODE_EXISTS(DT_NODELABEL(debug1))
	if (debug1_dev) {
		// gpio_pin_set(debug1_dev, DT_ALIAS_DEBUG1_GPIOS_PIN, 0);
	}
#endif
}

/*
 * Reads RTC COUNT register. First a read request must be written to READREQ,
 * then - when bus synchronization completes - the COUNT register is read and
 * returned.
 */
static uint32_t rtc_count(void)
{
#ifdef RTC_READREQ_RREQ
	rtc_sync();
	if ((RTC0->READREQ.bit.RCONT) == 0) {
		RTC0->READREQ.reg |= RTC_READREQ_RCONT;
		RTC0->READREQ.reg |= RTC_READREQ_RREQ;
		rtc_sync();
	}

	return RTC0->COUNT.reg;
#endif
	rtc_sync();
	return RTC0->COUNT.reg;
}

static void rtc_reset(void)
{
	struct timespec tp;
	struct tm tm;

	clock_gettime(CLOCK_REALTIME, &tp);
	gmtime_r(&tp.tv_sec, &tm);

	uptime = k_uptime_get();

	rtc_boot_uptime = tp.tv_sec * 1000 + tp.tv_nsec / 1000000;
	rtc_sync();

	/* Disable interrupt. */
	RTC0->INTENCLR.reg = RTC_MODE0_INTENCLR_MASK;
	/* Clear interrupt flag. */
	RTC0->INTFLAG.reg = RTC_MODE0_INTFLAG_MASK;

	/* Disable RTC module. */
#ifdef RTC_MODE0_CTRL_ENABLE
	RTC0->CTRL.reg &= ~RTC_MODE0_CTRL_ENABLE;
#else
	RTC0->CTRLA.reg &= ~RTC_MODE0_CTRLA_ENABLE;
#endif

	rtc_sync();

	/* Initiate software reset. */
#ifdef RTC_MODE0_CTRL_SWRST
	RTC0->CTRL.bit.SWRST = 1;
	while (RTC0->CTRL.bit.SWRST) {
	}
#else
	RTC0->CTRLA.bit.SWRST = 1;
	while (RTC0->CTRLA.bit.SWRST) {
	}
#endif
}

static void rtc_isr(void *arg)
{
	ARG_UNUSED(arg);

	/* Read and clear the interrupt flag register. */
	uint16_t status = RTC0->INTFLAG.reg;

	RTC0->INTFLAG.reg = status;

#ifdef CONFIG_TICKLESS_KERNEL

	int key = irq_lock();

	/* Read the current counter and announce the elapsed time in ticks. */
	uint32_t count = rtc_count();

	uint32_t ticks = (count - rtc_last) / CYCLES_PER_TICK;

	rtc_last += ticks * CYCLES_PER_TICK;
	irq_unlock(key);
	z_clock_announce(ticks);

#else /* !CONFIG_TICKLESS_KERNEL */

	if (status) {
		/* RTC just ticked one more tick... */
		if (++rtc_counter == rtc_timeout) {
			z_clock_announce(rtc_counter - rtc_last);
			rtc_last = rtc_counter;
		}
	} else {
		/* ISR was invoked directly from z_clock_set_timeout. */
		z_clock_announce(0);
	}

#endif /* CONFIG_TICKLESS_KERNEL */
}

int z_clock_driver_init(struct device *device)
{
	ARG_UNUSED(device);

#ifdef MCLK
	MCLK->APBAMASK.reg |= MCLK_APBAMASK_RTC;
	OSC32KCTRL->RTCCTRL.reg = OSC32KCTRL_RTCCTRL_RTCSEL_ULP32K;
#else
	/* Set up bus clock and GCLK generator. */
	PM->APBAMASK.reg |= PM_APBAMASK_RTC;
	GCLK->CLKCTRL.reg =
	    GCLK_CLKCTRL_ID(DT_INST_CLOCKS_CELL_BY_NAME(0, gclk, clkctrl_id)) |
	    GCLK_CLKCTRL_CLKEN |
	    GCLK_CLKCTRL_GEN(ATMEL_SAM0_DT_INST_GCLK_REG_ADDR(0));

	/* Synchronize GCLK. */
	while (GCLK->STATUS.bit.SYNCBUSY) {
	}
#endif

	/* Reset module to hardware defaults. */
	rtc_reset();

	rtc_last = 0U;

	/* Configure RTC with 32-bit mode, configured prescaler and MATCHCLR. */
#ifdef RTC_MODE0_CTRL_MODE
	uint16_t ctrl = RTC_MODE0_CTRL_MODE(0)
		| RTC_MODE0_CTRL_PRESCALER(DT_INST_PROP(0, prescaler));
#else
	uint16_t ctrl = RTC_MODE0_CTRLA_MODE(0)
		| RTC_MODE0_CTRLA_PRESCALER(DT_INST_PROP(0, prescaler));
#endif

#ifndef CONFIG_TICKLESS_KERNEL
#ifdef RTC_MODE0_CTRL_MATCHCLR
	ctrl |= RTC_MODE0_CTRL_MATCHCLR;
#else
	ctrl |= RTC_MODE0_CTRLA_MATCHCLR;
#endif
#endif
	rtc_sync();
#ifdef RTC_MODE0_CTRL_MODE
	RTC0->CTRL.reg = ctrl;
#else
	RTC0->CTRLA.reg = ctrl;
#endif

#ifdef CONFIG_TICKLESS_KERNEL
	/* Tickless kernel lets RTC count continually and ignores overflows. */
	RTC0->INTENSET.reg = RTC_MODE0_INTENSET_CMP0;
#else
	/* Non-tickless mode uses comparator together with MATCHCLR. */
	rtc_sync();
	RTC0->COMP[0].reg = CYCLES_PER_TICK;
	RTC0->INTENSET.reg = RTC_MODE0_INTENSET_OVF;
	rtc_counter = 0U;
	rtc_timeout = 0U;
#endif

	/* Enable RTC module. */
	rtc_sync();
#ifdef RTC_MODE0_CTRL_ENABLE
	RTC0->CTRL.reg |= RTC_MODE0_CTRL_ENABLE;
#else
	RTC0->CTRLA.reg |= RTC_MODE0_CTRLA_ENABLE;
#endif

	/* Enable RTC interrupt. */
	NVIC_ClearPendingIRQ(DT_INST_IRQN(0));
	IRQ_CONNECT(DT_INST_IRQN(0),
		    DT_INST_IRQ(0, priority), rtc_isr, 0, 0);
	irq_enable(DT_INST_IRQN(0));

	return 0;
}

void z_clock_set_timeout(int32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#ifdef CONFIG_TICKLESS_KERNEL

	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;
	ticks = MAX(MIN(ticks - 1, (int32_t) MAX_TICKS), 0);

	int key = irq_lock();

	/* Compute number of RTC cycles until the next timeout. */
	uint32_t count = rtc_count();
	uint32_t timeout = ticks * CYCLES_PER_TICK + count % CYCLES_PER_TICK;

	/* Round to the nearest tick boundary. */
	timeout = (timeout + CYCLES_PER_TICK - 1) / CYCLES_PER_TICK
		  * CYCLES_PER_TICK;

	if (timeout < TICK_THRESHOLD) {
		timeout += CYCLES_PER_TICK;
	}

	rtc_sync();
	RTC0->COMP[0].reg = count + timeout;
	irq_unlock(key);

#else /* !CONFIG_TICKLESS_KERNEL */

	if (ticks == K_FOREVER) {
		/* Disable comparator for K_FOREVER and other negative
		 * values.
		 */
		rtc_timeout = rtc_counter;
		return;
	}

	if (ticks < 1) {
		ticks = 1;
	}

	/* Avoid race condition between reading counter and ISR incrementing
	 * it.
	 */
	int key = irq_lock();

	rtc_timeout = rtc_counter + ticks;
	irq_unlock(key);

#endif /* CONFIG_TICKLESS_KERNEL */
}

uint32_t z_clock_elapsed(void)
{
#ifdef CONFIG_TICKLESS_KERNEL
	int key;
	uint32_t ret;

	key = irq_lock();
	ret = (rtc_count() - rtc_last) / CYCLES_PER_TICK;
	irq_unlock(key);
	return ret;
#else
	return rtc_counter - rtc_last;
#endif
}

uint32_t z_timer_cycle_get_32(void)
{
	int key;
	uint32_t ret;

	/* Just return the absolute value of RTC cycle counter. */
	/*
	 * Note:  rtc_count() returns the number of rtc hw clock cycles
	 *        So, this value is converted to ticks and then the
	 *        ticks are converted to SYS HW cycles
	 */
	ret = k_ticks_to_cyc_near32(rtc_count() / CYCLES_PER_TICK);
	return ret;
}

/**
 * @brief Prepare RTC to wake from STANDBY
 *
 * The kernel does not like it when the RTC does not tick
 * after waking from sleep.  RCONT enables continuously syncing the
 * COUNT register but after sleep, we need to wait for a new read
 * request to complete.
 *
 * So, we issue a new READREQ to start the sync
 *
 * For samd5x, synchronization is started automatically when waking from sleep
 * see 13.3.3
 *
 */
void rtc_wake(void)
{
#ifdef RTC_READREQ_RREQ
	RTC0->READREQ.reg = RTC_READREQ_RREQ || RTC_READREQ_RCONT;
#endif
}

int64_t sam0_rtc_timer_boot_time(void) {
	return rtc_boot_uptime;
}

/**
 * @brief Timer idle exit notification
 *
 * The system is exiting the idle so wake up the RTC counter (SAMD21)
 *
 */
void z_clock_idle_exit(void)
{
	rtc_wake();
}

