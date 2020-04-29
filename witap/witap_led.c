/**
 * @file led.c
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-01-11
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#include "witap_led.h"

/* size of stack area used by each thread */
#define LED_THREAD_STACKSIZE 256

/* scheduling priority used by each thread */
#define LED_THREAD_PRIORITY 7

#define LED_ANIMATION_DWELL_TIME 1000
#define LED_MAX_ANIMATIONS 8
#define LED_RING_BUF_BYTES ((LED_MAX_ANIMATIONS * sizeof(struct led_animation_t)) + 1)
RING_BUF_DECLARE(led_ring_buf, LED_RING_BUF_BYTES);

LOG_MODULE_DECLARE(myApp);
K_THREAD_STACK_DEFINE(led_stack, LED_THREAD_STACKSIZE);

struct led_data_t {
	struct k_thread led_thread;
	struct k_sem animation_sync;
	struct k_mutex led_lock;

#if DT_HAS_ALIAS(led_red)
	struct device *red_dev;
#endif

#if DT_HAS_ALIAS(led_green)
	struct device *green_dev;
#endif

#if DT_HAS_ALIAS(led_blue)
	struct device *blue_dev;
#endif

#if DT_HAS_ALIAS(led_white)
	struct device *white_dev;
#endif

#if DT_HAS_ALIAS(led_mono)
	struct device *mono_dev;
#endif
};

const struct led_color_t LED_COLOR_BLACK = {
	.red = 0,
	.green = 0,
	.blue = 0,
	.white = 0,
};

const struct led_color_t LED_COLOR_RED = {
	.red = 1,
	.green = 0,
	.blue = 0,
	.white = 1,
};

const struct led_color_t LED_COLOR_GREEN = {
	.red = 0,
	.green = 1,
	.blue = 0,
	.white = 1,
};

const struct led_color_t LED_COLOR_BLUE = {
	.red = 0,
	.green = 0,
	.blue = 1,
	.white = 1,
};

const struct led_color_t LED_COLOR_WHITE = {
	.red = 1,
	.green = 1,
	.blue = 1,
	.white = 1,
};

const struct led_animation_t animation_heartbeat = {
	.effect = LED_EFFECT_BLINK,
	.color = (struct led_color_t *)&LED_COLOR_RED,
	.duration = 1000,
	.duty_cycle = 100,
	.repeat = 0,
};

const struct led_animation_t animation_boot = {
	.effect = LED_EFFECT_BLINK,
	.color = (struct led_color_t *)&LED_COLOR_RED,
	.duration = 200,
	.duty_cycle = 500,
	.repeat = 4,
};

const struct led_animation_t animation_success = {
	.effect = LED_EFFECT_BLINK,
	.color = (struct led_color_t *)&LED_COLOR_GREEN,
	.duration = 1000,
	.duty_cycle = 100,
	.repeat = 0,
};

const struct led_animation_t animation_wait = {
	.effect = LED_EFFECT_BLINK,
	.color = (struct led_color_t *)&LED_COLOR_GREEN,
	.duration = 200,
	.duty_cycle = 500,
	.repeat = 2,
};

const struct led_animation_t animation_error = {
	.effect = LED_EFFECT_BLINK,
	.color = (struct led_color_t *)&LED_COLOR_RED,
	.duration = 1000,
	.duty_cycle = 200,
	.repeat = 2,
};

static bool led_initialized = false;
static struct led_data_t led_data;

void led_set_color(const struct led_color_t *color)
{
#if DT_HAS_ALIAS(led_red)
		gpio_pin_set(led_data.red_dev, LED_RED, color->red);
#endif
#if DT_HAS_ALIAS(led_green)
		gpio_pin_set(led_data.green_dev, LED_GREEN, color->green);
#endif
#if DT_HAS_ALIAS(led_blue)
		gpio_pin_set(led_data.blue_dev, LED_BLUE, color->blue);
#endif
#if DT_HAS_ALIAS(led_white)
		gpio_pin_set(led_data.white_dev, LED_WHITE, color->white);
#endif
#if DT_HAS_ALIAS(led_mono)
		gpio_pin_set(led_data.mono_dev, LED_MONO, color->red |color->green
			     | color->blue | color->white);
#endif
}

void led_animation_blink(const struct led_animation_t *animation)
{
	u16_t on = ((u32_t) animation->duration * animation->duty_cycle) / 10000U;
	u16_t off = animation->duration - on;
	u8_t num = MIN(animation->repeat + 1, 255);

	LOG_DBG("on: %d, off: %d, num: %d", on, off, num);
	for (; num > 0; num--) {
		led_set_color(animation->color);
		k_sleep(K_MSEC(on));
		led_set_color(&LED_COLOR_BLACK);
		k_sleep(K_MSEC(off));
	}
}

void led_process(void *p1, void *p2, void *p3)
{
	size_t ret;
	struct led_animation_t animation;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!led_initialized) {
		k_sleep(K_MSEC(100));
	}

	while (1) {
		k_sem_take(&led_data.animation_sync, K_FOREVER);
		ret = ring_buf_get(&led_ring_buf, (u8_t *) &animation, sizeof(animation));
		if (ret == sizeof(animation)) {
			switch ( animation.effect ) {
				case LED_EFFECT_BLINK: {
					led_animation_blink(&animation);
					break;
				}
				default: {
					LOG_ERR("Unsupported LED animation");
					break;
				}
			}
			k_sleep(K_MSEC(LED_ANIMATION_DWELL_TIME));
		} else {
			LOG_ERR("LED ring buffer underflow");
		}
	}
}

void led_add(const struct led_animation_t *animation) {
	u32_t ret;

	if ( !led_initialized ) {
		return;
	}

	k_mutex_lock(&led_data.led_lock, K_FOREVER);
	if (k_sem_count_get(&led_data.animation_sync) < LED_MAX_ANIMATIONS) {
		ret = ring_buf_put(&led_ring_buf, (u8_t *) animation, sizeof(struct led_animation_t));
		if ( ret != sizeof(struct led_animation_t) ) {
			LOG_ERR("LED ring buffer overflow");
			goto done;
		}
		k_sem_give(&led_data.animation_sync);
	}
done:
	k_mutex_unlock(&led_data.led_lock);
	return;
}

void led_init()
{
	/* Set LED pin as output */
#if DT_HAS_ALIAS(led_red)
	led_data.red_dev = device_get_binding(LED_RED_DEVICE_NAME);
	if (!led_data.red_dev) {
		LOG_ERR("LED RED device driver not found");
		return;
	}
	gpio_pin_configure(led_data.red_dev, LED_RED, GPIO_OUTPUT);
#endif

#if DT_HAS_ALIAS(led_green)
	led_data.green_dev = device_get_binding(LED_GREEN_DEVICE_NAME);
	if (!led_data.green_dev) {
		LOG_ERR("LED Green device driver not found");
		return;
	}
	gpio_pin_configure(led_data.green_dev, LED_GREEN, GPIO_OUTPUT);
#endif

#if DT_HAS_ALIAS(led_blue)
	led_data.blue_dev = device_get_binding(LED_BLUE_DEVICE_NAME);
	if (!led_data.blue_dev) {
		LOG_ERR("LED Blue device driver not found");
		return;
	}
	gpio_pin_configure(led_data.blue_dev, LED_BLUE, GPIO_OUTPUT);
#endif

#if DT_HAS_ALIAS(led_white)
	led_data.white_dev = device_get_binding(LED_WHITE_DEVICE_NAME);
	if (!led_data.white_dev) {
		LOG_ERR("LED White device driver not found");
		return;
	}
	gpio_pin_configure(led_data.white_dev, LED_WHITE, GPIO_OUTPUT);
#endif

#if DT_HAS_ALIAS(led_mono)
	led_data.mono_dev = device_get_binding(LED_MONO_DEVICE_NAME);
	if (!led_data.mono_dev) {
		LOG_ERR("LED Mono device driver not found");
		return;
	}
	gpio_pin_configure(led_data.mono_dev, LED_MONO, GPIO_OUTPUT);
#endif

	k_thread_create(&led_data.led_thread, led_stack,
			K_THREAD_STACK_SIZEOF(led_stack),
			(k_thread_entry_t)led_process,
			&led_data, NULL, NULL,
			LED_THREAD_PRIORITY,
			0, K_NO_WAIT);
	k_thread_name_set(&led_data.led_thread, "led process");

	k_sem_init(&led_data.animation_sync, 0, LED_MAX_ANIMATIONS);
	led_initialized = true;
}
