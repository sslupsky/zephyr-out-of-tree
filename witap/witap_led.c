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
#define LED_THREAD_STACKSIZE 512

/* scheduling priority used by each thread */
#define LED_THREAD_PRIORITY 7

#define LED_ANIMATION_DWELL_TIME 1000
#define LED_MAX_ANIMATIONS 8
#define LED_RING_BUF_BYTES ((LED_MAX_ANIMATIONS * sizeof(struct led_animation_t)) + 1)
RING_BUF_DECLARE(led_ring_buf, LED_RING_BUF_BYTES);

LOG_MODULE_DECLARE(myApp);


struct k_sem animation_sync;
static struct k_mutex led_lock;
static bool led_initialized = false;

#if DT_HAS_ALIAS(led_red)
static struct device *red_led_dev;
#endif

#if DT_HAS_ALIAS(led_green)
static struct device *green_led_dev;
#endif

#if DT_HAS_ALIAS(led_blue)
static struct device *blue_led_dev;
#endif

#if DT_HAS_ALIAS(led_white)
static struct device *white_led_dev;
#endif

#if DT_HAS_ALIAS(led_mono)
static struct device *mono_led_dev;
#endif

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

void led_init()
{
	/* Set LED pin as output */
#if DT_HAS_ALIAS(led_red)
	red_led_dev = device_get_binding(LED_RED_DEVICE_NAME);
	if (!red_led_dev) {
		LOG_ERR("LED RED device driver not found");
		return;
	}
	gpio_pin_configure(red_led_dev, LED_RED, GPIO_OUTPUT);
#endif

#if DT_HAS_ALIAS(led_green)
	green_led_dev = device_get_binding(LED_GREEN_DEVICE_NAME);
	if (!green_led_dev) {
		LOG_ERR("LED Green device driver not found");
		return;
	}
	gpio_pin_configure(green_led_dev, LED_GREEN, GPIO_OUTPUT);
#endif

#if DT_HAS_ALIAS(led_blue)
	blue_led_dev = device_get_binding(LED_BLUE_DEVICE_NAME);
	if (!blue_led_dev) {
		LOG_ERR("LED Blue device driver not found");
		return;
	}
	gpio_pin_configure(blue_led_dev, LED_BLUE, GPIO_OUTPUT);
#endif

#if DT_HAS_ALIAS(led_white)
	white_led_dev = device_get_binding(LED_WHITE_DEVICE_NAME);
	if (!white_led_dev) {
		LOG_ERR("LED White device driver not found");
		return;
	}
	gpio_pin_configure(white_led_dev, LED_WHITE, GPIO_OUTPUT);
#endif

#if DT_HAS_ALIAS(led_mono)
	mono_led_dev = device_get_binding(LED_MONO_DEVICE_NAME);
	if (!mono_led_dev) {
		LOG_ERR("LED Mono device driver not found");
		return;
	}
	gpio_pin_configure(mono_led_dev, LED_MONO, GPIO_OUTPUT);
#endif

   	k_sem_init(&animation_sync, 0, LED_MAX_ANIMATIONS);
	led_initialized = true;
}

void led_set_color(const struct led_color_t *color)
{
#if DT_HAS_ALIAS(led_red)
		gpio_pin_set(red_led_dev, LED_RED, color->red);
#endif
#if DT_HAS_ALIAS(led_green)
		gpio_pin_set(green_led_dev, LED_GREEN, color->green);
#endif
#if DT_HAS_ALIAS(led_blue)
		gpio_pin_set(blue_led_dev, LED_BLUE, color->blue);
#endif
#if DT_HAS_ALIAS(led_white)
		gpio_pin_set(white_led_dev, LED_WHITE, color->white);
#endif
#if DT_HAS_ALIAS(led_mono)
		gpio_pin_set(mono_led_dev, LED_MONO, color->red |color->green
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
		k_sem_take(&animation_sync, K_FOREVER);
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

	k_mutex_lock(&led_lock, K_FOREVER);
	if (k_sem_count_get(&animation_sync) < LED_MAX_ANIMATIONS) {
		ret = ring_buf_put(&led_ring_buf, (u8_t *) animation, sizeof(struct led_animation_t));
		if ( ret != sizeof(struct led_animation_t) ) {
			LOG_ERR("LED ring buffer overflow");
			goto done;
		}
		k_sem_give(&animation_sync);
	}
done:
	k_mutex_unlock(&led_lock);
	return;
}

K_THREAD_DEFINE(led, LED_THREAD_STACKSIZE, led_process, NULL, NULL, NULL,
		LED_THREAD_PRIORITY, 0, 0);
