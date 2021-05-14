/**
 * @file led.h
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-01-11
 * 
 * @copyright Copyright (c) 2020
 * 
 */

#ifndef __WITAP_LED_H__
#define __WITAP_LED_H__

#include <stdio.h>

#include <zephyr.h>
#include <device.h>
#include <version.h>
#include <stdlib.h>
#include <soc.h>
#include <sys/ring_buffer.h>

#include <drivers/gpio.h>

#include <logging/log.h>

#ifdef __cplusplus
extern "C" {
#endif

#if DT_NODE_EXISTS(DT_ALIAS(led_red))
#define LED_RED_DEVICE_NAME		DT_PROP(DT_PHANDLE(DT_ALIAS(led_red), gpios), label)
#define LED_RED				DT_GPIO_PIN(DT_ALIAS(led_red), gpios)
#endif

#if DT_NODE_EXISTS(DT_ALIAS(led_green))
#define LED_GREEN_DEVICE_NAME		DT_PROP(DT_PHANDLE(DT_ALIAS(led_green), gpios), label)
#define LED_GREEN			DT_GPIO_PIN(DT_ALIAS(led_green), gpios)
#endif

#if DT_NODE_EXISTS(DT_ALIAS(led_blue))
#define LED_BLUE_DEVICE_NAME		DT_PROP(DT_PHANDLE(DT_ALIAS(led_blue), gpios), label)
#define LED_BLUE			DT_GPIO_PIN(DT_ALIAS(led_blue), gpios)
#endif

#if DT_NODE_EXISTS(DT_ALIAS(led_white))
#define LED_WHITE_DEVICE_NAME		DT_PROP(DT_PHANDLE(DT_ALIAS(led_white), gpios), label)
#define LED_WHITE			DT_GPIO_PIN(DT_ALIAS(led_white), gpios)
#endif

#if DT_NODE_EXISTS(DT_ALIAS(led_mono))
#define LED_MONO_DEVICE_NAME		DT_PROP(DT_PHANDLE(DT_ALIAS(led_mono), gpios), label)
#define LED_MONO			DT_GPIO_PIN(DT_ALIAS(led_mono), gpios)
#endif

enum LED_EFFECT_e {
	LED_EFFECT_OFF,
	LED_EFFECT_ON,
	LED_EFFECT_BLINK,
	LED_EFFECT_FADE_IN,
	LED_EFFECT_FADE_OUT,
	LED_EFFECT_BREATHE,
	LED_EFFECT_FLICKER,
};

struct led_color_t {
	u8_t red;
	u8_t green;
	u8_t blue;
	u8_t white;
} ;

struct led_animation_t {
	u8_t effect;
	struct led_color_t *color;
	u16_t duration;
	u16_t duty_cycle;
	u32_t repeat;
} __attribute__((packed)) ;

extern const struct led_animation_t animation_heartbeat;
extern const struct led_animation_t animation_heartbeat_error;
extern const struct led_animation_t animation_boot;
extern const struct led_animation_t animation_success;
extern const struct led_animation_t animation_wait;
extern const struct led_animation_t animation_error;
extern const struct led_animation_t animation_gpslock;
extern const struct led_animation_t animation_usb_connect;
extern const struct led_animation_t animation_usb_disconnect;

void led_animation_blink(const struct led_animation_t *animation);
void led_add(const struct led_animation_t *animation );
void led_init(void);

#ifdef __cplusplus
}
#endif


#endif /* __WITAP_LED_H__ */