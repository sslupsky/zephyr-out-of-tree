/**
 * @file witap_power.hpp
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-20
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


#include "witap_types.h"
#include <usb/usb_device.h>
#include <drivers/pmic.h>

enum witap_pm_devices {
	WITAP_PM_I2C,
	WITAP_PM_SPI,
	WITAP_PM_LORA_UART,
	WITAP_PM_USB,
	WITAP_PM_BOOT,
	WITAP_PM_FORCE_SLEEP,
};

/* forward declaration of global scope variables */
extern volatile enum usb_dc_status_code usbState;
extern volatile bool application_boot;
extern volatile bool force_sleep;
extern struct k_delayed_work usb_disconnect_work;

/* Forward declaration of "C" functions */
#ifdef __cplusplus
extern "C" {
#endif
	void witap_pm_busy_set(witap_pm_devices dev);
	void witap_pm_busy_clear(witap_pm_devices dev);

#ifdef __cplusplus
}
#endif
