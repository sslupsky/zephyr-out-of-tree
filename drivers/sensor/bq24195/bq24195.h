/**
 * @file bq24195.h
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-18
 * 
 * @copyright Copyright (c) 2020
 * SPDX-License-Identifier: Apache-2.0
 * 
 *    _____                 _                _        _          
 *   / ____|               (_)              | |      (_)         
 *  | (___   ___ __ _ _ __  _ _ __ ___   ___| |_ _ __ _  ___ ___ 
 *   \___ \ / __/ _` | '_ \| | '_ ` _ \ / _ \ __| '__| |/ __/ __|
 *   ____) | (_| (_| | | | | | | | | | |  __/ |_| |  | | (__\__ \
 *  |_____/ \___\__,_|_| |_|_|_| |_| |_|\___|\__|_|  |_|\___|___/
 *                                                               
 */

/*
 *  This is a library for the MKR PMIC BQ24195.
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef __PMIC_BQ24195_H__
#define __PMIC_BQ24195_H__

#include <sys/util.h>
#include <drivers/pmic.h>


#define BQ24195_STARTUP_TIME_USEC     1000

/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define BQ24195_ADDRESS		(0x6B)		/**< The default I2C address for the pmic. */
#define BQ24195_CHIPID		(0x23)		/**< Default chip ID. */
#define BQ24195_CHIPID_MASK	0b11111011

#define DEFAULT_INPUT_VOLTAGE_LIMIT	4360
#define DEFAULT_INPUT_CURRENT_LIMIT	2000
#define DEFAULT_CHARGE_CURRENT		512
#define DEFAULT_TERMINATION_VOLTAGE	4112

#define BQ24195_SIGNAL_UPDATE_IMMEDIATE			0
#define BQ24195_SIGNAL_UPDATE_DELAYED			1
#define BQ24195_SIGNAL_DISCONNECT_CHECK			2

/**
 * Driver for the BQ24195 pmic.
 */

/**
 * @brief Registers available on the pmic.
 * 
 */

// public:
enum bq24195_register_address {
	BQ24195_REGISTER_INPUT_SOURCE_CONTROL       = 0x00,
	BQ24195_REGISTER_POWERON_CONFIG             = 0x01,
	BQ24195_REGISTER_CHARGE_CURRENT_CONTROL     = 0x02,
	BQ24195_REGISTER_PRECHARGE_CURRENT_CONTROL  = 0x03,
	BQ24195_REGISTER_CHARGE_VOLTAGE_CONTROL     = 0x04,
	BQ24195_REGISTER_CHARGE_TERMINATION_CONTROL = 0x05,
	BQ24195_REGISTER_THERMAL_REGULATION_CONTROL = 0x06,
	BQ24195_REGISTER_MISC_OPERATION_CONTROL     = 0x07,
	BQ24195_REGISTER_SYSTEM_STATUS              = 0x08,
	BQ24195_REGISTER_FAULT                      = 0x09,
	BQ24195_REGISTER_PMIC_VENDOR                = 0x0A
};

/**
 * @brief Struct's to hold register bit fields.
 * 
 */ 

typedef union {
	struct {
		u8_t IINLIM         : 3;
		u8_t VINDPM         : 4;
		u8_t EN_HIZ         : 1;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_INPUT_SOURCE_CONTROL_t;

typedef union {
	struct {
		u8_t reserved       : 1;
		u8_t SYS_MIN        : 3;
		u8_t CHG_CONFIG     : 2;
		u8_t I2C_WDT_RESET  : 1;
		u8_t REG_RESET      : 1;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_POWERON_CONFIG_t;

typedef union {
	struct {
		u8_t FORCE_20PCT    : 1;
		u8_t reserved       : 1;
		u8_t ICHG           : 6;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_CHARGE_CURRENT_CONTROL_t;

typedef union {
	struct {
		u8_t ITERM          : 4;
		u8_t IPRECHG        : 4;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_PRECHARGE_CURRENT_CONTROL_t;

typedef union {
	struct {
		u8_t VRECHG         : 1;
		u8_t BATLOWV        : 1;
		u8_t VREG           : 6;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_CHARGE_VOLTAGE_CONTROL_t;

typedef union {
	struct {
		u8_t reserved       : 1;
		u8_t CHG_TIMER      : 2;
		u8_t EN_TIMER       : 1;
		u8_t WATCHDOG       : 2;
		u8_t TERM_STAT      : 1;
		u8_t EN_TERM        : 1;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_CHARGE_TERMINATION_CONTROL_t;

typedef union {
	struct {
		u8_t TREG           : 2;
		u8_t reserved       : 6;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_THERMAL_REGULATION_CONTROL_t;

typedef union {
	struct {
		u8_t INT_MASK       : 2;
		u8_t reserved       : 3;
		u8_t BATFET_Disable : 1;
		u8_t TMR2X_EN       : 1;
		u8_t DPDM_EN        : 1;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_MISC_OPERATION_CONTROL_t;

typedef union {
	struct {
		u8_t VSYS_STAT      : 1;
		u8_t THERM_STAT     : 1;
		u8_t PG_STAT        : 1;
		u8_t DPM_STAT       : 1;
		u8_t CHRG_STAT      : 2;
		u8_t VBUS_STAT      : 2;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_SYSTEM_STATUS_t;

typedef union {
	struct {
		u8_t NTC_FAULT      : 3;
		u8_t BAT_FAULT      : 1;
		u8_t CHRG_FAULT     : 2;
		u8_t reserved       : 1;
		u8_t WATCHDOG_FAULT : 1;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_FAULT_t;

typedef struct {
	BQ24195_FAULT_t latched_fault;
	BQ24195_FAULT_t fault;
} __attribute__((packed)) BQ24195_FAULT_DATA_t;

typedef union {
	struct {
		u8_t DEV_REG        : 2;
		u8_t TS_PROFILE     : 1;
		u8_t PN             : 3;
		u8_t reserved       : 2;
	} bit;
	u8_t reg;
} __attribute__((packed)) BQ24195_VENDOR_STATUS_t;

/*!
 *  Struct to hold registers.
 */
typedef struct {
	BQ24195_INPUT_SOURCE_CONTROL_t InputSourceControl;  /**< input source control register. */
	BQ24195_POWERON_CONFIG_t PowerOnConfig;  /**< poweron config register. */
	BQ24195_CHARGE_CURRENT_CONTROL_t ChargeCurrentControl;  /**< charge current control register. */
	BQ24195_PRECHARGE_CURRENT_CONTROL_t PrechargeCurrentControl;  /**< precharge current control register. */
	BQ24195_CHARGE_VOLTAGE_CONTROL_t ChargeVoltageControl;  /**< charge voltage control register. */
	BQ24195_CHARGE_TERMINATION_CONTROL_t ChargeTerminationControl;  /**< charge timer control register. */
	BQ24195_THERMAL_REGULATION_CONTROL_t ThermalRegulationControl;  /**< thermal regulation control register. */
	BQ24195_MISC_OPERATION_CONTROL_t MiscOperationControl;  /**< misc operation control register. */
	BQ24195_SYSTEM_STATUS_t SystemStatus;  /**< system status register. */
	BQ24195_FAULT_t Latched_Fault;	/**< Latched (past) fault register. */
	BQ24195_VENDOR_STATUS_t VendorStatus;  /**< vender status register. */
	BQ24195_FAULT_t Fault;		/**< Current fault register. */
} BQ24195_device_t;

typedef union {
	BQ24195_INPUT_SOURCE_CONTROL_t InputSourceControl;  /**< input source control register. */
	BQ24195_POWERON_CONFIG_t PowerOnConfig;  /**< poweron config register. */
	BQ24195_CHARGE_CURRENT_CONTROL_t ChargeCurrentControl;  /**< charge current control register. */
	BQ24195_PRECHARGE_CURRENT_CONTROL_t PrechargeCurrentControl;  /**< precharge current control register. */
	BQ24195_CHARGE_VOLTAGE_CONTROL_t ChargeVoltageControl;  /**< charge voltage control register. */
	BQ24195_CHARGE_TERMINATION_CONTROL_t ChargeTerminationControl;  /**< charge timer control register. */
	BQ24195_THERMAL_REGULATION_CONTROL_t ThermalRegulationControl;  /**< thermal regulation control register. */
	BQ24195_MISC_OPERATION_CONTROL_t MiscOperationControl;  /**< misc operation control register. */
	BQ24195_SYSTEM_STATUS_t SystemStatus;  /**< system status register. */
	BQ24195_FAULT_t Fault;  /**< fault register. */
	BQ24195_VENDOR_STATUS_t VendorStatus;  /**< vender status register. */
	u8_t reg;
} BQ24195_register_t;

#define BQ24195_INPUT_VOLTAGE_LIMIT_MIN		3880
#define BQ24195_INPUT_VOLTAGE_LIMIT_MAX		5080
#define BQ24195_INPUT_VOLTAGE_LIMIT_STEP	80
#define BQ24195_INPUT_VOLTAGE_LIMIT_BITS	4

/** input voltage limit for the pmic. */
enum bq24195_input_voltage_limit {
	BQ24195_INPUT_VOLTAGE_LIMIT_3880 = 0,
	BQ24195_INPUT_VOLTAGE_LIMIT_3960,
	BQ24195_INPUT_VOLTAGE_LIMIT_4040,
	BQ24195_INPUT_VOLTAGE_LIMIT_4120,
	BQ24195_INPUT_VOLTAGE_LIMIT_4200,
	BQ24195_INPUT_VOLTAGE_LIMIT_4280,
	BQ24195_INPUT_VOLTAGE_LIMIT_4360,
	BQ24195_INPUT_VOLTAGE_LIMIT_4440,
	BQ24195_INPUT_VOLTAGE_LIMIT_4520,
	BQ24195_INPUT_VOLTAGE_LIMIT_4600,
	BQ24195_INPUT_VOLTAGE_LIMIT_4680,
	BQ24195_INPUT_VOLTAGE_LIMIT_4760,
	BQ24195_INPUT_VOLTAGE_LIMIT_4840,
	BQ24195_INPUT_VOLTAGE_LIMIT_4920,
	BQ24195_INPUT_VOLTAGE_LIMIT_5000,
	BQ24195_INPUT_VOLTAGE_LIMIT_5080,
};

/** input current limit for the pmic. */
enum bq24195_input_current_limit {
	/** 100 mA. */
	BQ24195_CURRENT_LIMIT_100 = 0x00,
	/** 150 mA. */
	BQ24195_CURRENT_LIMIT_150 = 0x01,
	/** 500 mA. */
	BQ24195_CURRENT_LIMIT_500 = 0x02,
	/** 900 mA. */
	BQ24195_CURRENT_LIMIT_900 = 0x03,
	/** 1200 mA. */
	BQ24195_CURRENT_LIMIT_1200 = 0x04,
	/** 1500 mA. */
	BQ24195_CURRENT_LIMIT_1500 = 0x05,
	/** 2000 mA. */
	BQ24195_CURRENT_LIMIT_2000 = 0x06,
	/** 3000 mA. */
	BQ24195_CURRENT_LIMIT_3000 = 0x07,
};

/** Filtering level for pmic data. */
enum bq24195_charger_config {
	/** Charge disable */
	BQ24195_CHARGE_DISABLE = 0x00,
	/** Charge Battery */
	BQ24195_CHARGE_BATTERY = 0x01,
	/** OTG. */
	BQ24195_CHARGE_OTG = 0x02,
	/** OTG. */
	BQ24195_CHARGE_OTG2 = 0x03,
};

/** Standby duration in ms */
enum bq24195_standby_duration {
	/** 1 ms standby. */
	BQ24195_STANDBY_MS_1 = 0x00,
	/** 63 ms standby. */
	BQ24195_STANDBY_MS_63 = 0x01,
	/** 125 ms standby. */
	BQ24195_STANDBY_MS_125 = 0x02,
	/** 250 ms standby. */
	BQ24195_STANDBY_MS_250 = 0x03,
	/** 500 ms standby. */
	BQ24195_STANDBY_MS_500 = 0x04,
	/** 1000 ms standby. */
	BQ24195_STANDBY_MS_1000 = 0x05,
	/** 2000 ms standby. */
	BQ24195_STANDBY_MS_2000 = 0x06,
	/** 4000 ms standby. */
	BQ24195_STANDBY_MS_4000 = 0x07
};

#define BQ24195_CHARGE_VOLTAGE_MIN	3504
#define BQ24195_CHARGE_VOLTAGE_MAX	4400
#define BQ24195_CHARGE_VOLTAGE_STEP	16
#define BQ24195_CHARGE_VOLTAGE_BITS	4

enum bq24195_charge_voltage {
	BQ24195_CHARGE_VOLTAGE_3504 = 0,
	BQ24195_CHARGE_VOLTAGE_3520,
	BQ24195_CHARGE_VOLTAGE_3536,
	BQ24195_CHARGE_VOLTAGE_3552,
	BQ24195_CHARGE_VOLTAGE_3568,
	BQ24195_CHARGE_VOLTAGE_3584,
	BQ24195_CHARGE_VOLTAGE_3600,
	BQ24195_CHARGE_VOLTAGE_3616,
	BQ24195_CHARGE_VOLTAGE_3632,
	BQ24195_CHARGE_VOLTAGE_3648,
	BQ24195_CHARGE_VOLTAGE_3664,
	BQ24195_CHARGE_VOLTAGE_3680,
	BQ24195_CHARGE_VOLTAGE_3696,
	BQ24195_CHARGE_VOLTAGE_3712,
	BQ24195_CHARGE_VOLTAGE_3728,
	BQ24195_CHARGE_VOLTAGE_3744,
	BQ24195_CHARGE_VOLTAGE_3760,
	BQ24195_CHARGE_VOLTAGE_3776,
	BQ24195_CHARGE_VOLTAGE_3792,
	BQ24195_CHARGE_VOLTAGE_3808,
	BQ24195_CHARGE_VOLTAGE_3824,
	BQ24195_CHARGE_VOLTAGE_3840,
	BQ24195_CHARGE_VOLTAGE_3856,
	BQ24195_CHARGE_VOLTAGE_3872,
	BQ24195_CHARGE_VOLTAGE_3888,
	BQ24195_CHARGE_VOLTAGE_3904,
	BQ24195_CHARGE_VOLTAGE_3920,
	BQ24195_CHARGE_VOLTAGE_3936,
	BQ24195_CHARGE_VOLTAGE_3952,
	BQ24195_CHARGE_VOLTAGE_3968,
	BQ24195_CHARGE_VOLTAGE_3984,
	BQ24195_CHARGE_VOLTAGE_4000,
	BQ24195_CHARGE_VOLTAGE_4016,
	BQ24195_CHARGE_VOLTAGE_4032,
	BQ24195_CHARGE_VOLTAGE_4048,
	BQ24195_CHARGE_VOLTAGE_4064,
	BQ24195_CHARGE_VOLTAGE_4080,
	BQ24195_CHARGE_VOLTAGE_4096,
	BQ24195_CHARGE_VOLTAGE_4112,
	BQ24195_CHARGE_VOLTAGE_4128,
	BQ24195_CHARGE_VOLTAGE_4144,
	BQ24195_CHARGE_VOLTAGE_4160,
	BQ24195_CHARGE_VOLTAGE_4176,
	BQ24195_CHARGE_VOLTAGE_4192,
	BQ24195_CHARGE_VOLTAGE_4208,
	BQ24195_CHARGE_VOLTAGE_4224,
	BQ24195_CHARGE_VOLTAGE_4240,
	BQ24195_CHARGE_VOLTAGE_4256,
	BQ24195_CHARGE_VOLTAGE_4272,
	BQ24195_CHARGE_VOLTAGE_4288,
	BQ24195_CHARGE_VOLTAGE_4304,
	BQ24195_CHARGE_VOLTAGE_4320,
	BQ24195_CHARGE_VOLTAGE_4336,
	BQ24195_CHARGE_VOLTAGE_4352,
	BQ24195_CHARGE_VOLTAGE_4368,
	BQ24195_CHARGE_VOLTAGE_4384,
	BQ24195_CHARGE_VOLTAGE_4400,

	/**
	 * Number of all common sensor attributes.
	 */
	CHARGE_VOLTAGE_COMMON_COUNT,

	/**
	 * This and higher values are sensor specific.
	 * Refer to the sensor header file.
	 */
	CHARGE_VOLTAGE_PRIV_START = CHARGE_VOLTAGE_COMMON_COUNT,

	/**
	 * Maximum value describing a sensor attribute type.
	 */
	CHARGE_VOLTAGE_MAX = INT16_MAX,
};

#define BQ24195_CHARGE_CURRENT_MIN	512
#define BQ24195_CHARGE_CURRENT_MAX	4544
#define BQ24195_CHARGE_CURRENT_STEP	64
#define BQ24195_CHARGE_CURRENT_BITS	6

enum bq24195_charge_current {
	BQ24195_CHARGE_CURRENT_512 = 0,
	BQ24195_CHARGE_CURRENT_576,
	BQ24195_CHARGE_CURRENT_640,
	BQ24195_CHARGE_CURRENT_704,
	BQ24195_CHARGE_CURRENT_768,
	BQ24195_CHARGE_CURRENT_832,
	BQ24195_CHARGE_CURRENT_896,
	BQ24195_CHARGE_CURRENT_960,
	BQ24195_CHARGE_CURRENT_1024,
	BQ24195_CHARGE_CURRENT_1088,
	BQ24195_CHARGE_CURRENT_1152,
	BQ24195_CHARGE_CURRENT_1216,
	BQ24195_CHARGE_CURRENT_1280,
	BQ24195_CHARGE_CURRENT_1344,
	BQ24195_CHARGE_CURRENT_1408,
	BQ24195_CHARGE_CURRENT_1472,
	BQ24195_CHARGE_CURRENT_1536,
	BQ24195_CHARGE_CURRENT_1600,
	BQ24195_CHARGE_CURRENT_1664,
	BQ24195_CHARGE_CURRENT_1728,
	BQ24195_CHARGE_CURRENT_1792,
	BQ24195_CHARGE_CURRENT_1856,
	BQ24195_CHARGE_CURRENT_1920,
	BQ24195_CHARGE_CURRENT_1984,
	BQ24195_CHARGE_CURRENT_2048,
	BQ24195_CHARGE_CURRENT_2112,
	BQ24195_CHARGE_CURRENT_2176,
	BQ24195_CHARGE_CURRENT_2240,
	BQ24195_CHARGE_CURRENT_2304,
	BQ24195_CHARGE_CURRENT_2368,
	BQ24195_CHARGE_CURRENT_2432,
	BQ24195_CHARGE_CURRENT_2496,
	BQ24195_CHARGE_CURRENT_2560,
	BQ24195_CHARGE_CURRENT_2624,
	BQ24195_CHARGE_CURRENT_2688,
	BQ24195_CHARGE_CURRENT_2752,
	BQ24195_CHARGE_CURRENT_2816,
	BQ24195_CHARGE_CURRENT_2880,
	BQ24195_CHARGE_CURRENT_2944,
	BQ24195_CHARGE_CURRENT_3008,
	BQ24195_CHARGE_CURRENT_3072,
	BQ24195_CHARGE_CURRENT_3136,
	BQ24195_CHARGE_CURRENT_3200,
	BQ24195_CHARGE_CURRENT_3264,
	BQ24195_CHARGE_CURRENT_3328,
	BQ24195_CHARGE_CURRENT_3392,
	BQ24195_CHARGE_CURRENT_3456,
	BQ24195_CHARGE_CURRENT_3520,
	BQ24195_CHARGE_CURRENT_3584,
	BQ24195_CHARGE_CURRENT_3648,
	BQ24195_CHARGE_CURRENT_3712,
	BQ24195_CHARGE_CURRENT_3776,
	BQ24195_CHARGE_CURRENT_3840,
	BQ24195_CHARGE_CURRENT_3904,
	BQ24195_CHARGE_CURRENT_3968,
	BQ24195_CHARGE_CURRENT_4032,
	BQ24195_CHARGE_CURRENT_4096,
	BQ24195_CHARGE_CURRENT_4160,
	BQ24195_CHARGE_CURRENT_4224,
	BQ24195_CHARGE_CURRENT_4288,
	BQ24195_CHARGE_CURRENT_4352,
	BQ24195_CHARGE_CURRENT_4416,
	BQ24195_CHARGE_CURRENT_4480,
	BQ24195_CHARGE_CURRENT_4544,
};

// int begin(u8_t addr = BQ24195_ADDRESS, u8_t chipid = BQ24195_CHIPID);
// int end(u8_t addr = BQ24195_ADDRESS);

// int disableWatchdog(struct device *dev);
// int setInputVoltageLimit(struct device *dev, u8_t voltage);
// int setInputCurrentLimit(struct device *dev, u8_t current);
// int setChargeCurrent(struct device *dev, u8_t current);
// int setChargeVoltage(struct device *dev, u8_t voltage);
// int disableCharge(struct device *dev);
// int  apply_pmic_newdefaults(struct device *dev);
// int  configure_pmic(struct device *dev);
// bool is_battery_present(struct device *dev);
// int setHiZ(struct device *dev, bool enable = true);
// int disablePMIC(struct device *dev);
// int resetWatchdog(struct device *dev);
// int setChargeMode(struct device *dev, u8_t mode);
// int readPMICRegisters(struct device *dev);
// int setBATFET(struct device *dev, bool disable);
// bool isPowerGood(struct device *dev);
// bool getSystemStatus(struct device *dev);

// private:
/** Encapsulates the input source control register */
struct bq24195_input_source_control {
	/** input current limit */
	u8_t iinlim : 3;
	/** vin dynamic power management */
	u8_t vindpm : 4;
	/** enable HiZ */
	u8_t en_hiz : 1;
};

/** Encapsulates the poweron config register */
struct bq24195_poweron_config {
	/** register reset */
	u8_t register_reset : 1;
	/** i2c watchdog timer reset */
	u8_t watchdog_reset : 1;
	/** charger configuration */
	u8_t charger_config : 2;
	/** minimum system voltage limit */
	u8_t min_sys_voltage : 3;
	/** reserved - must write "1" */
	u8_t reserved : 1;
};

enum bq24195_watchdog_timeout {
	PMIC_WATCHDOG_disabled,
	PMIC_WATCHDOG_TIMEOUT_40,
	PMIC_WATCHDOG_TIMEOUT_80,
	PMIC_WATCHDOG_TIMEOUT_160,
};

enum bq24195_charge_status {
	BQ24195_CHARGE_STATUS_NOT_CHARGING = 0,
	BQ24195_CHARGE_STATUS_PRECHARGE,
	BQ24195_CHARGE_STATUS_FAST_CHARGE,
	BQ24195_CHARGE_STATUS_CHARGE_DONE,
};

enum bq24195_vbus_status {
	BQ24195_VBUS_STATUS_UNKNOWN = 0,
	BQ24195_VBUS_STATUS_USB_HOST,
	BQ24195_VBUS_STATUS_ADAPTER,
	BQ24195_VBUS_STATUS_USB_OTG,
};

enum bq24195_fault_ntc {
	BQ24195_FAULT_NTC_NORMAL = 0,
	BQ24195_FAULT_NTC_COLD = 5,
	BQ24195_FAULT_NTC_HOT = 6,
};

enum bq24195_fault_charge {
	BQ24195_FAULT_CHARGE_NORMAL = 0,
	BQ24195_FAULT_CHARGE_INPUT,
	BQ24195_FAULT_CHARGE_THERMAL,
	BQ24195_FAULT_CHARGE_SAFETY_TIMER,
};

/**
 * @typedef bq24195_reg_read_t
 * @brief Callback API upon reading a sensor's register
 *
 * See bq24195_reg_read() for argument description
 */
typedef 
int (*bq24195_reg_read_t)(struct device *dev, u8_t reg, u8_t *val, int size);

/**
 * @typedef bq24195_reg_write_t
 * @brief Callback API upon writing a sensor's register
 *
 * See bq24195_reg_write() for argument description
 */
typedef
int (*bq24195_reg_write_t)(struct device *dev, u8_t reg, u8_t val);


struct bq24195_driver_api {
	bq24195_reg_read_t reg_read;
	bq24195_reg_write_t reg_write;
};

/* driver structs */
struct bq24195_data {
	struct device *i2c;
	enum pmic_device_state device_state;
	enum pmic_battery_state battery_state;
	enum pmic_power_state power_state;
	void (*battery_state_change_cb)(u8_t state);
	void (*power_state_change_cb)(u8_t state);
	void (*device_state_change_cb)(u8_t state);
	s64_t charging_disabled_timestamp;
	struct k_poll_event events[1];
	struct k_poll_signal update_signal;
	struct pmic_power_config power_config;
	u8_t battery_disconnected;
};

struct bq24195_dev_config {
	u16_t i2c_addr;
	struct bq24195_driver_api api;
	struct pmic_power_config default_power_config;
};

#endif /* __PMIC_BQ24195_H__ */
