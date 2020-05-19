/**
 * @file Untitled-1
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-05-18
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

/*
 *  This is a library for the MKR PMIC BQ24195.
 *
 *  BSD license, all text above must be included in any redistribution
 */

#ifndef __PMIC_BQ24195_H__
#define __PMIC_BQ24195_H__

#include <sys/util.h>


#define BQ24195_STARTUP_TIME_USEC     1000

/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define BQ24195_ADDRESS		(0x6B)		/**< The default I2C address for the pmic. */
#define BQ24195_CHIPID		(0x23)		/**< Default chip ID. */
#define BQ24195_CHIPID_MASK	0b11111011


/**
 * Driver for the BQ24195 pmic.
 */

/**
 * @brief Registers available on the pmic.
 * 
 */

// public:
enum {
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
} BQ24195_Reg_Addr_e;

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
	BQ24195_FAULT_t Fault;  /**< fault register. */
	BQ24195_VENDOR_STATUS_t VendorStatus;  /**< vender status register. */
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

typedef struct {
	union {
		BQ24195_device_t reg;
		u8_t raw[sizeof(BQ24195_device_t)];
	};
	BQ24195_FAULT_t CurrentFault;
} pmic_t;


/** input voltage limit for the pmic. */
enum input_voltage {
	INPUT_VOLTAGE_LIMIT_3880 = 0,
	INPUT_VOLTAGE_LIMIT_3960,
	INPUT_VOLTAGE_LIMIT_4040,
	INPUT_VOLTAGE_LIMIT_4120,
	INPUT_VOLTAGE_LIMIT_4200,
	INPUT_VOLTAGE_LIMIT_4280,
	INPUT_VOLTAGE_LIMIT_4360,
	INPUT_VOLTAGE_LIMIT_4440,
	INPUT_VOLTAGE_LIMIT_4520,
	INPUT_VOLTAGE_LIMIT_4600,
	INPUT_VOLTAGE_LIMIT_4680,
	INPUT_VOLTAGE_LIMIT_4760,
	INPUT_VOLTAGE_LIMIT_4840,
	INPUT_VOLTAGE_LIMIT_4920,
	INPUT_VOLTAGE_LIMIT_5000,
	INPUT_VOLTAGE_LIMIT_5080,
};

/** input current limit for the pmic. */
enum input_current {
	/** 100 mA. */
	CURRENT_LIMIT_100 = 0x00,
	/** 150 mA. */
	CURRENT_LIMIT_150 = 0x01,
	/** 500 mA. */
	CURRENT_LIMIT_500 = 0x02,
	/** 900 mA. */
	CURRENT_LIMIT_900 = 0x03,
	/** 1200 mA. */
	CURRENT_LIMIT_1200 = 0x04,
	/** 1500 mA. */
	CURRENT_LIMIT_1500 = 0x05,
	/** 2000 mA. */
	CURRENT_LIMIT_2000 = 0x06,
	/** 3000 mA. */
	CURRENT_LIMIT_3000 = 0x07,
};

/** Filtering level for pmic data. */
enum charger_config {
	/** Charge disable */
	CHARGE_DISABLE = 0x00,
	/** Charge Battery */
	CHARGE_BATTERY = 0x01,
	/** OTG. */
	CHARGE_OTG = 0x02,
	/** OTG. */
	CHARGE_OTG2 = 0x03,
};

/** Standby duration in ms */
enum standby_duration {
	/** 1 ms standby. */
	STANDBY_MS_1 = 0x00,
	/** 63 ms standby. */
	STANDBY_MS_63 = 0x01,
	/** 125 ms standby. */
	STANDBY_MS_125 = 0x02,
	/** 250 ms standby. */
	STANDBY_MS_250 = 0x03,
	/** 500 ms standby. */
	STANDBY_MS_500 = 0x04,
	/** 1000 ms standby. */
	STANDBY_MS_1000 = 0x05,
	/** 2000 ms standby. */
	STANDBY_MS_2000 = 0x06,
	/** 4000 ms standby. */
	STANDBY_MS_4000 = 0x07
};

enum charge_voltage {
	CHARGE_VOLTAGE_3504 = 0,
	CHARGE_VOLTAGE_3520,
	CHARGE_VOLTAGE_3536,
	CHARGE_VOLTAGE_3552,
	CHARGE_VOLTAGE_3568,
	CHARGE_VOLTAGE_3584,
	CHARGE_VOLTAGE_3600,
	CHARGE_VOLTAGE_3616,
	CHARGE_VOLTAGE_3632,
	CHARGE_VOLTAGE_3648,
	CHARGE_VOLTAGE_3664,
	CHARGE_VOLTAGE_3680,
	CHARGE_VOLTAGE_3696,
	CHARGE_VOLTAGE_3712,
	CHARGE_VOLTAGE_3728,
	CHARGE_VOLTAGE_3744,
	CHARGE_VOLTAGE_3760,
	CHARGE_VOLTAGE_3776,
	CHARGE_VOLTAGE_3792,
	CHARGE_VOLTAGE_3808,
	CHARGE_VOLTAGE_3824,
	CHARGE_VOLTAGE_3840,
	CHARGE_VOLTAGE_3856,
	CHARGE_VOLTAGE_3872,
	CHARGE_VOLTAGE_3888,
	CHARGE_VOLTAGE_3904,
	CHARGE_VOLTAGE_3920,
	CHARGE_VOLTAGE_3936,
	CHARGE_VOLTAGE_3952,
	CHARGE_VOLTAGE_3968,
	CHARGE_VOLTAGE_3984,
	CHARGE_VOLTAGE_4000,
	CHARGE_VOLTAGE_4016,
	CHARGE_VOLTAGE_4032,
	CHARGE_VOLTAGE_4048,
	CHARGE_VOLTAGE_4064,
	CHARGE_VOLTAGE_4080,
	CHARGE_VOLTAGE_4096,
	CHARGE_VOLTAGE_4112,
	CHARGE_VOLTAGE_4128,
	CHARGE_VOLTAGE_4144,
	CHARGE_VOLTAGE_4160,
	CHARGE_VOLTAGE_4176,
	CHARGE_VOLTAGE_4192,
	CHARGE_VOLTAGE_4208,
	CHARGE_VOLTAGE_4224,
	CHARGE_VOLTAGE_4240,
	CHARGE_VOLTAGE_4256,
	CHARGE_VOLTAGE_4272,
	CHARGE_VOLTAGE_4288,
	CHARGE_VOLTAGE_4304,
	CHARGE_VOLTAGE_4320,
	CHARGE_VOLTAGE_4336,
	CHARGE_VOLTAGE_4352,
	CHARGE_VOLTAGE_4368,
	CHARGE_VOLTAGE_4384,
	CHARGE_VOLTAGE_4400,
};

enum charge_current {
	CHARGE_CURRENT_512 = 0,
	CHARGE_CURRENT_576,
	CHARGE_CURRENT_640,
	CHARGE_CURRENT_704,
	CHARGE_CURRENT_768,
	CHARGE_CURRENT_832,
	CHARGE_CURRENT_896,
	CHARGE_CURRENT_960,
	CHARGE_CURRENT_1024,
	CHARGE_CURRENT_1088,
	CHARGE_CURRENT_1152,
	CHARGE_CURRENT_1216,
	CHARGE_CURRENT_1280,
	CHARGE_CURRENT_1344,
	CHARGE_CURRENT_1408,
	CHARGE_CURRENT_1472,
	CHARGE_CURRENT_1536,
	CHARGE_CURRENT_1600,
	CHARGE_CURRENT_1664,
	CHARGE_CURRENT_1728,
	CHARGE_CURRENT_1792,
	CHARGE_CURRENT_1856,
	CHARGE_CURRENT_1920,
	CHARGE_CURRENT_1984,
	CHARGE_CURRENT_2048,
	CHARGE_CURRENT_2112,
	CHARGE_CURRENT_2176,
	CHARGE_CURRENT_2240,
	CHARGE_CURRENT_2304,
	CHARGE_CURRENT_2368,
	CHARGE_CURRENT_2432,
	CHARGE_CURRENT_2496,
	CHARGE_CURRENT_2560,
	CHARGE_CURRENT_2624,
	CHARGE_CURRENT_2688,
	CHARGE_CURRENT_2752,
	CHARGE_CURRENT_2816,
	CHARGE_CURRENT_2880,
	CHARGE_CURRENT_2944,
	CHARGE_CURRENT_3008,
	CHARGE_CURRENT_3072,
	CHARGE_CURRENT_3136,
	CHARGE_CURRENT_3200,
	CHARGE_CURRENT_3264,
	CHARGE_CURRENT_3328,
	CHARGE_CURRENT_3392,
	CHARGE_CURRENT_3456,
	CHARGE_CURRENT_3520,
	CHARGE_CURRENT_3584,
	CHARGE_CURRENT_3648,
	CHARGE_CURRENT_3712,
	CHARGE_CURRENT_3776,
	CHARGE_CURRENT_3840,
	CHARGE_CURRENT_3904,
	CHARGE_CURRENT_3968,
	CHARGE_CURRENT_4032,
	CHARGE_CURRENT_4096,
	CHARGE_CURRENT_4160,
	CHARGE_CURRENT_4224,
	CHARGE_CURRENT_4288,
	CHARGE_CURRENT_4352,
	CHARGE_CURRENT_4416,
	CHARGE_CURRENT_4480,
	CHARGE_CURRENT_4544,
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
struct input_source {
	/** input current limit */
	unsigned int iinlim : 3;
	/** vin dynamic power management */
	unsigned int vindpm : 4;
	/** enable HiZ */
	unsigned int en_hiz : 1;
};

/** Encapsulates the poweron config register */
struct poweron_config {
	/** register reset */
	unsigned int register_reset : 1;
	/** i2c watchdog timer reset */
	unsigned int watchdog_reset : 1;
	/** charger configuration */
	unsigned int charger_config : 2;
	/** minimum system voltage limit */
	unsigned int min_sys_voltage : 3;
	/** reserved - must write "1" */
	unsigned int reserved : 1;
};

/* driver structs */
struct bq24195_data {
	struct device *i2c;
	pmic_t pmic;
};

struct bq24195_dev_config {
	u16_t i2c_addr;
};

/* driver sensor channels */
enum pmic_sensor_chan {
	SENSOR_CHAN_PMIC_STATUS = SENSOR_CHAN_PRIV_START,
	SENSOR_CHAN_PMIC_FAULT,
	SENSOR_CHAN_PMIC_REGISTERS,
};

/* driver sensor attributes */
enum bq24195_sensor_attributes {
    SENSOR_ATTR_FAULT_COUNT = SENSOR_ATTR_PRIV_START,
    SENSOR_ATTR_LATCH,
};

#endif /* __PMIC_BQ24195_H__ */
