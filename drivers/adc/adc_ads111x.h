/**
 * @file adc_ads111x.h
 * @author Steven Slupsky (sslupsky@scanimetrics.com)
 * @brief 
 * @version 0.1
 * @date 2020-04-14
 * 
 * @copyright Copyright (c) 2020
 *
 */


#ifndef ZEPHYR_DRIVERS_ADC_ADS111X_H_
#define ZEPHYR_DRIVERS_ADC_ADS111X_H_

/*  not specified in datasheet */
#define ADS111X_POWER_ON_TIME_USEC	1000

/*=========================================================================
    I2C ADDRESS/BITS
    -----------------------------------------------------------------------*/
    #define ADS111X_ADDRESS                 (0x48)    // 1001 000 (ADDR = GND)
/*=========================================================================*/

/*=========================================================================
    CONVERSION DELAY (in mS)
    -----------------------------------------------------------------------*/
    #define ADS111X_CONVERSIONDELAY         (1)
    #define ADS1115_CONVERSIONDELAY         (8)
/*=========================================================================*/

/*=========================================================================
    POINTER REGISTER
    -----------------------------------------------------------------------*/
    #define ADS111X_REG_POINTER_MASK        (0x03)
    #define ADS111X_REG_POINTER_CONVERT     (0x00)
    #define ADS111X_REG_POINTER_CONFIG      (0x01)
    #define ADS111X_REG_POINTER_LOWTHRESH   (0x02)
    #define ADS111X_REG_POINTER_HITHRESH    (0x03)
/*=========================================================================*/

/*=========================================================================
    CONFIG REGISTER
    -----------------------------------------------------------------------*/
    #define ADS111X_REG_CONFIG_OS_MASK      BIT(15)
    #define ADS111X_REG_CONFIG_OS_POS	    15
    #define ADS111X_REG_CONFIG_OS_SINGLE    (0x8000)  // Write: Set to start a single-conversion
    #define ADS111X_REG_CONFIG_OS_BUSY      (0x0000)  // Read: Bit = 0 when conversion is in progress
    #define ADS111X_REG_CONFIG_OS_NOTBUSY   (0x8000)  // Read: Bit = 1 when device is not performing a conversion

    #define ADS111X_REG_CONFIG_MUX_MASK     (0x7000)
    #define ADS111X_REG_CONFIG_MUX_DIFF_0_1 (0x0000)  // Differential P = AIN0, N = AIN1 (default)
    #define ADS111X_REG_CONFIG_MUX_DIFF_0_3 (0x1000)  // Differential P = AIN0, N = AIN3
    #define ADS111X_REG_CONFIG_MUX_DIFF_1_3 (0x2000)  // Differential P = AIN1, N = AIN3
    #define ADS111X_REG_CONFIG_MUX_DIFF_2_3 (0x3000)  // Differential P = AIN2, N = AIN3
    #define ADS111X_REG_CONFIG_MUX_SINGLE_0 (0x4000)  // Single-ended AIN0
    #define ADS111X_REG_CONFIG_MUX_SINGLE_1 (0x5000)  // Single-ended AIN1
    #define ADS111X_REG_CONFIG_MUX_SINGLE_2 (0x6000)  // Single-ended AIN2
    #define ADS111X_REG_CONFIG_MUX_SINGLE_3 (0x7000)  // Single-ended AIN3

    #define ADS111X_REG_CONFIG_PGA_MASK     (0x0E00)
    #define ADS111X_REG_CONFIG_PGA_6_144V   (0x0000)  // +/-6.144V range = Gain 2/3
    #define ADS111X_REG_CONFIG_PGA_4_096V   (0x0200)  // +/-4.096V range = Gain 1
    #define ADS111X_REG_CONFIG_PGA_2_048V   (0x0400)  // +/-2.048V range = Gain 2 (default)
    #define ADS111X_REG_CONFIG_PGA_1_024V   (0x0600)  // +/-1.024V range = Gain 4
    #define ADS111X_REG_CONFIG_PGA_0_512V   (0x0800)  // +/-0.512V range = Gain 8
    #define ADS111X_REG_CONFIG_PGA_0_256V   (0x0A00)  // +/-0.256V range = Gain 16

    #define ADS111X_REG_CONFIG_MODE_MASK    (0x0100)
    #define ADS111X_REG_CONFIG_MODE_CONTIN  (0x0000)  // Continuous conversion mode
    #define ADS111X_REG_CONFIG_MODE_SINGLE  (0x0100)  // Power-down single-shot mode (default)

    #define ADS111X_REG_CONFIG_DR_MASK      (0x00E0)  

    #define ADS111X_REG_CONFIG_CMODE_MASK   (0x0010)
    #define ADS111X_REG_CONFIG_CMODE_TRAD   (0x0000)  // Traditional comparator with hysteresis (default)
    #define ADS111X_REG_CONFIG_CMODE_WINDOW (0x0010)  // Window comparator

    #define ADS111X_REG_CONFIG_CPOL_MASK    (0x0008)
    #define ADS111X_REG_CONFIG_CPOL_ACTVLOW (0x0000)  // ALERT/RDY pin is low when active (default)
    #define ADS111X_REG_CONFIG_CPOL_ACTVHI  (0x0008)  // ALERT/RDY pin is high when active

    #define ADS111X_REG_CONFIG_CLAT_MASK    (0x0004)  // Determines if ALERT/RDY pin latches once asserted
    #define ADS111X_REG_CONFIG_CLAT_NONLAT  (0x0000)  // Non-latching comparator (default)
    #define ADS111X_REG_CONFIG_CLAT_LATCH   (0x0004)  // Latching comparator

    #define ADS111X_REG_CONFIG_CQUE_MASK    (0x0003)
    #define ADS111X_REG_CONFIG_CQUE_1CONV   (0x0000)  // Assert ALERT/RDY after one conversions
    #define ADS111X_REG_CONFIG_CQUE_2CONV   (0x0001)  // Assert ALERT/RDY after two conversions
    #define ADS111X_REG_CONFIG_CQUE_4CONV   (0x0002)  // Assert ALERT/RDY after four conversions
    #define ADS111X_REG_CONFIG_CQUE_NONE    (0x0003)  // Disable the comparator and put ALERT/RDY in high state (default)
/*=========================================================================*/

#define ADS111X_ODR_PERIOD_860SPS	1
#define ADS111X_ODR_PERIOD_475SPS	2
#define ADS111X_ODR_PERIOD_250SPS	4
#define ADS111X_ODR_PERIOD_128SPS	8
#define ADS111X_ODR_PERIOD_64SPS	15
#define ADS111X_ODR_PERIOD_32SPS	31
#define ADS111X_ODR_PERIOD_16SPS	62
#define ADS111X_ODR_PERIOD_8SPS		125


/*
 * Approximated ads111x acquisition times in milliseconds. These are
 * used for the initial delay when polling for data ready.
 */
static const u32_t ads111x_odr_delay_tbl[8] = {
	ADS111X_ODR_PERIOD_8SPS,	/* 8 SPS */
	ADS111X_ODR_PERIOD_16SPS,	/* 16 SPS */
	ADS111X_ODR_PERIOD_32SPS,	/* 32 SPS */
	ADS111X_ODR_PERIOD_64SPS, 	/* 64 SPS */
	ADS111X_ODR_PERIOD_128SPS, 	/* 128 SPS */
	ADS111X_ODR_PERIOD_250SPS,	/* 250 SPS */
	ADS111X_ODR_PERIOD_475SPS,	/* 475 SPS */
	ADS111X_ODR_PERIOD_860SPS,	/* 860 SPS (default) */
};

#define ADS111X_CONFIG_ODR_POS		5
#define ADS111X_CONFIG_ODR_SEL(x)	((x & BIT_MASK(3)) << ADS111X_CONFIG_ODR_POS)
#define ADS111X_CONFIG_GAIN_POS		9
#define ADS111X_CONFIG_GAIN_SEL(x)	((x & BIT_MASK(3)) << ADS111X_CONFIG_GAIN_POS)
#define ADS111X_CONFIG_MUX_POS		12
#define ADS111X_CONFIG_MUX_SEL(x)	((x & BIT_MASK(2)) << ADS111X_CONFIG_MUX_POS)
#define ADS111X_CONFIG_CQUE_SEL(x)	(x & BIT_MASK(2))
#define ADS111X_CONFIG_INPUT_TYPE	BIT(14)
#define ADS111X_CONFIG_OS		BIT(15)
#define ADS111X_CONFIG_MODE		BIT(8)

enum ADS111X_CONFIG_ODR_e {
	ADS111X_CONFIG_ODR_8SPS = 0,
	ADS111X_CONFIG_ODR_16SPS,
	ADS111X_CONFIG_ODR_32SPS,
	ADS111X_CONFIG_ODR_64SPS,
	ADS111X_CONFIG_ODR_128SPS,
	ADS111X_CONFIG_ODR_250SPS,
	ADS111X_CONFIG_ODR_475SPS,
	ADS111X_CONFIG_ODR_860SPS,
};

#endif /* ZEPHYR_DRIVERS_ADC_ADS111X_H_ */