// SPDX-License-Identifier: GPL-2.0-only
/*
 * An I2C and SPI driver for the NXP PCF2127/29 RTC
 * Copyright 2022 Simone Tollardo
 * 
 * Author: Simone Tollardo <simonetollardo@gmail.com>
 *
 * Datasheet: https://www.nxp.com/docs/en/data-sheet/PCF2127.pdf
 */

#ifndef __ESP32_PCF2127_H__
#define __ESP32_PCF2127_H__

#include <stdint.h>
#include "driver/i2c.h"
#include <time.h>
#include <sys/time.h>
#include <esp_err.h>


/* NOTE: As a consequence of this method, it is very important to make a read or write access in
one go. That is, setting or reading seconds through to years should be made in one single
access. Failing to comply with this method could result in the time becoming corrupted.
As an example, if the time (seconds through to hours) is set in one access and then in a
second access the date is set, it is possible that the time may increment between the two
accesses. A similar problem exists when reading. A roll-over may occur between reads
thus giving the minutes from one moment and the hours from the next. Therefore it is
advised to read all time and date registers in one access.
*/

/* Remark: For the PCF2127, a repeated START is not allowed. Therefore a STOP has to
be released before the next START.
The number of data bytes transferred between the START and STOP conditions from
transmitter to receiver is unlimited. Each byte of eight bits is followed by an acknowledge
cycle.
*/

#define PCF2127_I2C_ADDRESS             0x51

/* Control register 1 */
#define PCF2127_REG_CTRL1		        0x00
#define PCF2127_BIT_CTRL1_SI            BIT(0)
#define PCF2127_BIT_CTRL1_MI            BIT(1)
#define PCF2127_BIT_CTRL1_12_24         BIT(2)
#define PCF2127_BIT_CTRL1_POR_OVRD		BIT(3)
#define PCF2127_BIT_CTRL1_TSF1			BIT(4)
#define PCF2127_BIT_CTRL1_STOP          BIT(5)
/* Control register 2 */
#define PCF2127_REG_CTRL2		        0x01
#define PCF2127_BIT_CTRL2_CDTIE         BIT(0)
#define PCF2127_BIT_CTRL2_AIE			BIT(1)
#define PCF2127_BIT_CTRL2_TSIE			BIT(2)
#define PCF2127_BIT_CTRL2_CDTF          BIT(3)
#define PCF2127_BIT_CTRL2_AF			BIT(4)
#define PCF2127_BIT_CTRL2_TSF2			BIT(5)
#define PCF2127_BIT_CTRL2_WDTF			BIT(6)
#define PCF2127_BIT_CTRL2_MSF			BIT(7)
/* Control register 3 */
#define PCF2127_REG_CTRL3		        0x02
#define PCF2127_BIT_CTRL3_BLIE			BIT(0)
#define PCF2127_BIT_CTRL3_BIE			BIT(1)
#define PCF2127_BIT_CTRL3_BLF			BIT(2)
#define PCF2127_BIT_CTRL3_BF			BIT(3)
#define PCF2127_BIT_CTRL3_BTSE			BIT(4)
#define PCF2127_BIT_CTRL3_PWRMNG_LOW	BIT(5)
#define PCF2127_BIT_CTRL3_PWRMNG_MID	BIT(6)
#define PCF2127_BIT_CTRL3_PWRMNG_HIGH	BIT(7)
/* Time and date registers */
#define PCF2127_REG_SC			        0x03
#define PCF2127_BIT_SC_OSF			    BIT(7)
#define PCF2127_REG_MN			        0x04
#define PCF2127_REG_HR			        0x05
#define PCF2127_BIT_AMPM                BIT(5)  //only if 12h mode selected in control register 1
#define PCF2127_REG_DM			        0x06
#define PCF2127_REG_DW			        0x07
#define PCF2127_REG_MO			        0x08
#define PCF2127_REG_YR			        0x09
/* Alarm registers */
#define PCF2127_REG_ALARM_SC		    0x0A
#define PCF2127_REG_ALARM_MN		    0x0B
#define PCF2127_REG_ALARM_HR		    0x0C
#define PCF2127_BIT_ALARM_AMPM          BIT(5)  //only if 12h mode selected in control register 1
#define PCF2127_REG_ALARM_DM		    0x0D
#define PCF2127_REG_ALARM_DW		    0x0E
#define PCF2127_BIT_ALARM_AE			BIT(7)  //enable is pin 7 for all alarm registers
/* CLKOUT control register */
#define PCF2127_REG_CLKOUT		        0x0f
#define PCF2127_BIT_CLKOUT_COF_LOW      BIT(0)
#define PCF2127_BIT_CLKOUT_COF_MID      BIT(1)
#define PCF2127_BIT_CLKOUT_COF_HIGH     BIT(2)
#define PCF2127_BIT_CLKOUT_OTPR			BIT(5)
#define PCF2127_BIT_CLKOUT_TCR_LOW	    BIT(6)
#define PCF2127_BIT_CLKOUT_TCR_HIGH	    BIT(7)
/* Watchdog registers */
#define PCF2127_REG_WD_CTL		        0x10
#define PCF2127_BIT_WD_CTL_TF0			BIT(0)
#define PCF2127_BIT_WD_CTL_TF1			BIT(1)
#define PCF2127_BIT_WD_TI_TP            BIT(5)
#define PCF2127_BIT_WD_CTL_CD0			BIT(6)
#define PCF2127_BIT_WD_CTL_CD1			BIT(7)
#define PCF2127_REG_WD_VAL		        0x11
/* Tamper timestamp registers */
#define PCF2127_REG_TS_CTRL		        0x12
#define PCF2127_BIT_TS_CTRL_TSOFF		BIT(6)
#define PCF2127_BIT_TS_CTRL_TSM			BIT(7)
#define PCF2127_REG_TS_SC		        0x13
#define PCF2127_REG_TS_MN		        0x14
#define PCF2127_REG_TS_HR		        0x15
#define PCF2127_REG_TS_DM		        0x16
#define PCF2127_REG_TS_MO		        0x17
#define PCF2127_REG_TS_YR		        0x18
/* Register Aging_offset */
#define PCF2127_REG_AO		            0x19
#define PCF2127_BIT_AO_1                BIT(0)
#define PCF2127_BIT_AO_2                BIT(1)
#define PCF2127_BIT_AO_3                BIT(2)
#define PCF2127_BIT_AO_4                BIT(3)
/*
 * RAM registers
 * PCF2127 has 512 bytes general-purpose static RAM (SRAM) that is
 * battery backed and can survive a power outage.
 * PCF2129 doesn't have this feature.
 */
#define PCF2127_REG_RAM_ADDR_MSB	    0x1A
#define PCF2127_REG_RAM_ADDR_LSB	    0x1B
#define PCF2127_REG_RAM_WRT_CMD		    0x1C
#define PCF2127_REG_RAM_RD_CMD		    0x1D

/* Watchdog timer value constants */
#define PCF2127_WD_VAL_STOP		        (0)
#define PCF2127_WD_VAL_MIN		        (2)
#define PCF2127_WD_VAL_MAX		        (255)
#define PCF2127_WD_VAL_DEFAULT		    (60)

// ---------------- BIN2BCD/BCD2BIN AUXILIARY MACROS ----------------
#define bcd2bin(x) ((uint8_t) (((x) & 0x0f) + ((x) >> 4) * 10))
#define bin2bcd(x) ((uint8_t) ((((x) / 10) << 4) + (x) % 10))

struct alm {
    struct tm tm;
    bool enabled;
    bool pending;
};

/**
 *  @brief  Struct that stores state and functions for interacting with
 *          AS7341 Spectral Sensor
 */
typedef struct{
    uint8_t i2c_port;
    bool init;
    struct tm time;
    struct alm alarm;
	bool irq_enabled;
    bool batt_Low;
    bool OSF;
}   ESP32_PCF2127;

void PCF_init(ESP32_PCF2127* PCF, uint8_t i2c_port);

/* Time and date functions */
struct tm* PCF_rtc_read_time(ESP32_PCF2127* PCF);
void PCF_rtc_set_time(ESP32_PCF2127* PCF, struct tm* time);

// int pcf2127_nvmem_read(void *priv, unsigned int offset, void *val, size_t bytes);
// int pcf2127_nvmem_write(void *priv, unsigned int offset, void *val, size_t bytes);

/* watchdog driver */
// int pcf2127_wdt_ping(struct watchdog_device *wdd);
// int pcf2127_wdt_active_ping(struct watchdog_device *wdd);
// int pcf2127_wdt_start(struct watchdog_device *wdd);
// int pcf2127_wdt_stop(struct watchdog_device *wdd);
// int pcf2127_wdt_set_timeout(struct watchdog_device *wdd, unsigned int new_timeout);
// int pcf2127_watchdog_init(struct device *dev, struct pcf2127 *pcf2127);

/* Alarm */
struct alm* PCF_rtc_read_alarm(ESP32_PCF2127* PCF);
void PCF_rtc_alarm_irq_enable(ESP32_PCF2127* PCF, bool enable);
void PCF_rtc_set_alarm(ESP32_PCF2127* PCF, struct alm* alarm);

// int pcf2127_rtc_ts_read(struct device *dev, time64_t *ts);
// void pcf2127_rtc_ts_snapshot(struct device *dev);

#endif