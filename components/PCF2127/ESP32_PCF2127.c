// SPDX-License-Identifier: GPL-2.0-only
/*
 * An I2C and SPI driver for the NXP PCF2127/29 RTC
 * Copyright 2013 Til-Technologies
 *
 * Author: Renaud Cerrato <r.cerrato@til-technologies.fr>
 *
 * Watchdog and tamper functions
 * Author: Bruno Thomsen <bruno.thomsen@gmail.com>
 * 
 * ESP-IDF Porting
 * Author: Simone Tollardo <simonetollardo@gmail.com>
 *
 * based on the other drivers in this same directory.
 *
 * Datasheet: https://www.nxp.com/docs/en/data-sheet/PCF2127.pdf
 */

#include "ESP32_PCF2127.h"
#include <esp_err.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>

//TODO: are we sure about this?
//  Note: Reading CTRL2 register causes watchdog to stop which is unfortunate,
//  since register also contain control/status flags for other features.
//  Always call this function after reading CTRL2 register.
//  See functions below.

static const char* TAG="ESP32_PCF2127";

// ---------------- I2C AUXILIARY FUNCTIONS ----------------

/**
 *  @brief  read <LEN> bytes over I2C
 *  @param  reg
 *  @param  buffer
 *  @param  len
 */
static void pcf_i2c_read(uint8_t reg, uint8_t* buffer, uint8_t len) {
    uint8_t reg_buffer[1] = {reg};
    ESP_ERROR_CHECK(i2c_master_write_to_device(PCF_I2C_PORT, PCF2127_I2C_ADDRESS, reg_buffer, 1, 1000 / portTICK_PERIOD_MS));
    ESP_ERROR_CHECK(i2c_master_read_from_device(PCF_I2C_PORT, PCF2127_I2C_ADDRESS, buffer, len, 1000 / portTICK_PERIOD_MS));
}

/**
 *  @brief  write a byte over I2C
 *  @param  reg
 *  @param  data
 *
 */
static void pcf_i2c_write8(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    ESP_ERROR_CHECK(i2c_master_write_to_device(PCF_I2C_PORT, PCF2127_I2C_ADDRESS, buffer, 2, 1000 / portTICK_PERIOD_MS));
}

/**
 *  @brief  write 5 bytes (40 bits) over I2C
 *  @param  reg
 *  @param  data
 *
 */
static void pcf_i2c_write40(uint8_t reg, uint8_t* data) {
    uint8_t buffer[6] = {reg, data[0], data[1], data[2], data[3], data[4]};
    ESP_ERROR_CHECK(i2c_master_write_to_device(PCF_I2C_PORT, PCF2127_I2C_ADDRESS, buffer, 6, 1000 / portTICK_PERIOD_MS));
}

/**
 *  @brief  write 7 bytes (56 bits) over I2C
 *  @param  reg
 *  @param  data
 *
 */
static void pcf_i2c_write56(uint8_t reg, uint8_t* data) {
    uint8_t buffer[8] = {reg, data[0], data[1], data[2], data[3], data[4], data[5], data[6]};
    ESP_ERROR_CHECK(i2c_master_write_to_device(PCF_I2C_PORT, PCF2127_I2C_ADDRESS, buffer, 8, 1000 / portTICK_PERIOD_MS));
}

/**
 * @brief Write bits in a register
 * 
 * @param reg register to write to
 * @param mask mask to apply
 * @param value value to write
 */
static void pcf_i2c_writeBits(uint8_t reg, uint8_t mask, uint8_t value) {
    uint8_t tmp[1];
    pcf_i2c_read(reg, tmp, 1);
    tmp[0] &= ~mask;
    tmp[0] |= (value & mask);
    pcf_i2c_write8(reg, tmp[0]);
}

// ---------------- PCF INIT/DELETE FUNCTIONS ----------------

esp_err_t PCF_init(ESP32_PCF2127* PCF){
    //Load default values


    //Set I2C configuration
    PCF->conf.mode = I2C_MODE_MASTER;
    PCF->conf.sda_io_num = PCF_SDA_PIN;
    PCF->conf.scl_io_num = PCF_SCL_PIN;
    PCF->conf.sda_pullup_en = GPIO_PULLUP_DISABLE;     //disable if you have external pullup
    PCF->conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    PCF->conf.master.clk_speed = 400000;               //I2C Full Speed

    ESP_ERROR_CHECK(i2c_param_config(PCF_I2C_PORT, &(PCF->conf))); //set I2C Config

    ESP_ERROR_CHECK(i2c_driver_install(PCF_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    PCF->init = true;

    /* Set default values */


    return ESP_OK;
}

void PCF_delete(){
    ESP_ERROR_CHECK(i2c_driver_delete(PCF_I2C_PORT));
}

// ---------------- RTC READ/SET TIME FUNCTIONS ----------------

/*
 * In the routines that deal directly with the pcf2127 hardware, we use
 * rtc_time -- month 0-11, hour 0-23, yr = calendar year-epoch.
 */
void PCF_rtc_read_time(ESP32_PCF2127* PCF)
{
	uint8_t buf[9];
	/*
	 * Avoid reading CTRL2 register as it causes WD_VAL register
	 * value to reset to 0 which means watchdog is stopped.
	 */
	pcf_i2c_read(PCF2127_REG_CTRL3, buf, 9);

	if (buf[0] & PCF2127_BIT_CTRL3_BLF){
        PCF->batt_Low=true;
		ESP_LOGE(TAG, "Low voltage detected, date/time not reliable, check RTC battery");
    }

	/* Clock integrity is not guaranteed when OSF flag is set. */
	if (buf[1] & PCF2127_BIT_SC_OSF) {
		/*
		 * no need clear the flag here,
		 * it will be cleared in the set_time function.
         * 
		 */
		PCF->OSF=true;
		ESP_LOGE(TAG, "Oscillator stop detected, date/time is not reliable");
	}

    /* Save the time and the date in the PCF2127 struct */
	PCF->time.tm_sec = bcd2bin(buf[1] & 0x7F);
	PCF->time.tm_min = bcd2bin(buf[2] & 0x7F);
	PCF->time.tm_hour = bcd2bin(buf[3] & 0x3F); /* rtc hr 0-23 */
	PCF->time.tm_mday = bcd2bin(buf[4] & 0x3F);
	PCF->time.tm_wday = buf[5] & 0x07;
	PCF->time.tm_mon = bcd2bin(buf[6] & 0x1F) - 1; /* rtc mn 1-12 */
	PCF->time.tm_year = bcd2bin(buf[7]);
}

void PCF_rtc_set_time(ESP32_PCF2127* PCF)
{
	unsigned char buf[7];

	/* hours, minutes and seconds */
	buf[0] = bin2bcd(PCF->time.tm_sec);	/* this will also clear OSF flag */
	buf[1] = bin2bcd(PCF->time.tm_min);
	buf[2] = bin2bcd(PCF->time.tm_hour);
	buf[3] = bin2bcd(PCF->time.tm_mday);
	buf[4] = PCF->time.tm_wday & 0x07;
	buf[5] = bin2bcd(PCF->time.tm_mon + 1);	/* month, 1 - 12 */
	buf[6] = bin2bcd(PCF->time.tm_year);

    PCF->OSF=false;

	/* write register's data */
    pcf_i2c_write56(PCF2127_REG_SC, buf);

}

// ---------------- ALARM FUNCTIONS ----------------

void PCF_rtc_read_alarm(ESP32_PCF2127* PCF)
{
	uint8_t buf[5];
	uint8_t ctrl2;

	pcf_i2c_read(PCF2127_REG_CTRL2, &ctrl2, 1);

    // TODO: ? See todo in the top of the file
	// pcf2127_wdt_active_ping(&pcf2127->wdd);

	pcf_i2c_read(PCF2127_REG_ALARM_SC, buf, 5);

	PCF->alarm.enabled = ctrl2 & PCF2127_BIT_CTRL2_AIE;
	PCF->alarm.pending = ctrl2 & PCF2127_BIT_CTRL2_AF;

	PCF->alarm.tm_sec = bcd2bin(buf[0] & 0x7F);
	PCF->alarm.tm_min = bcd2bin(buf[1] & 0x7F);
	PCF->alarm.tm_hour = bcd2bin(buf[2] & 0x3F);
	PCF->alarm.tm_mday = bcd2bin(buf[3] & 0x3F);
    PCF->alarm.tm_wday = buf[4] & 0x07;
}

void PCF_rtc_alarm_irq_enable(ESP32_PCF2127* PCF, bool enable)
{
    pcf_i2c_writeBits(PCF2127_REG_CTRL2, PCF2127_BIT_CTRL2_AIE, (uint8_t)enable << 1);

	// TODO: ? See todo in the top of the file
	// pcf2127_wdt_active_ping(&pcf2127->wdd);
}

void PCF_rtc_set_alarm(ESP32_PCF2127* PCF)
{
	uint8_t buf[5];

    pcf_i2c_writeBits(PCF2127_REG_CTRL2, PCF2127_BIT_CTRL2_AF, 0x00);

    // TODO: ? See todo in the top of the file
	// pcf2127_wdt_active_ping(&pcf2127->wdd);

    /*NOTE: all alarms en bit are clear so all alarms are enabled, 
    * also weekdays alarm
    */
	buf[0] = bin2bcd(PCF->alarm.tm_sec);
	buf[1] = bin2bcd(PCF->alarm.tm_min);
	buf[2] = bin2bcd(PCF->alarm.tm_hour);
	buf[3] = bin2bcd(PCF->alarm.tm_mday);
    buf[4] = PCF->alarm.tm_wday;

	pcf_i2c_write40(PCF2127_REG_ALARM_SC, buf);
    PCF->alarm.enabled = true;
    PCF_rtc_alarm_irq_enable(PCF, PCF->alarm.enabled);
}

// ---------------- NOT IMPLEMENTED FUNCTIONS ----------------

// int pcf2127_nvmem_read(void *priv, unsigned int offset,
// 			      void *val, size_t bytes)
// {
// 	struct pcf2127 *pcf2127 = priv;
// 	int ret;
// 	unsigned char offsetbuf[] = { offset >> 8, offset };

// 	ret = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_RAM_ADDR_MSB,
// 				offsetbuf, 2);
// 	if (ret)
// 		return ret;

// 	return regmap_bulk_read(pcf2127->regmap, PCF2127_REG_RAM_RD_CMD,
// 				val, bytes);
// }

// int pcf2127_nvmem_write(void *priv, unsigned int offset,
// 			       void *val, size_t bytes)
// {
// 	struct pcf2127 *pcf2127 = priv;
// 	int ret;
// 	unsigned char offsetbuf[] = { offset >> 8, offset };

// 	ret = regmap_bulk_write(pcf2127->regmap, PCF2127_REG_RAM_ADDR_MSB,
// 				offsetbuf, 2);
// 	if (ret)
// 		return ret;

// 	return regmap_bulk_write(pcf2127->regmap, PCF2127_REG_RAM_WRT_CMD,
// 				 val, bytes);
// }

// /* watchdog driver */

// int pcf2127_wdt_ping(struct watchdog_device *wdd)
// {
// 	struct pcf2127 *pcf2127 = watchdog_get_drvdata(wdd);

// 	return regmap_write(pcf2127->regmap, PCF2127_REG_WD_VAL, wdd->timeout);
// }

// /*
//  * Restart watchdog timer if feature is active.
//  *
//  * Note: Reading CTRL2 register causes watchdog to stop which is unfortunate,
//  * since register also contain control/status flags for other features.
//  * Always call this function after reading CTRL2 register.
//  */
// int pcf2127_wdt_active_ping(struct watchdog_device *wdd)
// {
// 	int ret = 0;

// 	if (watchdog_active(wdd)) {
// 		ret = pcf2127_wdt_ping(wdd);
// 		if (ret)
// 			dev_err(wdd->parent,
// 				"%s: watchdog restart failed, ret=%d\n",
// 				__func__, ret);
// 	}

// 	return ret;
// }

// int pcf2127_wdt_start(struct watchdog_device *wdd)
// {
// 	return pcf2127_wdt_ping(wdd);
// }

// int pcf2127_wdt_stop(struct watchdog_device *wdd)
// {
// 	struct pcf2127 *pcf2127 = watchdog_get_drvdata(wdd);

// 	return regmap_write(pcf2127->regmap, PCF2127_REG_WD_VAL,
// 			    PCF2127_WD_VAL_STOP);
// }

// int pcf2127_wdt_set_timeout(struct watchdog_device *wdd,
// 				   unsigned int new_timeout)
// {
// 	dev_dbg(wdd->parent, "new watchdog timeout: %is (old: %is)\n",
// 		new_timeout, wdd->timeout);

// 	wdd->timeout = new_timeout;

// 	return pcf2127_wdt_active_ping(wdd);
// }

// int pcf2127_watchdog_init(struct device *dev, struct pcf2127 *pcf2127)
// {
// 	u32 wdd_timeout;
// 	int ret;

// 	if (!IS_ENABLED(CONFIG_WATCHDOG) ||
// 	    !device_property_read_bool(dev, "reset-source"))
// 		return 0;

// 	pcf2127->wdd.parent = dev;
// 	pcf2127->wdd.info = &pcf2127_wdt_info;
// 	pcf2127->wdd.ops = &pcf2127_watchdog_ops;
// 	pcf2127->wdd.min_timeout = PCF2127_WD_VAL_MIN;
// 	pcf2127->wdd.max_timeout = PCF2127_WD_VAL_MAX;
// 	pcf2127->wdd.timeout = PCF2127_WD_VAL_DEFAULT;
// 	pcf2127->wdd.min_hw_heartbeat_ms = 500;
// 	pcf2127->wdd.status = WATCHDOG_NOWAYOUT_INIT_STATUS;

// 	watchdog_set_drvdata(&pcf2127->wdd, pcf2127);

// 	/* Test if watchdog timer is started by bootloader */
// 	ret = regmap_read(pcf2127->regmap, PCF2127_REG_WD_VAL, &wdd_timeout);
// 	if (ret)
// 		return ret;

// 	if (wdd_timeout)
// 		set_bit(WDOG_HW_RUNNING, &pcf2127->wdd.status);

// 	return devm_watchdog_register_device(dev, &pcf2127->wdd);
// }

// /*
//  * This function reads ctrl2 register, caller is responsible for calling
//  * pcf2127_wdt_active_ping()
//  */
// int pcf2127_rtc_ts_read(struct device *dev, time64_t *ts)
// {
// 	struct pcf2127 *pcf2127 = dev_get_drvdata(dev);
// 	struct rtc_time tm;
// 	int ret;
// 	unsigned char data[25];

// 	ret = regmap_bulk_read(pcf2127->regmap, PCF2127_REG_CTRL1, data,
// 			       sizeof(data));
// 	if (ret) {
// 		dev_err(dev, "%s: read error ret=%d\n", __func__, ret);
// 		return ret;
// 	}

// 	dev_dbg(dev,
// 		"%s: raw data is cr1=%02x, cr2=%02x, cr3=%02x, ts_sc=%02x, ts_mn=%02x, ts_hr=%02x, ts_dm=%02x, ts_mo=%02x, ts_yr=%02x\n",
// 		__func__, data[PCF2127_REG_CTRL1], data[PCF2127_REG_CTRL2],
// 		data[PCF2127_REG_CTRL3], data[PCF2127_REG_TS_SC],
// 		data[PCF2127_REG_TS_MN], data[PCF2127_REG_TS_HR],
// 		data[PCF2127_REG_TS_DM], data[PCF2127_REG_TS_MO],
// 		data[PCF2127_REG_TS_YR]);

// 	tm.tm_sec = bcd2bin(data[PCF2127_REG_TS_SC] & 0x7F);
// 	tm.tm_min = bcd2bin(data[PCF2127_REG_TS_MN] & 0x7F);
// 	tm.tm_hour = bcd2bin(data[PCF2127_REG_TS_HR] & 0x3F);
// 	tm.tm_mday = bcd2bin(data[PCF2127_REG_TS_DM] & 0x3F);
// 	/* TS_MO register (month) value range: 1-12 */
// 	tm.tm_mon = bcd2bin(data[PCF2127_REG_TS_MO] & 0x1F) - 1;
// 	tm.tm_year = bcd2bin(data[PCF2127_REG_TS_YR]);
// 	if (tm.tm_year < 70)
// 		tm.tm_year += 100; /* assume we are in 1970...2069 */

// 	ret = rtc_valid_tm(&tm);
// 	if (ret) {
// 		dev_err(dev, "Invalid timestamp. ret=%d\n", ret);
// 		return ret;
// 	}

// 	*ts = rtc_tm_to_time64(&tm);
// 	return 0;
// };

// void pcf2127_rtc_ts_snapshot(struct device *dev)
// {
// 	struct pcf2127 *pcf2127 = dev_get_drvdata(dev);
// 	int ret;

// 	/* Let userspace read the first timestamp */
// 	if (pcf2127->ts_valid)
// 		return;

// 	ret = pcf2127_rtc_ts_read(dev, &pcf2127->ts);
// 	if (!ret)
// 		pcf2127->ts_valid = true;
// }

// irqreturn_t pcf2127_rtc_irq(int irq, void *dev)
// {
// 	struct pcf2127 *pcf2127 = dev_get_drvdata(dev);
// 	unsigned int ctrl1, ctrl2;
// 	int ret = 0;

// 	ret = regmap_read(pcf2127->regmap, PCF2127_REG_CTRL1, &ctrl1);
// 	if (ret)
// 		return IRQ_NONE;

// 	ret = regmap_read(pcf2127->regmap, PCF2127_REG_CTRL2, &ctrl2);
// 	if (ret)
// 		return IRQ_NONE;

// 	if (!(ctrl1 & PCF2127_CTRL1_IRQ_MASK || ctrl2 & PCF2127_CTRL2_IRQ_MASK))
// 		return IRQ_NONE;

// 	if (ctrl1 & PCF2127_BIT_CTRL1_TSF1 || ctrl2 & PCF2127_BIT_CTRL2_TSF2)
// 		pcf2127_rtc_ts_snapshot(dev);

// 	if (ctrl1 & PCF2127_CTRL1_IRQ_MASK)
// 		regmap_write(pcf2127->regmap, PCF2127_REG_CTRL1,
// 			ctrl1 & ~PCF2127_CTRL1_IRQ_MASK);

// 	if (ctrl2 & PCF2127_CTRL2_IRQ_MASK)
// 		regmap_write(pcf2127->regmap, PCF2127_REG_CTRL2,
// 			ctrl2 & ~PCF2127_CTRL2_IRQ_MASK);

// 	if (ctrl2 & PCF2127_BIT_CTRL2_AF)
// 		rtc_update_irq(pcf2127->rtc, 1, RTC_IRQF | RTC_AF);

// 	pcf2127_wdt_active_ping(&pcf2127->wdd);

// 	return IRQ_HANDLED;
// }

// /* sysfs interface */

// ssize_t timestamp0_store(struct device *dev,
// 				struct device_attribute *attr,
// 				const char *buf, size_t count)
// {
// 	struct pcf2127 *pcf2127 = dev_get_drvdata(dev->parent);
// 	int ret;

// 	if (pcf2127->irq_enabled) {
// 		pcf2127->ts_valid = false;
// 	} else {
// 		ret = regmap_update_bits(pcf2127->regmap, PCF2127_REG_CTRL1,
// 			PCF2127_BIT_CTRL1_TSF1, 0);
// 		if (ret) {
// 			dev_err(dev, "%s: update ctrl1 ret=%d\n", __func__, ret);
// 			return ret;
// 		}

// 		ret = regmap_update_bits(pcf2127->regmap, PCF2127_REG_CTRL2,
// 			PCF2127_BIT_CTRL2_TSF2, 0);
// 		if (ret) {
// 			dev_err(dev, "%s: update ctrl2 ret=%d\n", __func__, ret);
// 			return ret;
// 		}

// 		ret = pcf2127_wdt_active_ping(&pcf2127->wdd);
// 		if (ret)
// 			return ret;
// 	}

// 	return count;
// };

// ssize_t timestamp0_show(struct device *dev,
// 			       struct device_attribute *attr, char *buf)
// {
// 	struct pcf2127 *pcf2127 = dev_get_drvdata(dev->parent);
// 	unsigned int ctrl1, ctrl2;
// 	int ret;
// 	time64_t ts;

// 	if (pcf2127->irq_enabled) {
// 		if (!pcf2127->ts_valid)
// 			return 0;
// 		ts = pcf2127->ts;
// 	} else {
// 		ret = regmap_read(pcf2127->regmap, PCF2127_REG_CTRL1, &ctrl1);
// 		if (ret)
// 			return 0;

// 		ret = regmap_read(pcf2127->regmap, PCF2127_REG_CTRL2, &ctrl2);
// 		if (ret)
// 			return 0;

// 		if (!(ctrl1 & PCF2127_BIT_CTRL1_TSF1) &&
// 		    !(ctrl2 & PCF2127_BIT_CTRL2_TSF2))
// 			return 0;

// 		ret = pcf2127_rtc_ts_read(dev->parent, &ts);
// 		if (ret)
// 			return 0;

// 		ret = pcf2127_wdt_active_ping(&pcf2127->wdd);
// 		if (ret)
// 			return ret;
// 	}
// 	return sprintf(buf, "%llu\n", (unsigned long long)ts);
// };