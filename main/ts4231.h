/*****************************************************
 *
 *  Copyright (C) 2024 Frto027
 *
 *  Modified under MIT License
 *****************************************************/

/*******************************************************************
    Copyright (C) 2017 Triad Semiconductor

    ts4231.h - Library for configuring the Triad Semiconductor TS4231 Light
               to Digital converter.
    Created by: John Seibel
*******************************************************************/

#ifndef ts4231_h
#define ts4231_h

#include <stdint.h>
#include "ir_sensor.h"

#define TS4231_BUS_DRV_DLY 1      // delay in microseconds between bus level changes
#define TS4231_BUS_CHECK_DLY 500  // delay in microseconds for the checkBus() function
#define TS4231_SLEEP_RECOVERY 100 // delay in microseconds for analog wake-up after exiting SLEEP mode
#define TS4231_UNKNOWN_STATE 0x04 // checkBus() function state
#define TS4231_S3_STATE 0x03      // checkBus() function state
#define TS4231_WATCH_STATE 0x02   // checkBus() function state
#define TS4231_SLEEP_STATE 0x01   // checkBus() function state
#define TS4231_S0_STATE 0x00      // checkBus() function state
#define TS4231_CFG_WORD 0x392B    // configuration value
#define TS4231_BUS_FAIL 0x01      // configDevice() function status return value
#define TS4231_VERIFY_FAIL 0x02   // configDevice() function status return value
#define TS4231_WATCH_FAIL 0x03    // configDevice() function status return value
#define TS4231_CONFIG_PASS 0x04   // configDevice() function status return value

void ts4231_init(ir_sensor_config_t *dev);
bool ts4231_waitForLight(ir_sensor_config_t *dev, uint16_t light_timeout); // timeout in milliseconds
bool ts4231_goToSleep(ir_sensor_config_t *dev);
uint8_t ts4231_configDevice(ir_sensor_config_t *dev, uint16_t config_val /*= CFG_WORD*/);
bool ts4231_goToWatch(ir_sensor_config_t *dev);

uint8_t ts4231_checkBus(ir_sensor_config_t *dev);
void ts4231_writeConfig(ir_sensor_config_t *dev, uint16_t config_val);
bool ts4231_goToWatch(ir_sensor_config_t *dev);
uint16_t ts4231_readConfig(ir_sensor_config_t *dev);

#endif