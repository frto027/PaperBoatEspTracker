/*****************************************************
 *
 *  Copyright (C) 2024 Frto027
 *
 *  Modified under MIT License, port to ESP32
 *****************************************************/

/*******************************************************************
    Copyright (C) 2017 Triad Semiconductor

    ts4231.h - Library for configuring the Triad Semiconductor TS4231 Light
               to Digital converter.
    Created by: John Seibel
*******************************************************************/

#include "ts4231.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <rom/ets_sys.h>

const char * TAG = "ts4231";

// uint8_t ts4231_checkBus(ir_sensor_config_t *dev);
// uint8_t ts4231_ts_digitalRead(ir_sensor_config_t *dev, int pin);
// void ts4231_ts_digitalWrite(ir_sensor_config_t *dev, int pin, uint8_t write_val);
// unsigned ts4231_long_ts_millis(void);
// void ts4231_writeConfig(ir_sensor_config_t *dev, uint16_t config_val);
// uint16_t ts4231_readConfig(ir_sensor_config_t *dev);

#define LOW 0
#define HIGH 1

#define BUS_DRV_DLY     TS4231_BUS_DRV_DLY
#define BUS_CHECK_DLY   TS4231_BUS_CHECK_DLY
#define SLEEP_RECOVERY  TS4231_SLEEP_RECOVERY
#define UNKNOWN_STATE   TS4231_UNKNOWN_STATE
#define S3_STATE        TS4231_S3_STATE
#define WATCH_STATE     TS4231_WATCH_STATE
#define SLEEP_STATE     TS4231_SLEEP_STATE
#define S0_STATE        TS4231_S0_STATE
#define CFG_WORD        TS4231_CFG_WORD
#define BUS_FAIL        TS4231_BUS_FAIL
#define VERIFY_FAIL     TS4231_VERIFY_FAIL
#define WATCH_FAIL      TS4231_WATCH_FAIL
#define CONFIG_PASS     TS4231_CONFIG_PASS


void gpio_set_direction_my(gpio_num_t gpio, int direct){
    // if(direct == GPIO_MODE_INPUT){
    //     gpio_set_pull_mode(gpio, GPIO_FLOATING);
    // }else{
    //     // gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
    // }
    gpio_set_direction(gpio, direct);
    ets_delay_us(10);
}
#define gpio_set_direction gpio_set_direction_my

void ts4231_init(ir_sensor_config_t *dev)
{
    dev->configured = false;
    gpio_set_direction(dev->E_pin, GPIO_MODE_INPUT);
    gpio_set_direction(dev->D_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(dev->E_pin, GPIO_FLOATING);
    gpio_set_pull_mode(dev->D_pin, GPIO_FLOATING);
    // gpio_dump_io_configuration(stdout, (1<<dev->D_pin));
    ets_delay_us(1000);
}

#define ts_delayUs(us) ets_delay_us(us)
// void ts_delayUs(int64_t us)
// {
//     int64_t m = esp_timer_get_time() + us;
//     while(esp_timer_get_time() < m)
//         continue;
//     // if(us){
//     //     uint32_t e = (m + us);
//     //     if(m > e){ //overflow
//     //         while(esp_timer_get_time() > e){
//     //             continue;
//     //         }
//     //     }
//     //     while(esp_timer_get_time() < e){
//     //         continue;
//     //     }
//     // }

// }

uint8_t ts_digitalRead(gpio_num_t pin)
{
    uint8_t read_val;
    ets_delay_us(100);
    read_val = gpio_get_level(pin);
    return read_val;
}

void ts_digitalWrite(gpio_num_t pin, uint8_t write_val)
{
    gpio_set_level(pin, write_val);
    // A short delay function can be inserted here to extend the time between writes to
    // the E and D outputs if TS4231 timing parameters are being violated.  Consult
    // the TS4231 datasheet for more information on timing parameters.  It is recommended
    // that any added delay be no longer than approximately 1us.
    // for(int i=0;i<1;i++){
    //     asm("nop");
    // }
    ets_delay_us(100);
}

unsigned long ts_millis()
{
    unsigned long current_time;

    current_time = esp_timer_get_time() / 1000;
    return current_time;
}

// Function waitForLight() should be executed after power-up and prior to
// configuring the device.  Upon power-up, D is a 0 and will output a 1
// when light is detected.  D will return to 0 at the end of light detection.
// This funciton looks for the falling edge of D to indicate that the end of
// light detection has occurred.
bool ts4231_waitForLight(ir_sensor_config_t *dev, uint16_t light_timeout)
{
    bool light = false;
    bool exit = false;
    unsigned long time0;

    if (ts4231_checkBus(dev) == S0_STATE)
    {
        time0 = ts_millis();
        while (exit == false)
        {
            if (ts_digitalRead(dev->D_pin) > 0)
            {
                while (exit == false)
                {
                    if (ts_digitalRead(dev->D_pin) == 0)
                    {
                        exit = true;
                        light = true;
                    }
                    else if (ts_millis() > (time0 + light_timeout))
                    {
                        exit = true;
                        light = false;
                    }
                    else
                    {
                        exit = false;
                        light = false;
                    }
                }
            }
            else if (ts_millis() > (time0 + light_timeout))
            {
                exit = true;
                light = false;
            }
            else
            {
                exit = false;
                light = false;
            }
        }
    }
    else
        light = true; // if not in state S0_state, light has already been detected
    return light;
}

bool ts4231_goToSleep(ir_sensor_config_t *dev)
{
    bool sleep_success;

    if (dev->configured == false)
        sleep_success = false;
    else
    {
        switch (ts4231_checkBus(dev))
        {
        case S0_STATE:
            sleep_success = false;
            break;
        case SLEEP_STATE:
            sleep_success = true;
            break;
        case WATCH_STATE:
            ts_digitalWrite(dev->E_pin, LOW);
            gpio_set_direction(dev->E_pin, GPIO_MODE_OUTPUT);
            ts_delayUs(BUS_DRV_DLY);
            gpio_set_direction(dev->E_pin, GPIO_MODE_INPUT);
            ts_delayUs(BUS_DRV_DLY);
            if (ts4231_checkBus(dev) == SLEEP_STATE)
                sleep_success = true;
            else
                sleep_success = false;
            break;
        case S3_STATE:
            sleep_success = false;
            break;
        default:
            sleep_success = false;
            break;
        }
    }
    return sleep_success;
}

uint8_t ts4231_configDevice(ir_sensor_config_t *dev, uint16_t config_val)
{
    uint8_t config_success = 0x00;
    uint16_t readback;

    dev->configured = false;
    gpio_set_direction(dev->D_pin, GPIO_MODE_INPUT);
    gpio_set_direction(dev->E_pin, GPIO_MODE_INPUT);
    ts_digitalWrite(dev->D_pin, LOW);
    ts_digitalWrite(dev->E_pin, LOW);
    gpio_set_direction(dev->E_pin, GPIO_MODE_OUTPUT);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->E_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->E_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->E_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    gpio_set_direction(dev->D_pin, GPIO_MODE_OUTPUT);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->D_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    gpio_set_direction(dev->E_pin, GPIO_MODE_INPUT);
    gpio_set_direction(dev->D_pin, GPIO_MODE_INPUT);
    if (ts4231_checkBus(dev) == S3_STATE)
    {
        ESP_LOGI(TAG, "config val before write:%d", ts4231_readConfig(dev));
        ts4231_writeConfig(dev, config_val);
        readback = ts4231_readConfig(dev);
        if (readback == config_val)
        {
            dev->configured = true;
            if (ts4231_goToWatch(dev))
                config_success = CONFIG_PASS;
            else
                config_success = WATCH_FAIL;
        }
        else
            config_success = VERIFY_FAIL;
    }
    else
        config_success = BUS_FAIL;

    return config_success;
}

void ts4231_writeConfig(ir_sensor_config_t *dev, uint16_t config_val)
{
    ts_digitalWrite(dev->E_pin, HIGH);
    ts_digitalWrite(dev->D_pin, HIGH);
    gpio_set_direction(dev->E_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->D_pin, GPIO_MODE_OUTPUT);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->D_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->E_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    for (uint8_t i = 0; i < 15; i++)
    {
        config_val = config_val << 1;
        if ((config_val & 0x8000) > 0)
            ts_digitalWrite(dev->D_pin, HIGH);
        else
            ts_digitalWrite(dev->D_pin, LOW);
        ts_delayUs(BUS_DRV_DLY);
        ts_digitalWrite(dev->E_pin, HIGH);
        ts_delayUs(BUS_DRV_DLY);
        ts_digitalWrite(dev->E_pin, LOW);
        ts_delayUs(BUS_DRV_DLY);
    }
    ts_digitalWrite(dev->D_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->E_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->D_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    gpio_set_direction(dev->E_pin, GPIO_MODE_INPUT);
    gpio_set_direction(dev->D_pin, GPIO_MODE_INPUT);
}

uint16_t ts4231_readConfig(ir_sensor_config_t *dev)
{
    uint16_t readback;

    readback = 0x0000;
    ts_digitalWrite(dev->E_pin, HIGH);
    ts_digitalWrite(dev->D_pin, HIGH);
    gpio_set_direction(dev->E_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(dev->D_pin, GPIO_MODE_OUTPUT);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->D_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->E_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->D_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->E_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    gpio_set_direction(dev->D_pin, GPIO_MODE_INPUT);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->E_pin, LOW);
    ts_delayUs(BUS_DRV_DLY);
    for (uint8_t i = 0; i < 14; i++)
    {
        ts_digitalWrite(dev->E_pin, HIGH);
        ts_delayUs(BUS_DRV_DLY);
        readback = (readback << 1) | (ts_digitalRead(dev->D_pin) & 0x0001);
        ts_digitalWrite(dev->E_pin, LOW);
        ts_delayUs(BUS_DRV_DLY);
    }
    ts_digitalWrite(dev->D_pin, LOW);
    gpio_set_direction(dev->D_pin, GPIO_MODE_OUTPUT);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->E_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    ts_digitalWrite(dev->D_pin, HIGH);
    ts_delayUs(BUS_DRV_DLY);
    gpio_set_direction(dev->E_pin, GPIO_MODE_INPUT);
    gpio_set_direction(dev->D_pin, GPIO_MODE_INPUT);
    return readback;
}

// checkBus() performs a voting function where the bus is sampled 3 times
// to find 2 identical results.  This is necessary since light detection is
// asynchronous and can indicate a false state.
uint8_t ts4231_checkBus(ir_sensor_config_t *dev)
{
    uint8_t state;
    uint8_t E_state;
    uint8_t D_state;
    uint8_t S0_count = 0;
    uint8_t SLEEP_count = 0;
    uint8_t WATCH_count = 0;
    uint8_t S3_count = 0;

    for (uint8_t i = 0; i < 3; i++)
    {
        E_state = ts_digitalRead(dev->E_pin);
        D_state = ts_digitalRead(dev->D_pin);
        if (D_state == HIGH)
        {
            if (E_state == HIGH)
                S3_count++;
            else
                SLEEP_count++;
        }
        else
        {
            if (E_state == HIGH)
                WATCH_count++;
            else
                S0_count++;
        }
        ts_delayUs(BUS_CHECK_DLY);
    }
    if (SLEEP_count >= 2)
        state = SLEEP_STATE;
    else if (WATCH_count >= 2)
        state = WATCH_STATE;
    else if (S3_count >= 2)
        state = S3_STATE;
    else if (S0_count >= 2)
        state = S0_STATE;
    else
        state = UNKNOWN_STATE;
    ESP_LOGI(TAG, "check bus %d", state);
    return state;
}

bool ts4231_goToWatch(ir_sensor_config_t *dev)
{
    bool watch_success;

    if (dev->configured == false)
        watch_success = false;
    else
    {
        switch (ts4231_checkBus(dev))
        {
        case S0_STATE:
            watch_success = false;
            break;
        case SLEEP_STATE:
            ts_digitalWrite(dev->D_pin, HIGH);
            gpio_set_direction(dev->D_pin, GPIO_MODE_OUTPUT);
            ts_digitalWrite(dev->E_pin, LOW);
            gpio_set_direction(dev->E_pin, GPIO_MODE_OUTPUT);
            ts_digitalWrite(dev->D_pin, LOW);
            gpio_set_direction(dev->D_pin, GPIO_MODE_INPUT);
            ts_digitalWrite(dev->E_pin, HIGH);
            gpio_set_direction(dev->E_pin, GPIO_MODE_INPUT);
            ts_delayUs(SLEEP_RECOVERY);
            if (ts4231_checkBus(dev) == WATCH_STATE)
                watch_success = true;
            else
                watch_success = false;
            break;
        case WATCH_STATE:
            watch_success = true;
            break;
        case S3_STATE:
            ts_digitalWrite(dev->E_pin, HIGH);
            gpio_set_direction(dev->E_pin, GPIO_MODE_OUTPUT);
            ts_digitalWrite(dev->D_pin, HIGH);
            gpio_set_direction(dev->D_pin, GPIO_MODE_OUTPUT);
            ts_digitalWrite(dev->E_pin, LOW);
            ts_digitalWrite(dev->D_pin, LOW);
            gpio_set_direction(dev->D_pin, GPIO_MODE_INPUT);
            ts_digitalWrite(dev->E_pin, HIGH);
            gpio_set_direction(dev->E_pin, GPIO_MODE_INPUT);
            ts_delayUs(SLEEP_RECOVERY);
            if (ts4231_checkBus(dev) == WATCH_STATE)
                watch_success = true;
            else
                watch_success = false;
            break;
        default:
            watch_success = false;
            break;
        }
    }
    return watch_success;
}