/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "cc.h"
#include "driver/timer_types_legacy.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "portmacro.h"
#include "rom/ets_sys.h"
#include "hal/gpio_types.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_intr_types.h"
#include <sys/time.h>
#include <time.h>
#include "ir_sensor.h"
#include "esp_timer.h"
#include "soc/gpio_num.h"
#include "soc/soc.h"
#include "ts4231.h"
#include "esp_task_wdt.h"
#include "freertos/timers.h"
#include "driver/timer.h"

#include "http_server_config.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "esp_ipc.h"

#include "esp_freertos_hooks.h"

#include "freertos/portmacro.h"
#include "soc/gpio_reg.h"
#include "soc/timer_group_reg.h"
/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";
static const char *IRTAG = "ir sensor";
static int s_retry_num = 0;


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

enum ir_type{
    ir_unknown,
    ir_main_hori, // < 40(60-70 is okay)
    ir_main_vert, 
    ir_sub_hori, // < 40(60-70 is okay)
    ir_sub_vert, 
};

inline void handle_ootx(int data_1_bit){

}
/*
void irsensor_handler(ir_sensor_config_t * arg){
    uint64_t time_us_mul;
    static uint64_t next_ir_predict_outdate_timing = 0;
    static enum ir_type next_ir_predict = ir_unknown;
    // static int wrong_combo = 0;
    timer_get_counter_value(0, 0, &time_us_mul);
    // if(time_us_mul - arg->last_noise_down < 50 * TIMER_TICK_PER_US){
    //     while(gpio_get_level(arg->D_pin)){
    //         // timer_get_counter_value(0, 0, &arg->last_noise_down);
    //     }
    //     return;//this is a noise
    // }
    // int64_t time_us = esp_timer_get_time();
    uint64_t down_time_mul = time_us_mul;
    uint64_t down_time_delta = 0;
    
    while(gpio_get_level(arg->D_pin)){
        timer_get_counter_value(0, 0, &down_time_mul);
        if((down_time_delta = down_time_mul - time_us_mul) > 550 * TIMER_TICK_PER_US)
            break;
    }

    if(time_us_mul >= next_ir_predict_outdate_timing)
        next_ir_predict = ir_unknown;

    switch (next_ir_predict) {
        case ir_main_hori:
            arg->angle_main_hori_us = time_us_mul - arg->last_upward_tick_us_mul;
            next_ir_predict = ir_unknown;
        break;
        case ir_main_vert:
            arg->angle_main_vert_us = time_us_mul - arg->last_upward_tick_us_mul;
            next_ir_predict = ir_unknown;
        break;
        case ir_sub_hori:
            arg->angle_sub_hori_us = time_us_mul - arg->last_upward_tick_us_mul;
            next_ir_predict = ir_unknown;
        break;
        case ir_sub_vert:
            arg->angle_sub_vert_us = time_us_mul - arg->last_upward_tick_us_mul;
            next_ir_predict = ir_unknown;
        break;
        case ir_unknown:
            if(down_time_delta > 60 * TIMER_TICK_PER_US){
                int data = 0;
                if(down_time_delta < 70 * TIMER_TICK_PER_US){
                    next_ir_predict = ir_main_hori;
                    data = 0;
                    arg->hori_delta = down_time_delta;
                }else if(down_time_delta < 80 * TIMER_TICK_PER_US){
                    next_ir_predict = ir_main_vert;
                    data = 0;
                }else if(down_time_delta < 90 * TIMER_TICK_PER_US){
                    next_ir_predict = ir_main_hori;
                    arg->hori_delta = down_time_delta;
                    data = 1;
                }else if(down_time_delta < 100 * TIMER_TICK_PER_US){
                    next_ir_predict = ir_main_vert;
                    data = 1;
                }else if(down_time_delta < 111 * TIMER_TICK_PER_US){
                    next_ir_predict = ir_sub_hori;
                    data = 0;
                }else if(down_time_delta < 121 * TIMER_TICK_PER_US){
                    next_ir_predict = ir_sub_vert;
                    data = 0;
                }else if(down_time_delta < 130 * TIMER_TICK_PER_US){
                    next_ir_predict = ir_sub_hori;
                    data = 1;
                }else if(down_time_delta < 140 * TIMER_TICK_PER_US){
                    next_ir_predict = ir_sub_vert;
                    data = 1;
                }else{
                    //unknown
                    break;
                }
                arg->last_upward_tick_us_mul = time_us_mul;
                next_ir_predict_outdate_timing = time_us_mul + 8100 * TIMER_TICK_PER_US;
                handle_ootx(data);
            }
        break;
    }
    // irtype = (irtype + 1)%ir_type_count;
    // if(arg->last_upward_delay_us > 50 && arg->last_upward_delay_us < 100)
    // arg->angle_us = time_us - arg->last_upward_tick_us;
    // timer_get_counter_value(0, 0, &arg->last_noise_down);
    //arg->last_upward_delay_us = down_time_delta;
}
*/
void irsensor_init(ir_sensor_config_t * config){
    //TODO: enable filter
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pin_bit_mask = (1<<config->D_pin) | (1<<config->E_pin),
    };
    gpio_config(&io_conf);

    ts4231_init(config);
    gpio_set_direction(config->E_pin, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(config->E_pin, 0);

    // gpio_set_level(config->D_pin, 1);
    // gpio_set_level(config->E_pin, 1);
    // gpio_set_direction(config->D_pin, GPIO_MODE_OUTPUT);
    // gpio_set_direction(config->E_pin, GPIO_MODE_OUTPUT);
    // ets_delay_us(200);
    // gpio_set_level(config->E_pin, 0);
    // ets_delay_us(200);
    // gpio_set_level(config->D_pin, 0);
    // ets_delay_us(20);
    // gpio_set_level(config->E_pin, 1);
    // ets_delay_us(200);
    // gpio_set_direction(config->D_pin, GPIO_MODE_INPUT);
    // gpio_set_direction(config->E_pin, GPIO_MODE_INPUT);


    // int readback;
    // readback = ts4231_readConfig(config);
    // ESP_LOGI(IRTAG, "read config %d", readback);
    // ts4231_writeConfig(config, 1111);
    // readback = ts4231_readConfig(config);
    // ESP_LOGI(IRTAG, "read config %d", readback);


    // if(ts4231_waitForLight(config, 3000)){
    //     uint8_t config_result = ts4231_configDevice(config, TS4231_CFG_WORD);
    
    //     //user can determine how to handle each return value for the configuration function
    //     switch (config_result) {
    //     case TS4231_CONFIG_PASS:
    //         ESP_LOGI(IRTAG, "Configuration SUCCESS");
    //         break;
    //     case TS4231_BUS_FAIL:  //unable to resolve state of TS4231 (3 samples of the bus signals resulted in 3 different states)
    //         ESP_LOGI(IRTAG, "Configuration Unsuccessful - BUS_FAIL");
    //         break;
    //     case TS4231_VERIFY_FAIL:  //configuration read value did not match configuration write value, run configuration again
    //         ESP_LOGI(IRTAG, "Configuration Unsuccessful - VERIFY_FAIL");
    //         break;
    //     case TS4231_WATCH_FAIL:  //verify succeeded but entry into WATCH mode failed, run configuration again
    //         ESP_LOGI(IRTAG, "Configuration Unsuccessful - WATCH_FAIL");
    //         break;
    //     default:  //value returned was unknown
    //         ESP_LOGI(IRTAG, "Program Execution ERROR");
    //         break;
    //     }


    //     ESP_LOGI(IRTAG, "%s\n", ts4231_goToWatch(config)?"W":"N");
    // }else{
    //     ESP_LOGE(IRTAG, "Can't wait for light for sensor");
    // }
    // gpio_num_t gpio_D = config->D_pin;
    // //handle IR data with interrupt
    // gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    // gpio_set_intr_type(gpio_D, GPIO_INTR_POSEDGE);
    // gpio_intr_disable(gpio_D);
    // //monitor the gpio_D
    // gpio_isr_handler_add(gpio_D, (gpio_isr_t)irsensor_handler, config);
    // gpio_intr_enable(gpio_D);
}

#define SENSORS_D_PIN_MASK ((1<<GPIO_NUM_19)|(1<<GPIO_NUM_26)|(1<<GPIO_NUM_16))
#define SENSORS_COUNT 3

ir_sensor_config_t sensors[] = {
    {
        .id = 0,
        .D_pin = GPIO_NUM_19,
        .E_pin = GPIO_NUM_18,
    },    {
        .id = 1,
        .D_pin = GPIO_NUM_26,
        .E_pin = GPIO_NUM_25,
    },    {
        .id = 2,
        .D_pin = GPIO_NUM_16,
        .E_pin = GPIO_NUM_17,
    },
};

ir_sensor_config_t * target_sensor = &sensors[2];

#include "lwip/sockets.h"
int m_socket;
struct sockaddr_in dest_addr;
void debugSendCallback(TimerHandle_t xTimer)
{
    // uint64_t payload[] = {
    //     target_sensor->angle_main_hori_us,
    //     target_sensor->angle_main_vert_us,
    //     target_sensor->angle_sub_hori_us,
    //     target_sensor->angle_sub_vert_us,   
    // };
    // sendto(m_socket, payload, sizeof(payload), 0, &dest_addr, sizeof(dest_addr));
    // static int angle_us = 0;
    // int temp;
    // if(angle_us != (temp = first_sensor.angle_main_hori_us)){
    //     angle_us = temp;
    //     printf("(% 6d-% 2d,% 6d-% 2d) (% 6d-% 2d, % 6d-% 2d) - (%d)\n", 
    //     first_sensor.angle_main_hori_us / TIMER_TICK_PER_US,first_sensor.angle_main_hori_us % TIMER_TICK_PER_US,
    //     first_sensor.angle_main_vert_us / TIMER_TICK_PER_US,first_sensor.angle_main_vert_us % TIMER_TICK_PER_US,
    //     first_sensor.angle_sub_hori_us / TIMER_TICK_PER_US,first_sensor.angle_sub_hori_us % TIMER_TICK_PER_US,
    //     first_sensor.angle_sub_vert_us / TIMER_TICK_PER_US,first_sensor.angle_sub_vert_us % TIMER_TICK_PER_US
    //     ,first_sensor.hori_delta);


    // }
    // else{
    //     // ESP_LOGI(IRTAG,"NOCHANGE\n");
    // }
}

timer_config_t ir_timer_cfg = {
    .alarm_en = 0,
    .counter_en = 1,
    .intr_type = 0,
    .counter_dir = TIMER_COUNT_UP,
    .auto_reload = TIMER_AUTORELOAD_EN,
    .divider = 2
};

struct sensor_status_event_t {
    uint32_t timestamp;
    uint32_t status;
};
volatile struct sensor_status_event_t sensor_event_queue[1024];
volatile int sensor_event_queue_head = 0; // next element to take
volatile int sensor_event_queue_tail = 0; // next element to add
volatile int sensor_event_lost = 0;

void sensor_init(){
    for(int i=0;i<sizeof(sensors)/sizeof(*sensors);i++){
        ir_sensor_config_t * config = &sensors[i];
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT_OUTPUT,
            .pin_bit_mask = (1<<config->D_pin) | (1<<config->E_pin),
        };
        gpio_config(&io_conf);

        ts4231_init(config);
        gpio_set_direction(config->E_pin, GPIO_MODE_INPUT_OUTPUT);
        gpio_set_level(config->E_pin, 0);
    }
}

//this tiny task runs in core 1
void sensor_read_task(void *){
    //say bye-bye to free rtos context switch
    portDISABLE_INTERRUPTS();
    //the iwdt has been disabled in config file
    uint32_t bits = 0;
    while(1){
        uint32_t this_bits = SENSORS_D_PIN_MASK & REG_READ(GPIO_IN_REG);
        if(this_bits != bits){
            //the first things is access the timer update reg, so the count value is recorded in T0LO
            REG_WRITE(TIMG_T0UPDATE_REG(0),1);
            int new_tail = sensor_event_queue_tail+1;
            struct sensor_status_event_t *next = &sensor_event_queue[new_tail%1024];
            next->timestamp = REG_READ(TIMG_T0LO_REG(0));
            next->status = this_bits;
            if(new_tail - sensor_event_queue_head < 1020)
                sensor_event_queue_tail = new_tail;
            else
                sensor_event_lost++;
            bits = this_bits;
        }
        //this code is not protected by task watchdog
    }


    // while(1){
    //     int newbits = 0;
    //     for(int i=0;i<sizeof(sensors)/sizeof(*sensors);i++){
    //         newbits <<=1;
    //         newbits |= gpio_get_level(sensors[i].D_pin);
    //     }
    //     if(newbits != bits){
    //         struct sensor_status_event_t * next = &sensor_event_queue[sensor_event_queue_tail % 1024];
    //         timer_get_counter_value(0, 0, &next->timestamp);
    //         next->status = newbits;
    //         if(sensor_event_queue_tail - sensor_event_queue_head < 1020){
    //             sensor_event_queue_tail++;
    //         }else{
    //             sensor_event_lost++;
    //         }
    //         bits = newbits;
    //     }
    // }
}

struct udp_payload_t{
    struct station_axis_info sensor_payload[SENSORS_COUNT][2];
};

void inform_data_changed(){
    struct udp_payload_t payload;
    assert(sizeof(payload.sensor_payload[0]) == sizeof(sensors[0].last_axis_info));
    for(int i=0;i<SENSORS_COUNT;i++){
        memcpy(payload.sensor_payload[i], sensors[i].last_axis_info, sizeof(payload.sensor_payload[0]));
    }
    sendto(m_socket, &payload, sizeof(payload), 0, (const struct sockaddr*)&dest_addr, sizeof(dest_addr));
}

void sensor_handle_timer_task(TimerHandle_t xTimer){
    volatile int head_tail_distance = sensor_event_queue_tail - sensor_event_queue_head;
    while((head_tail_distance = sensor_event_queue_tail - sensor_event_queue_head, head_tail_distance) > 0){//be careful: this operation depends on undefined behavior!
        struct sensor_status_event_t * s = (/* remove the volatile to optimize the program */struct sensor_status_event_t*)&sensor_event_queue[sensor_event_queue_head%1024];
        // ESP_LOGI("sensor", "%d %d %d %lu", !!(s->status & (1<<sensors[0].D_pin)),!!(s->status & (1<<sensors[1].D_pin)),!!(s->status & (1<<sensors[2].D_pin)), s->timestamp);
        for(int i=0;i<sizeof(sensors)/sizeof(*sensors);i++){
            int new_status = !!(s->status & 1<<sensors[i].D_pin);
            ir_sensor_config_t * sensor = &sensors[i];
            if(sensor->current_status == new_status){
                continue;
            }
            sensor->current_status = new_status;
            //handle the new status
            if(new_status){
                //low to high

                //end of cycle
                if(sensor->stage == swait_waiting_for_scan_and_has_next_info && s->timestamp - sensor->last_upward_tick_us_mul > 8200 * TIMER_TICK_PER_US){
                    // ESP_LOGI("SENSOR","reset->(%lld) lost packet %d", (s->timestamp - sensor->last_upward_tick_us_mul)/TIMER_TICK_PER_US, sensor_event_lost);
                    sensor->stage = swait_none;
                }
                switch(sensor->stage){
                    case swait_none:
                        sensor->last_upward_tick_us_mul = s->timestamp;
                        sensor->stage = swait_syncing_high;
                        break;
                    case swait_syncing_high:
                        //impossible
                        break;
                    case swait_waiting_for_scan_and_has_next_info:
                        sensor->axis_distance_left = s->timestamp - sensor->last_upward_tick_us_mul;
                        sensor->stage = swait_waiting_for_scan_down_and_has_next_info;
                        break;
                    case swait_waiting_for_scan_down_and_has_next_info:
                        //impossible
                        break;
                }
            }else{
                //high to low
                switch(sensor->stage){
                    case swait_none:
                        //impossible
                        break;
                    case swait_syncing_high:
                        int high_delta = s->timestamp - sensor->last_upward_tick_us_mul;
                        if(high_delta > 60 * TIMER_TICK_PER_US){
                            int data;
                            int station_idx, axis;
                            if(high_delta < 70 * TIMER_TICK_PER_US){
                                station_idx = 0;axis = 0; data = 0;
                            }else if(high_delta < 80 * TIMER_TICK_PER_US){
                                station_idx = 0;axis = 1; data = 0;
                            }else if(high_delta < 90 * TIMER_TICK_PER_US){
                                station_idx = 0;axis = 0; data = 1;
                            }else if(high_delta < 100 * TIMER_TICK_PER_US){
                                station_idx = 0;axis = 1; data = 1;
                            }else if(high_delta < 111 * TIMER_TICK_PER_US){
                                station_idx = 1; axis = 0; data = 0;
                            }else if(high_delta < 121 * TIMER_TICK_PER_US){
                                station_idx = 1; axis = 1; data = 0;
                            }else if(high_delta < 130 * TIMER_TICK_PER_US){
                                station_idx = 1; axis = 0; data = 1;
                            }else if(high_delta < 140 * TIMER_TICK_PER_US){
                                station_idx = 1; axis = 1; data = 1;
                            }else{
                                //unknown
                                sensor->stage = swait_none;
                                break;
                            }
                            sensor->stage = swait_waiting_for_scan_and_has_next_info;
                            sensor->next_station_id = station_idx;
                            sensor->next_axis = axis;
                            memcpy(&sensor->last_axis_info, &sensor->axis_info, sizeof(sensor->axis_info));
                            sensor->axis_info[station_idx].axis[axis].count = 0;

                            handle_ootx(data);
                        }else{
                            sensor->stage = swait_none;
                            break;
                        }
                        break;
                    case swait_waiting_for_scan_and_has_next_info:
                        //impossible
                        break;
                    case swait_waiting_for_scan_down_and_has_next_info:
                        int axis_distance_right = s->timestamp - sensor->last_upward_tick_us_mul;
                        // sensor->axis_info[sensor->next_station_id].axis[sensor->next_axis] = (sensor->axis_distance_left + axis_distance_right)/2;
                        struct axis_value * axis = &sensor->axis_info[sensor->next_station_id].axis[sensor->next_axis];
                        if(axis->count < sizeof(axis->values) / sizeof(*(axis->values))){
                            axis->values[axis->count].start = sensor->axis_distance_left;
                            axis->values[axis->count].width = axis_distance_right - sensor->axis_distance_left;
                            axis->count++;
                            sensor->has_new_data = true;
                        }
                        sensor->stage = swait_waiting_for_scan_and_has_next_info;
                        break;
                }
            }
        }

        if(s->status == SENSORS_D_PIN_MASK){
            //三个传感器全亮，说明正在同步
            for(int i=0;i<SENSORS_COUNT;i++){
                struct ir_sensor_config_t * ss = &sensors[i];
                if(ss->stage != swait_syncing_high){
                    ss->last_upward_tick_us_mul = s->timestamp;
                    ss->stage = swait_syncing_high;
                }

                memcpy(&ss->last_axis_info, &ss->axis_info, sizeof(ss->last_axis_info));
            }
            inform_data_changed();
        }
        sensor_event_queue_head++;
    }
}

#define SENSOR_READ_STACK_DEPTH 2048
TaskHandle_t sensorReadTaskHandle;
StackType_t sensorReadTaskStack[SENSOR_READ_STACK_DEPTH];
StaticTask_t sensorReadTask;
void app_main(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    timer_init(0, 0, &ir_timer_cfg);

    m_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    dest_addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(9001);

    // irsensor_init(target_sensor);
    
    timer_start(0,0);
    
    TimerHandle_t timer = xTimerCreate("DebugTimer",pdMS_TO_TICKS(10),pdTRUE,(void*)0,
        sensor_handle_timer_task);
    xTimerStart(timer, 0);

    sensor_init();
    sensorReadTaskHandle = xTaskCreateStaticPinnedToCore(sensor_read_task, "sensor-read-task", SENSOR_READ_STACK_DEPTH, NULL, tskIDLE_PRIORITY,sensorReadTaskStack, &sensorReadTask, 1);
    
    // init_server_for_sensor(&first_sensor);
    return;
}
  