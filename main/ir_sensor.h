#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "driver/gpio.h"
#define TIMER_TICK_PER_US (APB_CLK_FREQ / (2 * 1000*1000))

enum sensor_waiting_stage{
    swait_none,
    swait_syncing_high,
    swait_waiting_for_scan_and_has_next_info,
    swait_waiting_for_scan_down_and_has_next_info
};

struct axis_value{
    // int v;
    int count;
    int __padding__;
    struct{
        int start,width;
    }values[3];
};

struct ir_sensor_config_t {
    int id;
    gpio_num_t E_pin;
    gpio_num_t D_pin;
    uint64_t last_upward_tick_us_mul;

    enum sensor_waiting_stage stage;
    int next_station_id;
    int next_axis;

    int axis_distance_left; // L->H time delta

    struct station_axis_info{
        struct axis_value axis[2/*x or y*/];
    } axis_info[2/*station id*/], last_axis_info[2];

    // int hori_delta;
    // int upward_distance[3];
    // int upward_distance_idx;

    // int angle_main_hori_us;
    // int angle_main_vert_us;

    // int angle_sub_hori_us;
    // int angle_sub_vert_us;

    //ts4231, do NOT configure for base station 1.0
    bool configured;
    
    bool current_status;

    bool has_new_data;
};

typedef struct ir_sensor_config_t ir_sensor_config_t;

#endif