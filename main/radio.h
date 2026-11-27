#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "fusion.h"
#define PACKET_TYPE_MULTI_SENSOR 2
typedef struct {float q0,q1,q2,q3;float accel_x,accel_y,accel_z;} sensor_values_t;
typedef struct {
    uint8_t packet_type,button_pressed,sensor_count,reserved;
    uint32_t sequence_number;
    sensor_values_t sensors[3];
} multi_sensor_packet_t;
_Static_assert(sizeof(multi_sensor_packet_t)==92,"Unexpected packet size");
void radio_init(void);
void radio_send(const fusion_t sensors[3],bool button_pressed);
void radio_stats(void);
