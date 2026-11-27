#pragma once
#include "imu.h"
typedef struct {
    float q0,q1,q2,q3;
    float gyro_offset[3];
    float linear_accel_x,linear_accel_y,linear_accel_z;
} fusion_t;
void fusion_reset(fusion_t *s);
void fusion_update(fusion_t *s,const imu_sample_t *raw,float dt);
