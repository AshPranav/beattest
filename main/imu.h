#pragma once
#include <stdbool.h>
#include "esp_err.h"
typedef struct {float ax,ay,az,gx,gy,gz;} imu_sample_t;
esp_err_t imu_init(void);
bool imu_read(int index, imu_sample_t *out);
