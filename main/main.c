#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"

#define SDA_PIN 8
#define SCL_PIN 9
#define MPU_ADDR 0x68

#define DEG_TO_RAD 0.017453292519943295f
#define MADGWICK_BETA 0.08f

static i2c_master_dev_handle_t mpu;

static float gyro_offset[3] = {0};

// Quaternion: w, x, y, z
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

static int16_t be16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return i2c_master_transmit(mpu, data, 2, 1000);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(
        mpu, &reg, 1, data, len, 1000
    );
}

static void calibrate_gyro(void)
{
    printf("Keep MPU6050 still for 3 seconds...\n");

    float sum[3] = {0};
    int valid = 0;

    for (int i = 0; i < 300; i++) {
        uint8_t data[6];

        if (read_regs(0x43, data, 6) == ESP_OK) {
            sum[0] += be16(data[0], data[1]) / 131.0f;
            sum[1] += be16(data[2], data[3]) / 131.0f;
            sum[2] += be16(data[4], data[5]) / 131.0f;
            valid++;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (valid < 250) {
        printf("Gyro calibration failed: %d samples\n", valid);
        return;
    }

    for (int i = 0; i < 3; i++) {
        gyro_offset[i] = sum[i] / valid;
    }

    printf("Calibration complete: %.3f %.3f %.3f\n",
           gyro_offset[0], gyro_offset[1], gyro_offset[2]);
}

// 6-axis Madgwick filter
// Gyroscope inputs must be in radians/second.
static void madgwick_update(float gx, float gy, float gz,
                            float ax, float ay, float az,
                            float dt)
{
    float norm;
    float s0, s1, s2, s3;

    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    norm = sqrtf(ax * ax + ay * ay + az * az);

    if (norm > 0.000001f) {
        ax /= norm;
        ay /= norm;
        az /= norm;

        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float _4q0 = 4.0f * q0;
        float _4q1 = 4.0f * q1;
        float _4q2 = 4.0f * q2;
        float _8q1 = 8.0f * q1;
        float _8q2 = 8.0f * q2;

        float q0q0 = q0 * q0;
        float q1q1 = q1 * q1;
        float q2q2 = q2 * q2;
        float q3q3 = q3 * q3;

        s0 = _4q0 * q2q2 + _2q2 * ax
           + _4q0 * q1q1 - _2q1 * ay;

        s1 = _4q1 * q3q3 - _2q3 * ax
           + 4.0f * q0q0 * q1 - _2q0 * ay
           - _4q1 + _8q1 * q1q1 + _8q1 * q2q2
           + _4q1 * az;

        s2 = 4.0f * q0q0 * q2 + _2q0 * ax
           + _4q2 * q3q3 - _2q3 * ay
           - _4q2 + _8q2 * q1q1 + _8q2 * q2q2
           + _4q2 * az;

        s3 = 4.0f * q1q1 * q3 - _2q1 * ax
           + 4.0f * q2q2 * q3 - _2q2 * ay;

        norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);

        if (norm > 0.000001f) {
            s0 /= norm;
            s1 /= norm;
            s2 /= norm;
            s3 /= norm;

            qDot0 -= MADGWICK_BETA * s0;
            qDot1 -= MADGWICK_BETA * s1;
            qDot2 -= MADGWICK_BETA * s2;
            qDot3 -= MADGWICK_BETA * s3;
        }
    }

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    if (norm > 0.000001f) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
}

void app_main(void)
{
    i2c_master_bus_handle_t bus;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU_ADDR,
        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(bus, &dev_config, &mpu)
    );

    // Wake MPU6050
    ESP_ERROR_CHECK(write_reg(0x6B, 0x00));

    // Accelerometer +/- 2g
    ESP_ERROR_CHECK(write_reg(0x1C, 0x00));

    // Gyroscope +/- 250 degrees/second
    ESP_ERROR_CHECK(write_reg(0x1B, 0x00));

    // Digital low-pass filter
    ESP_ERROR_CHECK(write_reg(0x1A, 0x03));

    vTaskDelay(pdMS_TO_TICKS(100));

    calibrate_gyro();

    printf("MPU6050 orientation tracking started\n");

    int64_t last_time = esp_timer_get_time();

    while (1) {
        uint8_t data[14];

        if (read_regs(0x3B, data, 14) == ESP_OK) {
            int64_t now = esp_timer_get_time();
            float dt = (now - last_time) / 1000000.0f;
            last_time = now;

            if (dt <= 0.0f || dt > 0.1f) {
                dt = 0.02f;
            }

            float ax = be16(data[0], data[1]) / 16384.0f;
            float ay = be16(data[2], data[3]) / 16384.0f;
            float az = be16(data[4], data[5]) / 16384.0f;

            float gx = (be16(data[8], data[9]) / 131.0f
                        - gyro_offset[0]) * DEG_TO_RAD;

            float gy = (be16(data[10], data[11]) / 131.0f
                        - gyro_offset[1]) * DEG_TO_RAD;

            float gz = (be16(data[12], data[13]) / 131.0f
                        - gyro_offset[2]) * DEG_TO_RAD;

            madgwick_update(gx, gy, gz, ax, ay, az, dt);

            // USB serial quaternion output
            printf("Q,%.6f,%.6f,%.6f,%.6f\n",
                   q0, q1, q2, q3);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
