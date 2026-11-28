#pragma once
#include <stdint.h>
#define BMI_MOSI 6
#define BMI_MISO 5
#define BMI_SCLK 4
#define BMI1_CS 10
#define BMI2_CS 3
#define MPU_SDA 8
#define MPU_SCL 9
#define MPU_ADDR 0x68
#define BUTTON_GPIO 7
#define MADGWICK_BETA 0.08f
#define DEG_TO_RAD 0.017453292519943295f
#define GRAVITY 9.80665f
#define ESPNOW_WIFI_CHANNEL 1
#define PACKET_TYPE_SENSOR 1
#define SENSOR_COUNT 3
static const uint8_t receiver_mac[6] = {0x34,0xb7,0xda,0xfb,0x23,0xf8};
