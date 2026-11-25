
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_idf_version.h"

#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

// ==========================================
// 1. CONFIGURATION
// ==========================================

#define SDA_PIN 8
#define SCL_PIN 9
#define MPU_ADDR 0x68

#define DEG_TO_RAD 0.017453292519943295f
#define MADGWICK_BETA 0.08f

#define ESPNOW_WIFI_CHANNEL 1
#define PACKET_TYPE_SENSOR 1

#define GRAVITY 9.80665f

// Receiver ESP32 MAC address
static const uint8_t receiver_mac[6] = {
    0x34, 0xb7, 0xda, 0xfb, 0x23, 0xf8
};

// ==========================================
// 2. GLOBAL VARIABLES
// ==========================================

// MPU6050 I2C device handle
static i2c_master_dev_handle_t mpu;

// Gyroscope calibration offsets (degrees/second)
static float gyro_offset[3] = {0};

// Quaternion: w, x, y, z
static float q0 = 1.0f;
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;

// Linear acceleration (m/s^2)
static float linear_accel_x = 0.0f;
static float linear_accel_y = 0.0f;
static float linear_accel_z = 0.0f;

// ESP-NOW transmission counters
static volatile uint32_t tx_success_count = 0;
static volatile uint32_t tx_failure_count = 0;

// ESP-NOW synchronization semaphore
static SemaphoreHandle_t send_sem;

// ==========================================
// 3. ESP-NOW PACKET STRUCTURE
// ==========================================

typedef struct {
    uint8_t packet_type;
    uint8_t reserved[3];

    uint32_t sequence_number;

    // Quaternion
    float q0;
    float q1;
    float q2;
    float q3;

    // Linear acceleration (m/s^2)
    float accel_x;
    float accel_y;
    float accel_z;

} sensor_packet_t;

// Verify packet size at compile time
_Static_assert(
    sizeof(sensor_packet_t) == 36,
    "Unexpected sensor packet size"
);

// Actual transmission packet
static sensor_packet_t tx_packet = {0};

// Sequence counter
static uint32_t sequence_number = 0;

// ==========================================
// 4. MPU6050 I2C FUNCTIONS
// ==========================================

// Convert two bytes into signed 16-bit integer
static int16_t be16(uint8_t hi, uint8_t lo)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

// Write a register
static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return i2c_master_transmit(
        mpu,
        data,
        2,
        1000
    );
}

// Read multiple consecutive registers
static esp_err_t read_regs(
    uint8_t reg,
    uint8_t *data,
    size_t len)
{
    return i2c_master_transmit_receive(
        mpu,
        &reg,
        1,
        data,
        len,
        1000
    );
}

// ==========================================
// 5. GYROSCOPE CALIBRATION
// ==========================================

static bool calibrate_gyro(void)
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

        printf(
            "Gyro calibration failed: %d samples\n",
            valid
        );

        return false;
    }

    for (int i = 0; i < 3; i++) {
        gyro_offset[i] = sum[i] / valid;
    }

    printf(
        "Calibration complete: %.3f %.3f %.3f\n",
        gyro_offset[0],
        gyro_offset[1],
        gyro_offset[2]
    );

    return true;
}

// ==========================================
// 6. MADGWICK FILTER
// ==========================================

// 6-axis Madgwick filter
// Gyroscope inputs must be in radians/second.

static void madgwick_update(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt)
{
    float norm;
    float s0, s1, s2, s3;

    // Quaternion derivative from gyroscope
    float qDot0 = 0.5f * (
        -q1 * gx - q2 * gy - q3 * gz
    );

    float qDot1 = 0.5f * (
         q0 * gx + q2 * gz - q3 * gy
    );

    float qDot2 = 0.5f * (
         q0 * gy - q1 * gz + q3 * gx
    );

    float qDot3 = 0.5f * (
         q0 * gz + q1 * gy - q2 * gx
    );

    // Normalize accelerometer measurement
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

        // Gradient descent corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax
           + _4q0 * q1q1 - _2q1 * ay;

        s1 = _4q1 * q3q3 - _2q3 * ax
           + 4.0f * q0q0 * q1 - _2q0 * ay
           - _4q1 + _8q1 * q1q1
           + _8q1 * q2q2 + _4q1 * az;

        s2 = 4.0f * q0q0 * q2 + _2q0 * ax
           + _4q2 * q3q3 - _2q3 * ay
           - _4q2 + _8q2 * q1q1
           + _8q2 * q2q2 + _4q2 * az;

        s3 = 4.0f * q1q1 * q3 - _2q1 * ax
           + 4.0f * q2q2 * q3 - _2q2 * ay;

        norm = sqrtf(
            s0*s0 + s1*s1 + s2*s2 + s3*s3
        );

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

    // Integrate quaternion
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // Normalize quaternion
    norm = sqrtf(
        q0*q0 + q1*q1 + q2*q2 + q3*q3
    );

    if (norm > 0.000001f) {

        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }
}

// ==========================================
// 7. LINEAR ACCELERATION CALCULATION
// ==========================================

// Input acceleration: g
// Output acceleration: m/s^2

static void calculate_linear_acceleration(
    float ax, float ay, float az)
{
    // Estimated gravity in sensor coordinates
    float gravity_x = 2.0f * (
        q1 * q3 - q0 * q2
    );

    float gravity_y = 2.0f * (
        q0 * q1 + q2 * q3
    );

    float gravity_z = 1.0f - 2.0f * (
        q1 * q1 + q2 * q2
    );

    // Remove gravity and convert to m/s^2
    linear_accel_x = (ax - gravity_x) * GRAVITY;

    linear_accel_y = (ay - gravity_y) * GRAVITY;

    linear_accel_z = (az - gravity_z) * GRAVITY;
}

// ==========================================
// 8. ESP-NOW SEND CALLBACK
// ==========================================

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)

static void espnow_send_callback(
    const wifi_tx_info_t *tx_info,
    esp_now_send_status_t status)
{
    (void)tx_info;

    if (status == ESP_NOW_SEND_SUCCESS) {
        tx_success_count++;
    }
    else {
        tx_failure_count++;
    }

    if (send_sem != NULL) {
        xSemaphoreGive(send_sem);
    }
}

#else

static void espnow_send_callback(
    const uint8_t *mac_addr,
    esp_now_send_status_t status)
{
    (void)mac_addr;

    if (status == ESP_NOW_SEND_SUCCESS) {
        tx_success_count++;
    }
    else {
        tx_failure_count++;
    }

    if (send_sem != NULL) {
        xSemaphoreGive(send_sem);
    }
}

#endif

// ==========================================
// 9. ESP-NOW INITIALIZATION
// ==========================================

static void espnow_init(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();

    if (
        ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND
    ) {

        ESP_ERROR_CHECK(nvs_flash_erase());

        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else {
        ESP_ERROR_CHECK(ret);
    }

    // Initialize network interface
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create Wi-Fi station interface
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi driver
    wifi_init_config_t wifi_config = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&wifi_config));

    // Set station mode
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    

    // Start Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());
// Set station mode
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(34)); //////
    // Set Wi-Fi channel
    ESP_ERROR_CHECK(
        esp_wifi_set_channel(
            ESPNOW_WIFI_CHANNEL,
            WIFI_SECOND_CHAN_NONE
        )
    );

    // Disable Wi-Fi power saving
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Create semaphore BEFORE registering callback
    send_sem = xSemaphoreCreateBinary();

    if (send_sem == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }

    // Allow first transmission
    xSemaphoreGive(send_sem);

    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());

    // Register send callback
    ESP_ERROR_CHECK(
        esp_now_register_send_cb(espnow_send_callback)
    );

    // Configure receiver as ESP-NOW peer
    esp_now_peer_info_t peer = {0};

    for (int i = 0; i < 6; i++) {
        peer.peer_addr[i] = receiver_mac[i];
    }

    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    // Add receiver to peer list
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    printf("ESP-NOW initialized successfully\n");
}

// ==========================================
// 10. PREPARE AND SEND SENSOR PACKET
// ==========================================

static void send_sensor_packet(void)
{
    // Check whether previous transmission completed
    if (xSemaphoreTake(send_sem, 0) == pdTRUE) {

        // Packet identification
        tx_packet.packet_type = PACKET_TYPE_SENSOR;

        // Sequence number
        tx_packet.sequence_number = sequence_number++;

        // Copy quaternion
        tx_packet.q0 = q0;
        tx_packet.q1 = q1;
        tx_packet.q2 = q2;
        tx_packet.q3 = q3;

        // Copy linear acceleration
        tx_packet.accel_x = linear_accel_x;
        tx_packet.accel_y = linear_accel_y;
        tx_packet.accel_z = linear_accel_z;

        // Transmit packet
        esp_err_t result = esp_now_send(
            receiver_mac,
            (const uint8_t *)&tx_packet,
            sizeof(tx_packet)
        );

        if (result != ESP_OK) {

            printf(
                "ESP-NOW send error: %s\n",
                esp_err_to_name(result)
            );

            // Allow another transmission attempt
            xSemaphoreGive(send_sem);
        }
    }
}

// ==========================================
// 11. MAIN APPLICATION
// ==========================================

void app_main(void)
{
    // ======================================
    // INITIALIZE ESP-NOW
    // ======================================

    espnow_init();

    // ======================================
    // INITIALIZE I2C
    // ======================================

    i2c_master_bus_handle_t bus;

    i2c_master_bus_config_t bus_config = {

        .i2c_port = I2C_NUM_0,

        .sda_io_num = SDA_PIN,

        .scl_io_num = SCL_PIN,

        .clk_source = I2C_CLK_SRC_DEFAULT,

        .glitch_ignore_cnt = 7,

        .flags.enable_internal_pullup = true,
    };

    ESP_ERROR_CHECK(
        i2c_new_master_bus(&bus_config, &bus)
    );

    // Configure MPU6050 device
    i2c_device_config_t dev_config = {

        .dev_addr_length = I2C_ADDR_BIT_LEN_7,

        .device_address = MPU_ADDR,

        .scl_speed_hz = 100000,
    };

    ESP_ERROR_CHECK(
        i2c_master_bus_add_device(
            bus,
            &dev_config,
            &mpu
        )
    );

    // ======================================
    // CHECK MPU6050 IDENTITY
    // ======================================

    uint8_t who_am_i = 0;

    ESP_ERROR_CHECK(
        read_regs(0x75, &who_am_i, 1)
    );

    if (who_am_i != 0x68) {

        printf(
            "MPU6050 not detected! WHO_AM_I = 0x%02X\n",
            who_am_i
        );

        return;
    }

    printf("MPU6050 detected successfully.\n");

    // ======================================
    // CONFIGURE MPU6050
    // ======================================

    // Wake MPU6050
    ESP_ERROR_CHECK(write_reg(0x6B, 0x00));

    // Accelerometer: +/- 2g
    ESP_ERROR_CHECK(write_reg(0x1C, 0x00));

    // Gyroscope: +/- 250 degrees/second
    ESP_ERROR_CHECK(write_reg(0x1B, 0x00));

    // Digital low-pass filter
    ESP_ERROR_CHECK(write_reg(0x1A, 0x03));

    vTaskDelay(pdMS_TO_TICKS(100));

    // ======================================
    // CALIBRATE GYROSCOPE
    // ======================================

    if (!calibrate_gyro()) {

        printf(
            "Stopping: gyro calibration unsuccessful.\n"
        );

        return;
    }

    printf("MPU6050 orientation tracking started\n");

    // ======================================
    // SENSOR PROCESSING LOOP
    // ======================================

    int64_t last_time = esp_timer_get_time();

    while (1) {

        uint8_t data[14];

        // Read accelerometer, temperature and gyroscope
        if (read_regs(0x3B, data, 14) == ESP_OK) {

            // Calculate elapsed time
            int64_t now = esp_timer_get_time();

            float dt = (now - last_time) / 1000000.0f;

            last_time = now;

            if (dt <= 0.0f || dt > 0.1f) {
                dt = 0.02f;
            }

            // ==================================
            // ACCELEROMETER DATA
            // ==================================

            // Convert raw values into g
            float ax = be16(data[0], data[1]) / 16384.0f;

            float ay = be16(data[2], data[3]) / 16384.0f;

            float az = be16(data[4], data[5]) / 16384.0f;

            // ==================================
            // GYROSCOPE DATA
            // ==================================

            // Convert raw values into rad/s
            float gx = (
                be16(data[8], data[9]) / 131.0f
                - gyro_offset[0]
            ) * DEG_TO_RAD;

            float gy = (
                be16(data[10], data[11]) / 131.0f
                - gyro_offset[1]
            ) * DEG_TO_RAD;

            float gz = (
                be16(data[12], data[13]) / 131.0f
                - gyro_offset[2]
            ) * DEG_TO_RAD;

            // ==================================
            // MADGWICK ORIENTATION UPDATE
            // ==================================

            madgwick_update(
                gx, gy, gz,
                ax, ay, az,
                dt
            );

            // ==================================
            // LINEAR ACCELERATION
            // ==================================

            calculate_linear_acceleration(
                ax, ay, az
            );

            // ==================================
            // ESP-NOW TRANSMISSION
            // ==================================

            send_sensor_packet();

            // ==================================
            // USB SERIAL OUTPUT
            // ==================================

            printf(
                "Q,%.6f,%.6f,%.6f,%.6f,"
                "A,%.3f,%.3f,%.3f\n",

                q0, q1, q2, q3,

                linear_accel_x,
                linear_accel_y,
                linear_accel_z
            );
        }

        // ==================================
        // TRANSMISSION STATISTICS
        // ==================================

        static uint32_t last_printed_success = 0;
        static uint32_t last_printed_failure = 0;

        if (
            tx_success_count != last_printed_success ||
            tx_failure_count != last_printed_failure
        ) {

            printf(
                "TX Success: %lu | TX Failure: %lu\n",

                (unsigned long)tx_success_count,

                (unsigned long)tx_failure_count
            );

            last_printed_success = tx_success_count;
            last_printed_failure = tx_failure_count;
        }

        // Approximate 50 Hz loop
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
