#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "config.h"
#include "imu.h"
#include "fusion.h"
#include "radio.h"
static fusion_t states[3];
static bool calibrate_all(void){
    printf("Keep ALL THREE sensors still for 3 seconds...\n");
    float sums[3][3]={0};int counts[3]={0};
    for(int n=0;n<300;n++){
        for(int i=0;i<3;i++){
            imu_sample_t s;
            if(imu_read(i,&s)){
                sums[i][0]+=s.gx;sums[i][1]+=s.gy;sums[i][2]+=s.gz;counts[i]++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    for(int i=0;i<3;i++){
        if(counts[i]<250){printf("Calibration failed sensor %d: %d samples\n",i,counts[i]);return false;}
        for(int a=0;a<3;a++)states[i].gyro_offset[a]=sums[i][a]/counts[i];
        printf("Sensor %d offsets %.3f %.3f %.3f\n",i,
            states[i].gyro_offset[0],states[i].gyro_offset[1],states[i].gyro_offset[2]);
    }
    return true;
}
void app_main(void){
    radio_init();
    gpio_config_t button={.pin_bit_mask=1ULL<<BUTTON_GPIO,.mode=GPIO_MODE_INPUT,
        .pull_up_en=GPIO_PULLUP_ENABLE,.pull_down_en=GPIO_PULLDOWN_DISABLE,.intr_type=GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&button));
    ESP_ERROR_CHECK(imu_init());
    for(int i=0;i<3;i++)fusion_reset(&states[i]);
    if(!calibrate_all())return;
    printf("Tracking 2x BMI160 + MPU6050 at approximately 50 Hz\n");
    int64_t last=esp_timer_get_time();
    while(1){
        imu_sample_t samples[3];bool valid=true;
        for(int i=0;i<3;i++)if(!imu_read(i,&samples[i]))valid=false;
        int64_t now=esp_timer_get_time();
        float dt=(now-last)/1000000.0f;last=now;
        if(valid){
            for(int i=0;i<3;i++)fusion_update(&states[i],&samples[i],dt);
            bool pressed=(gpio_get_level(BUTTON_GPIO)==0);
            radio_send(states,pressed);
            for(int i=0;i<3;i++)printf("S%d,Q,%.6f,%.6f,%.6f,%.6f,A,%.3f,%.3f,%.3f%s",
                i,states[i].q0,states[i].q1,states[i].q2,states[i].q3,
                states[i].linear_accel_x,states[i].linear_accel_y,states[i].linear_accel_z,
                i==2?"\n":" | ");
            if(pressed)printf("BUTTON,PRESSED\n");
        }else printf("Sensor read error; skipped frame\n");
        radio_stats();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
