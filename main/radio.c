#include "radio.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_idf_version.h"
#include "nvs_flash.h"
#include "esp_err.h"
static SemaphoreHandle_t send_sem;
static volatile uint32_t tx_success_count,tx_failure_count;
static uint32_t sequence_number;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,5,0)
static void sent(const wifi_tx_info_t *info,esp_now_send_status_t status){(void)info;
#else
static void sent(const uint8_t *mac,esp_now_send_status_t status){(void)mac;
#endif
    if(status==ESP_NOW_SEND_SUCCESS)tx_success_count++;else tx_failure_count++;
    if(send_sem)xSemaphoreGive(send_sem);
}
void radio_init(void){
    esp_err_t e=nvs_flash_init();
    if(e==ESP_ERR_NVS_NO_FREE_PAGES||e==ESP_ERR_NVS_NEW_VERSION_FOUND){ESP_ERROR_CHECK(nvs_flash_erase());e=nvs_flash_init();}
    ESP_ERROR_CHECK(e);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t w=WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&w));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(34));
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL,WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    send_sem=xSemaphoreCreateBinary();
    if(!send_sem)ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    xSemaphoreGive(send_sem);
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(sent));
    esp_now_peer_info_t peer={0};
    memcpy(peer.peer_addr,receiver_mac,6);
    peer.channel=ESPNOW_WIFI_CHANNEL;peer.ifidx=WIFI_IF_STA;peer.encrypt=false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
    printf("ESP-NOW ready\n");
}
void radio_send(const fusion_t sensors[3],bool button_pressed){
    if(xSemaphoreTake(send_sem,0)!=pdTRUE)return;
    multi_sensor_packet_t p={0};
    p.packet_type=PACKET_TYPE_MULTI_SENSOR;
    p.button_pressed=button_pressed?1:0;
    p.sensor_count=3;
    p.sequence_number=sequence_number++;
    for(int i=0;i<3;i++){
        p.sensors[i]=(sensor_values_t){sensors[i].q0,sensors[i].q1,sensors[i].q2,sensors[i].q3,
            sensors[i].linear_accel_x,sensors[i].linear_accel_y,sensors[i].linear_accel_z};
    }
    esp_err_t e=esp_now_send(receiver_mac,(const uint8_t *)&p,sizeof(p));
    if(e!=ESP_OK){printf("ESP-NOW send error: %s\n",esp_err_to_name(e));xSemaphoreGive(send_sem);}
}
void radio_stats(void){
    static uint32_t old_ok=0,old_fail=0;
    if(old_ok!=tx_success_count||old_fail!=tx_failure_count){
        printf("TX Success: %lu | TX Failure: %lu\n",(unsigned long)tx_success_count,(unsigned long)tx_failure_count);
        old_ok=tx_success_count;old_fail=tx_failure_count;
    }
}
