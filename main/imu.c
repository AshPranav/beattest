#include "imu.h"
#include "config.h"
#include "esp_check.h"
#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static spi_device_handle_t bmi[2];
static i2c_master_dev_handle_t mpu;
static int16_t le16(const uint8_t *p){return (int16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8));}
static int16_t be16(const uint8_t *p){return (int16_t)(((uint16_t)p[0]<<8)|p[1]);}

static esp_err_t bmi_read(int i,uint8_t reg,uint8_t *out,size_t n){
    uint8_t tx[16]={0},rx[16]={0};
    if(n+1>sizeof(tx))return ESP_ERR_INVALID_SIZE;
    tx[0]=reg|0x80;
    spi_transaction_t t={.length=(n+1)*8,.tx_buffer=tx,.rx_buffer=rx};
    esp_err_t e=spi_device_transmit(bmi[i],&t);
    if(e==ESP_OK)memcpy(out,rx+1,n);
    return e;
}
static esp_err_t bmi_write(int i,uint8_t reg,uint8_t val){
    uint8_t tx[2]={reg&0x7f,val};
    spi_transaction_t t={.length=16,.tx_buffer=tx};
    return spi_device_transmit(bmi[i],&t);
}
static esp_err_t mpu_write(uint8_t reg,uint8_t val){
    uint8_t tx[2]={reg,val};return i2c_master_transmit(mpu,tx,2,1000);
}
static esp_err_t mpu_read(uint8_t reg,uint8_t *buf,size_t n){
    return i2c_master_transmit_receive(mpu,&reg,1,buf,n,1000);
}

esp_err_t imu_init(void){
    spi_bus_config_t bus={.mosi_io_num=BMI_MOSI,.miso_io_num=BMI_MISO,
        .sclk_io_num=BMI_SCLK,.quadwp_io_num=-1,.quadhd_io_num=-1,.max_transfer_sz=32};
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST,&bus,SPI_DMA_DISABLED),"IMU","SPI bus");
    int cs[2]={BMI1_CS,BMI2_CS};
    for(int i=0;i<2;i++){
        spi_device_interface_config_t dev={.clock_speed_hz=1000000,.mode=0,
            .spics_io_num=cs[i],.queue_size=1};
        ESP_RETURN_ON_ERROR(spi_bus_add_device(SPI2_HOST,&dev,&bmi[i]),"IMU","SPI device");
        uint8_t id=0;
        (void)bmi_read(i,0x00,&id,1); // Switch BMI160 into SPI mode
        vTaskDelay(pdMS_TO_TICKS(10));
        ESP_RETURN_ON_ERROR(bmi_read(i,0x00,&id,1),"IMU","BMI ID read");
        printf("BMI160 %d CHIP_ID=0x%02X\n",i+1,id);
        if(id!=0xD1)return ESP_ERR_NOT_FOUND;
        ESP_RETURN_ON_ERROR(bmi_write(i,0x7E,0x11),"IMU","BMI accel normal");
        vTaskDelay(pdMS_TO_TICKS(5));
        ESP_RETURN_ON_ERROR(bmi_write(i,0x7E,0x15),"IMU","BMI gyro normal");
        vTaskDelay(pdMS_TO_TICKS(85));
        ESP_RETURN_ON_ERROR(bmi_write(i,0x41,0x03),"IMU","BMI accel 2g");
        ESP_RETURN_ON_ERROR(bmi_write(i,0x43,0x03),"IMU","BMI gyro 250dps");
        ESP_RETURN_ON_ERROR(bmi_write(i,0x40,0x28),"IMU","BMI accel ODR 100Hz");
        ESP_RETURN_ON_ERROR(bmi_write(i,0x42,0x28),"IMU","BMI gyro ODR 100Hz");
    }

    // Original fixed-address MPU6050 initialization (no address scanner).
    i2c_master_bus_handle_t ibus;
    i2c_master_bus_config_t cfg={
        .i2c_port=I2C_NUM_0,.sda_io_num=MPU_SDA,.scl_io_num=MPU_SCL,
        .clk_source=I2C_CLK_SRC_DEFAULT,.glitch_ignore_cnt=7,
        .flags.enable_internal_pullup=true
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&cfg,&ibus),"IMU","I2C bus");
    i2c_device_config_t dev={.dev_addr_length=I2C_ADDR_BIT_LEN_7,
        .device_address=MPU_ADDR,.scl_speed_hz=100000};
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(ibus,&dev,&mpu),"IMU","MPU add");
    vTaskDelay(pdMS_TO_TICKS(50));
    uint8_t id=0;
    ESP_RETURN_ON_ERROR(mpu_read(0x75,&id,1),"IMU","MPU ID read");
    printf("MPU6050 WHO_AM_I=0x%02X\n",id);
    if(id!=0x68)return ESP_ERR_NOT_FOUND;
    ESP_RETURN_ON_ERROR(mpu_write(0x6B,0x00),"IMU","MPU wake");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(mpu_write(0x1C,0x00),"IMU","MPU accel 2g");
    ESP_RETURN_ON_ERROR(mpu_write(0x1B,0x00),"IMU","MPU gyro 250dps");
    ESP_RETURN_ON_ERROR(mpu_write(0x1A,0x03),"IMU","MPU LPF");
    printf("MPU6050 initialized successfully\n");
    return ESP_OK;
}

bool imu_read(int index,imu_sample_t *s){
    if(!s||index<0||index>2)return false;
    if(index<2){
        uint8_t b[12];
        if(bmi_read(index,0x0C,b,sizeof(b))!=ESP_OK)return false;
        s->gx=le16(b+0)/131.2f;s->gy=le16(b+2)/131.2f;s->gz=le16(b+4)/131.2f;
        s->ax=le16(b+6)/16384.0f;s->ay=le16(b+8)/16384.0f;s->az=le16(b+10)/16384.0f;
    }else{
        uint8_t b[14];
        if(mpu_read(0x3B,b,sizeof(b))!=ESP_OK)return false;
        s->ax=be16(b+0)/16384.0f;s->ay=be16(b+2)/16384.0f;s->az=be16(b+4)/16384.0f;
        s->gx=be16(b+8)/131.0f;s->gy=be16(b+10)/131.0f;s->gz=be16(b+12)/131.0f;
    }
    return true;
}
