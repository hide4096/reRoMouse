#include "PCA9632.hpp"

PCA9632::PCA9632(i2c_port_t port, uint8_t adrs){
    // デバイスの設定
    _port = port;
    _adrs = adrs;
    write(0x00,0x01);
    _init = true;

    ESP_LOGI("PCA9632", "Initialized");
}
PCA9632::~PCA9632(){}

uint8_t PCA9632::read(uint8_t reg){
    esp_err_t err;
    uint8_t data;

    err = i2c_master_write_read_device(_port,_adrs,&reg,1,&data,1,10/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(err);
    return data;
}

esp_err_t PCA9632::write(uint8_t reg,uint8_t data){
    esp_err_t err;
    uint8_t send[2] = {reg,data};

    err = i2c_master_write_to_device(_port,_adrs,send,2,10/portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(err);
    return err;
}


void PCA9632::set(uint8_t led){
    uint8_t cmd = 0;
    for(int i=0;i<4;i++){
        if(led & (0b1 << i)){
            cmd |= 0b01 << (i*2);
        }
    }
    write(0x08,cmd);
}

void PCA9632::blink(){
    uint8_t led = 0b1;
    while(1){
        set(led);
        led = led << 1;
        if(led == 0b10000){
            led = 0b1;
        }

        printf("led : %d\n",led);
        vTaskDelay(50/portTICK_PERIOD_MS);
    }
}

