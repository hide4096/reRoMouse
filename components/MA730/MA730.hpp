#ifndef MA730_HPP
#define MA730_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <cstring>

#define READ_COMMAND 0b010
#define WRITE_COMMAND 0b100
#define Rotation_direction 0b01001 // 回転方向逆転のアドレス　ここの7bit目を1にすると回転方向が逆になる
#define RESOLUTION_MAX 16384
#define RESORUTION_HALF 8192



class MA730
{
    public:
    MA730(spi_host_device_t bus, gpio_num_t cs);
    ~MA730();

    //void GetData(t_sens_data *_sens) override;
    uint16_t readAngle();
    void ShowAngle();

private:
    uint16_t read();
    uint16_t OperateRegisters(const uint8_t command, const uint8_t address, const uint8_t data);
    uint8_t ReadRegister(const uint8_t address, const uint8_t data);
    uint8_t WriteRegister(const uint8_t address, const uint8_t data);
    esp_err_t ret;
    esp_err_t err;
    spi_transaction_t cmd;
    spi_device_handle_t _spi;
    spi_bus_config_t bus_enc;
    spi_device_interface_config_t dev_enc;
    //gpio_num_t _cs;
    
    //t_sens_data *sens;
    
};

#endif