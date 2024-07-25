#ifndef ADS7066_HPP
#define ADS7066_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <cstring>
#include <iostream>

class ADS7066
{
public:
    ADS7066(spi_host_device_t, gpio_num_t);
    ~ADS7066();
    uint16_t readOneShot(int8_t);
    uint16_t readOnTheFly(int8_t channelID = -1);

private:
    spi_device_handle_t _spi;
    spi_bus_config_t bus_adc;
    esp_err_t err;
    spi_device_interface_config_t dev_adc;
    uint8_t SEQ_MODE;

    void writeRegister(uint8_t adrs, uint8_t data);
    uint8_t readRegister(uint8_t adrs);
    esp_err_t changeSEQ_MODE(uint8_t mode);
};

#endif