#ifndef MCP3464_HPP
#define MCP3464_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <cstring>
#include <iostream>


class MCP3464
{
public:
    MCP3464(spi_host_device_t bus, gpio_num_t cs);
    ~MCP3464();

private:
    spi_device_handle_t _spi;
    spi_bus_config_t bus_adc;
    esp_err_t ret;
    esp_err_t err;
    spi_device_interface_config_t dev_adc;
    uint8_t read(uint8_t reg);
    uint16_t read16(uint8_t reg);
    void write(void* pData, uint8_t size);
};
#endif
