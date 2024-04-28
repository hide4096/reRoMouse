#ifndef PCA9632_HPP
#define PCA9632_HPP

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include <stdexcept>
#include <iostream>
#include <math.h>
#include <cstring>

class PCA9632
{
public:
    PCA9632(i2c_port_t port, uint8_t adrs);
    ~PCA9632();
    void blink();
    void set(uint8_t led);

private:
    bool _init = false;
    
    
    i2c_port_t _port;
    uint8_t _adrs;
    uint8_t read(uint8_t reg);
    esp_err_t write(uint8_t reg, uint8_t data);
    float speed;
    uint8_t mode;
};

#endif // PCA9632_HPP