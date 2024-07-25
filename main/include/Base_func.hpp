#ifndef MICROMOUSE_HPP
#define MICROMOUSE_HPP

#include <iostream>
#include <vector>
#include <memory>
#include "structs.hpp"
#include "ADS7066.hpp"
#include "MA730.hpp"
#include "Buzzer.hpp"
#include "MPU6500.hpp"
#include "PCA9632.hpp"
#include "Motor.hpp"


#define Interface struct

Interface Micromouse
{
    virtual void ptr_by_sensor(t_sens_data *_sens) = 0;
    virtual void ptr_by_motion(t_mouse_motion_val *_val) = 0;
    virtual void ptr_by_control(t_control *_control) = 0;
    virtual void ptr_by_map(t_map *_map) = 0;
    virtual void set_device(ADS7066 &_adc, MA730 &_encR, MA730 &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) = 0;
};


#endif // MICROMOUSE_HPP