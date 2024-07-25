#ifndef LOG_HPP
#define LOG_HPP

#include <fstream>
#include <iostream>
#include <string>
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_flash.h"
#include "spi_flash_mmap.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "UI.hpp"

class Log : public UI
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void set_device(ADS7066 &_adc, MA730 &_encR, MA730 &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void main_task() override;
        void ref_by_motion(Adachi &_adachi) override;
        void log_print();
    private:
        t_sens_data *sens;    
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        Adachi motion;
};

class Log1 : public UI
{
    public:
        void ptr_by_sensor(t_sens_data *_sens) override;
        void ptr_by_motion(t_mouse_motion_val *_val) override;
        void ptr_by_control(t_control *_control) override;
        void ptr_by_map(t_map *_map) override;
        void set_device(ADS7066 &_adc, MA730 &_encR, MA730 &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot) override;
        void main_task() override;
        void ref_by_motion(Adachi &_adachi) override;
        void log_print();
        void map_print();
    private:
        t_sens_data *sens;    
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        Adachi motion;
};

#endif // LOG_HPP