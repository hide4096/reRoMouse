#ifndef MOTION_HPP
#define MOTION_HPP

#include <iostream>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>    // freertos以下のファイルをインクルードしたい場合、必ず先にFreeRTOS.hをインクルードする
#include "../Base_func.hpp"
#include "files.hpp"

class Motion : public Micromouse
{
    public:
        Motion();
        ~Motion();
        void ptr_by_sensor(t_sens_data *sens) override;
        void ptr_by_motion(t_mouse_motion_val *val) override;
        void ptr_by_control(t_control *control) override;
        void ptr_by_map(t_map *map) override;
        void set_device_driver(std::shared_ptr<t_drivers> driver) override;
        void GetSemphrHandle(SemaphoreHandle_t *_on_logging);
        void run();
        void run2();
        void run_half();
        void turn_left();
        void turn_right();
        void turn_half();
        void stop();
        void stop2();
        void back();
        void slalom();
        void check_enkaigei();
        void turn_left_2();
        void turn_right_2();
        void wall_check();
        void adjust_pid(const char* gain, float *pid, float step, uint8_t mode_num);
        void set_pid_gain();
        void adjust_wall_threshold(const char* threshold, uint16_t *th_value, uint8_t step, uint8_t mode_num);
        void set_wall_threshold();
        void offset();
        void offset2();
        void calibrate_wall_th();
        void fast_straight(uint8_t straight_count);
        
        
    protected:
        t_sens_data *sens;
        t_mouse_motion_val *val;
        t_control *control;
        t_map *map;
        SemaphoreHandle_t *on_logging;

        std::shared_ptr<NeoPixel> np;
        std::shared_ptr<MPU6500> imu;
        std::shared_ptr<PCA9632> led;
        std::shared_ptr<BUZZER> bz;
        std::shared_ptr<Motor> mot;
        std::shared_ptr<MA730> encL;
        std::shared_ptr<MA730> encR;
        std::shared_ptr<ADS7066> adc;

    private:
        uint8_t len_count = 0;
        float local_rad = 0.0;
        

};

#endif // MOTION_HPP