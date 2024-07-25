#include "include/UI/search.hpp"

void Search::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Search::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Search::ptr_by_control(t_control *_control) { control = _control; }

void Search::ptr_by_map(t_map *_map) { map = _map; }

void Search::set_device(ADS7066 &_adc, MA730 &_encR, MA730 &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void Search::ref_by_motion(Adachi &_adachi) { motion = _adachi;}

void Search::main_task()
{
    val->current.rad = 0.0;
    map->pos.x = 0;
    map->pos.y = 0;
    map->pos.dir = NORTH;
    map->flag = SEARCH;
    control->log_flag = TRUE;
    motion.InitMaze();
    map->search_count_flag = TRUE;
    map->search_time = 0;
    motion.search_adachi(map->GOAL_X,map->GOAL_Y);
    control->log_flag = FALSE;
    map_write(map);
    
    //std::cout << "Search" << std::endl;
}

void All_Search::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void All_Search::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void All_Search::ptr_by_control(t_control *_control) { control = _control; }

void All_Search::ptr_by_map(t_map *_map) { map = _map; }

void All_Search::set_device(ADS7066 &_adc, MA730 &_encR, MA730 &_encL, BUZZER &_buz, MPU6500 &_imu, PCA9632 &_led, Motor &_mot){}

void All_Search::ref_by_motion(Adachi &_adachi) { motion = _adachi;}

void All_Search::main_task()
{
    val->current.rad = 0.0;
    map->pos.x = 0;
    map->pos.y = 0;
    map->pos.dir = NORTH;
    map->flag = SEARCH;
    control->log_flag = TRUE;
    motion.InitMaze();
    map->search_count_flag = TRUE;
    map->search_time = 0;
    motion.search_adachi(map->GOAL_X,map->GOAL_Y);
    control->log_flag = FALSE;
    
    map->flag = ALL_SEARCH;
    motion.search_adachi(0,0);
    map_write(map);
    //std::cout << "All_Search" << std::endl;
}
