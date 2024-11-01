
#include "include/UI/fast.hpp"

void Fast::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Fast::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Fast::ptr_by_control(t_control *_control) { control = _control; }

void Fast::ptr_by_map(t_map *_map) { map = _map; }

void Fast::set_device_driver(std::shared_ptr<t_drivers> driver) {}

void Fast::ref_by_motion(Adachi &_adachi) { motion = _adachi; } // ここでのポインタ渡しを忘れていて、InitMazeが正しく行えず、map_readがオーバーフローした

void Fast::main_task()
{
    //motion.InitMaze();
    //*map = map_read();

    val->current.rad = 0.0;
    map->pos.x = 0;
    map->pos.y = 0;
    map->pos.dir = NORTH;
    map->flag = SEARCH;
    control->log_flag = TRUE;
    
    map->search_count_flag = TRUE;
    map->search_time = 0;
    
    motion.search_adachi2(map->GOAL_X,map->GOAL_Y);
    control->log_flag = FALSE;
    
    //map->flag = ALL_SEARCH;
    //motion.search_adachi(0,0);
    map_write(map);
    std::cout << "Fast" << std::endl;
}

void Fast2::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Fast2::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Fast2::ptr_by_control(t_control *_control) { control = _control; }

void Fast2::ptr_by_map(t_map *_map) { map = _map; }

void Fast2::set_device_driver(std::shared_ptr<t_drivers> driver) {}

void Fast2::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Fast2::main_task()
{
    //motion.InitMaze();
    //*map = map_read();

    val->current.rad = 0.0;
    map->pos.x = 0;
    map->pos.y = 0;
    map->pos.dir = NORTH;
    map->flag = SEARCH;
    control->log_flag = TRUE;

    map->search_count_flag = TRUE;
    map->search_time = 0;

    val->max.acc = 2.0;
    val->max.vel = 0.4;
    val->end.vel = 0.4;

    val->max.ang_acc = M_PI*12.0;
    val->max.ang_vel = M_PI*4.0;
    
    motion.search_adachi2(map->GOAL_X,map->GOAL_Y);
    control->log_flag = FALSE;
    
    //map->flag = ALL_SEARCH;
    //motion.search_adachi(0,0);
    map_write(map);
    std::cout << "Fast2" << std::endl;
}

void Fast3::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Fast3::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Fast3::ptr_by_control(t_control *_control) { control = _control; }

void Fast3::ptr_by_map(t_map *_map) { map = _map; }

void Fast3::set_device_driver(std::shared_ptr<t_drivers> driver) {}

void Fast3::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Fast3::main_task()
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

    val->max.acc = 2.0;
    val->max.vel = 0.4;
    val->end.vel = 0.4;

    val->max.ang_acc = M_PI*12.0;
    val->max.ang_vel = M_PI*4.0;
    
    motion.search_adachi2(map->GOAL_X,map->GOAL_Y);
    control->log_flag = FALSE;
    
    //map->flag = ALL_SEARCH;
    //motion.search_adachi(0,0);
    map_write(map);
    std::cout << "Fast3" << std::endl;
}

void Fast4::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Fast4::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Fast4::ptr_by_control(t_control *_control) { control = _control; }

void Fast4::ptr_by_map(t_map *_map) { map = _map; }

void Fast4::set_device_driver(std::shared_ptr<t_drivers> driver) {}

void Fast4::ref_by_motion(Adachi &_adachi) { motion = _adachi; }

void Fast4::main_task()
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

    val->max.acc = 2.0;
    val->max.vel = 0.5;
    val->end.vel = 0.5;

    val->max.ang_acc = M_PI*12.0;
    val->max.ang_vel = M_PI*4.0;
    
    motion.search_adachi2(map->GOAL_X,map->GOAL_Y);
    control->log_flag = FALSE;
    
    //map->flag = ALL_SEARCH;
    //motion.search_adachi(0,0);
    map_write(map);
    
    std::cout << "Fast4" << std::endl;
}
