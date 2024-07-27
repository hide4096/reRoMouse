#ifndef BASE_FUNC_HPP
#define BASE_FUNC_HPP

#include <iostream>
#include <vector>
#include <memory>
#include "structs.hpp"
#include "drivers.hpp"


#define Interface struct

Interface Micromouse
{
    virtual void ptr_by_sensor(t_sens_data *_sens) = 0;
    virtual void ptr_by_motion(t_mouse_motion_val *_val) = 0;
    virtual void ptr_by_control(t_control *_control) = 0;
    virtual void ptr_by_map(t_map *_map) = 0;
    virtual void set_device_driver(std::shared_ptr<t_drivers> driver) = 0;
};


#endif // MICROMOUSE_HPP