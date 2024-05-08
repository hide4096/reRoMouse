#ifndef SENSOR_HPP
#define SENSOR_HPP

#include <iostream>
#include "PCA9632.hpp"
#include "MPU6500.hpp"
#include "MCP3464.hpp"
#include "MA730.hpp"
#include "structs.hpp"

/*
    < センサドライバのインターフェースクラス >
    センサからデータを取得するドライバを作成する場合は、このクラスを継承
*/

#define _interface struct

_interface Sensor 
{
    virtual void GetData(t_sens_data *_sens) = 0;
};

#endif // SENSOR_HPP