#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "NeoPixel.hpp"
#include "MPU6500.hpp"
#include "PCA9632.hpp"
#include "Buzzer.hpp"
#include "Motor.hpp"
#include "MA730.hpp"
#include "ADS7066.hpp"

struct pid_gain_t
{
    float kp;
    float ki;
    float kd;
};

struct trace_gain_t
{
    float kx1;
    float kx2;
    float ky1;
    float ky2;
};

enum Mode
{
    GENERAL,
    TRAJECT
};

enum Direction
{
    NORTH,
    EAST,
    SOUTH,
    WEST
};

enum Turn
{
    LEFT = -1,
    RIGHT = 1,
    STRAIGHT = 0,
    BACK = 2
};

struct orbit_t
{
    float x;
    float y;
};

struct orbitBase_t
{
    orbit_t *orbit;
    uint16_t size;
    Turn turn;
};

struct routemap_t
{
    orbitBase_t *path;
    bool isReverse;
    uint8_t divide;
};

class driver_t
{
public:
    MA730 *encL;
    MA730 *encR;
    MPU6500 *imu;
    PCA9632 *led;
    ADS7066 *adc;
    BUZZER *buzzer;
    NeoPixel *np;
    Motor *motor;

    float tgt_vel;
    float tgt_angvel;

    Mode mode = GENERAL;
    uint64_t tick;
    bool enable;

    pid_gain_t vel_gain;
    pid_gain_t angvel_gain;
    trace_gain_t trace_gain = {0.1, 0.1, 0.1, 0.1};

    float ff_gain = 100;
    float accelMax = 3.0;          // m/s/s
    float angaccelMax = 20 * M_PI; // rad/s/s

    orbitBase_t slalom;
    orbitBase_t start;
    orbitBase_t stop;
    orbitBase_t straight;
};

struct voltage_t
{
    float voltageL;
    float voltageR;
};

struct odometry_t
{
    float x;
    float y;
    float yaw;
};

struct notify_t
{
    odometry_t odom;
    float lipovoltage;
    uint16_t wallsens[4];
};

struct wallsensor_t
{
    //FR L R FL
    uint16_t value[4];
    uint16_t charge_us = 60;
    uint16_t rise_us = 15;
    const uint8_t SENS[4] = {0, 7, 1, 6};
    const gpio_num_t LED[4] = {GPIO_NUM_21, GPIO_NUM_10, GPIO_NUM_18, GPIO_NUM_17};
};

struct sensing_t
{
    uint16_t wall[4];
    float velL,velR;
    float lipo_V;
    float gyroZ_rad;
};

#endif