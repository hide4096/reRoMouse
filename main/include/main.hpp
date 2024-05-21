#ifndef STRUCTS_HPP
#define STRUCTS_HPP

#include "NeoPixel.hpp"
#include "MPU6500.hpp"
#include "PCA9632.hpp"
#include "Buzzer.hpp"
#include "Motor.hpp"
#include "MA730.hpp"

struct pid_gain_t
{
    float kp;
    float ki;
    float kd;
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

struct orbit_t
{
    float x;
    float y;
};

struct orbitBase_t
{
    int size;
    orbit_t *orbit;
};

class driver_t
{
public:
    MA730 *encL;
    MA730 *encR;
    MPU6500 *imu;
    PCA9632 *led;
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

    float ff_gain = 0;
    float accelMax = 3.0;           // m/s/s
    float angaccelMax = 20 * M_PI;  // rad/s/s

    orbitBase_t slalom;
    orbitBase_t start;
    orbitBase_t stop;
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
    voltage_t output_voltage;
    voltage_t ff_voltage;
    odometry_t odom;
    float tgt_vel;
    float tgt_angvel;
};

struct routemap_t
{
    orbitBase_t *orbit;
    Direction direction;
    bool isReverse;
};

#endif

