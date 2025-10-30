

// PID.h //Named to show that this file is the pair to PID.cpp
#ifndef WhiteMage
#define WhiteMage

// Defines constant for pid controlers
// TODO: Tune these guys
namespace Constants
{
    // Used for the planar pid, calculating the x, y, z position
    // Gyroscope PID constants
    constexpr double gyro_kp = 0;
    constexpr double gyro_ki = 0;
    constexpr double gyro_kd = 0;

    // Magnometer PID constants
    constexpr double mag_kp = 0;
    constexpr double mag_ki = 0;
    constexpr double mag_kd = 0;

    // Accelerometer PID constants
    constexpr double accel_kp = 0;
    constexpr double accel_ki = 0;
    constexpr double accel_kd = 0;

    // Used to calculate height pid
    float height_kp = 0;
    float height_ki = 0;
    float height_kd = 0;

}

#endif