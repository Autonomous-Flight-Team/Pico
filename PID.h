

// PID.h //Named to show that this file is the pair to PID.cpp
#ifndef WhiteMage
#define WhiteMage

// Defines constant for pid controlers
// TODO: Tune these guys
namespace Constants
{
    // Used for the planar pid, calculating the x, y, z position
    // Position PID constants
    constexpr double pos_kp = 0;
    constexpr double pos_ki = 0;
    constexpr double pos_kd = 0;

    // Velocity PID constants
    constexpr double vel_kp = 0;
    constexpr double vel_ki = 0;
    constexpr double vel_kd = 0;

    // Accelerometer PID constants
    constexpr double ori_kp = 0;
    constexpr double ori_ki = 0;
    constexpr double ori_kd = 0;

    // Used to calculate height pid
    float height_kp = 0;
    float height_ki = 0;
    float height_kd = 0;

}

#endif