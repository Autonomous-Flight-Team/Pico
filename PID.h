#ifndef WhiteMage
#define WhiteMage

// Defines constant for pid controlers
// SAMPLE CONSTANTS FOR TESTING FILE
namespace Constants
{
    // Used for the planar pid, calculating the x, y, z position
    // Gyroscope PID constants
    constexpr double gyro_kp = 0.8;
    constexpr double gyro_ki = 0.005; // Small I is okay for rate stabilization
    constexpr double gyro_kd = 0.05;

    // Magnometer PID constants
    constexpr double mag_kp = 0.5;
    constexpr double mag_ki = 0.01;
    constexpr double mag_kd = 0.01;

    // Accelerometer PID constants
    constexpr double accel_kp = 0.4;
    constexpr double accel_ki = 0.001;
    constexpr double accel_kd = 0.02;

    // Used to calculate height pid 
    float height_kp = 1.0;
    float height_ki = 0.02;
    float height_kd = 0.1;

}

#endif