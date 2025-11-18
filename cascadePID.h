

// PID.h //Named to show that this file is the pair to PID.cpp
#ifndef WhiteMage
#define WhiteMage

// Defines the structs needed for just CascadePID.cpp

// Holds the continuously changing PID states for all planar vectors

// Holds the continuously changing PID states for one vector (x, y, z)
struct updatingVector
{
    // Holds current proportional errors
    double error_x, error_y, error_z;

    // Holds previous x,y,z vector (used for derivative term)
    double prev_x, prev_y, prev_z;
    ;

    // Holds the accumulated integral terms
    double i_x, i_y, i_z;
};

struct updatingState
{
    // PID state for the position
    updatingVector position;

    // PID state for the velocity
    updatingVector velocity;

    // PID state for the orientation
    updatingVector orientation;

    // PID State for the orientation velocity
    updatingVector angular_velocity;
};

// Defines constant for pid controlers
// TODO: Tune these guys
namespace Constants
{
    // Used for the planar pid, calculating the x, y, z position
    // Position PID constants
    constexpr double pos_kp = 0;
    constexpr double pos_ki = 0;
    constexpr double pos_kd = 0;

    // Time used to conver the Positional PID output to a velocity vector
    // Changes how quickly the drone tries to get to the expected position
    constexpr double lin_velocity_divider = 1;
    constexpr double angular_velocity_divider = 1;

    // Linear Velocity PID constants
    constexpr double vel_kp = 0;
    constexpr double vel_ki = 0;
    constexpr double vel_kd = 0;

    // Orientation PID constants
    constexpr double ori_kp = 0;
    constexpr double ori_ki = 0;
    constexpr double ori_kd = 0;

    // Angular_velocity PID constants
    constexpr double angular_vel_kp = 0;
    constexpr double angular_vel_ki = 0;
    constexpr double angular_vel_kd = 0;

    // Used to calculate height pid
    float height_kp = 0;
    float height_ki = 0;
    float height_kd = 0;

    // Used to prevent integral error values from overshooting
    constexpr double i_max = 1;

}

#endif