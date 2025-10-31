#ifndef BlackMage
#define BlackMage

// Used to store an x, y, z vector (position, velocity, orientation)
struct planarVector
{
    float x, y, z;
};

// Used to store all planar vectors from the IMU (position, velocity, orientation)
struct planarState
{
    // Holds the x, y, z values of the positions
    planarVector position;

    // Holds the x, y, z values of the velocity
    planarVector velocity;

    // Holds the yaw, pitch, roll values of the acceleration (x, y, z)
    planarVector orientation; // (Yaw, pitch, roll)
};

// Holds the continuously changing PID states for one vector (x, y, z)
struct updatingVector
{
    // Holds current proportional errors
    double error_x, error_y, error_z;

    // Holds previous proportional errors (used for derivative term)
    double prev_error_x, prev_error_y, prev_error_z;

    // Holds the accumulated integral terms
    double i_x, i_y, i_z;
};

#endif