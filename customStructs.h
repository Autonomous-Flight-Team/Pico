#ifndef BlackMage
#define BlackMage

// Used to store an x, y, z vector (such as gyroscope, magnometer, or acceleration)
struct planarVector
{
    float x, y, z;
};

// Used to store all planar vectors from the IMU (acceleration, magnometer, gyroscope)
struct planarState
{
    // Holds the x, y, z values of the gyroscope
    planarVector gyroscope;

    // Holds the x, y, z values of the compass directions
    planarVector magnometer;

    // Holds the x, y, z values of the acceleration
    planarVector acceleration;
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