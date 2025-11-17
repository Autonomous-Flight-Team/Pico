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

    // Holds the positional yaw, pitch, roll values of the acceleration (x, y, z)
    planarVector orientation; // (Yaw, pitch, roll)

    // Holds the angular velocity
    planarVector angular_vel;
};

#endif