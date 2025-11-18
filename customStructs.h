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

// Clamp function to prevent overunning integral values
// Returns the low if value is less than low and high if value is greater than high
template <typename T>
T clamp(T value, T low, T high)
{
    if (value < low)
    {
        return low;
    }
    else if (value > high)
    {
        return high;
    }
    return value;
}

#endif