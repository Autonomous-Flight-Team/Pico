// Import the constants header file + libraries
#include "PID.h"
#include "customStructs.h"

// Creates a list of the 4 motors on the drone to store the number that will control their voltage
struct motorVoltage
{
    int backRight, frontRight, backLeft, frontLeft;
};

// Outputs a motorVoltage that holds how each motors' voltages needs to
// change to get to the correct rotation
int ChangeRotation(planarVector magnometer)
{
}

// Add all the voltages together