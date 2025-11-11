// Import the constants header file + libraries
#include "customStructs.h"
#include "runMotors.h"
#include "pinConstants.h"

// Outputs a motorVoltage that holds how each motors' voltages needs to
// change to get to the correct rotation
motorVoltage SetRotation(planarVector orientation)
{
}

// Outputs a motorVoltage that holds how each motors' voltages needs to
// change to get to the correct velocity
motorVoltage SetVelocity(planarVector velocity)
{
}

// Add all the voltages together
motorVoltage SetVoltage(planarState stateDirections)
{
    motorVoltage rotation_voltage = SetRotation(stateDirections.orientation);
    motorVoltage velocity_voltage = SetVelocity(stateDirections.velocity);
}

// Add somthing that sends