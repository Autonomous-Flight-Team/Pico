#ifndef RedMage
#define RedMage

// Creates a list of the 4 motors on the drone to store the number that will control their voltage
struct motorVoltage
{
    double backRight, frontRight, backLeft, frontLeft;
};

namespace motorConstanst
{
    double LinearTorqueToVoltage = 1;  // converts the wanted torque value to a voltage mV/s^2
    double AngularTorqueToVoltage = 1; // Might be same as above
} // namespace motorConstanst

#endif