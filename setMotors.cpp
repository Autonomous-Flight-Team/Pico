// Import the constants header file + libraries
#include "customStructs.h"
#include "setMotors.h"
#include "pinConstants.h"
// THIS COULD BE IMPROVED BY HAVING A CONSTRUCTOR BUILD THIS CLASS WITH CORRECT MOTOR PINS

// Outputs a motorVoltage that holds how each motors' voltages needs to

class setMotors
{

private: // These should only be called by the class itself
    // changes voltage to get to the correct rotation
    motorVoltage setVoltages(planarVector linear_acceleration, planarVector angular_acceleration)
    {

        // Turns each of the torque values into a voltage value
        double ax = linear_acceleration.x * motorConstanst::LinearTorqueToVoltage;
        double ay = linear_acceleration.y * motorConstanst::LinearTorqueToVoltage;
        double az = linear_acceleration.z * motorConstanst::LinearTorqueToVoltage;
        double yaw = angular_acceleration.z * motorConstanst::AngularTorqueToVoltage;

        // Positive ax means going forward
        // Postive ay means going right
        // Positive ax means going up
        // Positive yaw means turning counter clockwise

        // Clamp prevents us from trying to set an impossible motor voltage, set between min and max

        // Creates a motor voltage vector
        motorVoltage voltageHolder{};

        // Front Left Motor // Spins Counter Clockwise
        voltageHolder.frontLeft = clamp(-ax + ay + az + yaw, motorConstanst::minMotorVoltage, motorConstanst::maxMotorVoltage);

        // Front Right Motor //Spins Clockwise
        voltageHolder.frontRight = clamp(-ax - ay + az - yaw, motorConstanst::minMotorVoltage, motorConstanst::maxMotorVoltage);

        // Back left motor // Spins Counter Clockwise
        voltageHolder.backLeft = clamp(ax + ay + az - yaw, motorConstanst::minMotorVoltage, motorConstanst::maxMotorVoltage);

        // Back Right Motor // Spins Clockwise
        voltageHolder.backRight = clamp(ax - ay + az + yaw, motorConstanst::minMotorVoltage, motorConstanst::maxMotorVoltage);

        return voltageHolder;
    }
    // Add all the voltages together
public: // Only function a user should need to call
    motorVoltage SetVoltage(planarVector linear_acceleration, planarVector angular_acceleration)
    {

        return setVoltages(linear_acceleration, angular_acceleration);
    }

    // Add somthing that sends the data to the ESC
};