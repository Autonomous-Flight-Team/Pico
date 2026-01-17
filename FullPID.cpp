#include <iostream>
#include <cmath>
#include "pidStructs.h"

// TODOs:
/*
- Write complementry filter
- Add more flags for motor saturation
- Write checks to stop drone from flipping
*/

// structs

struct linear_vector
{
    double x{0}, y{0}, z{0};
};

struct attitude_vector
{
    double roll{0};
    double pitch{0};
    double yaw{0};
    double thrust{0};
};
// PID state holder

struct pidHistory
{
    double x_error{0}, y_error{0}, z_error{0}; // holds Previous error
    double x_integral{0}, y_integral{0}, z_integral{0};
    double x_derivative{0}, y_derivative{0}, z_derivative{0};
};

// motor torque struct. holds info about each motor, may be torque or voltage depending on when used
struct motor_vector
{
    double front_left_motor{0}, back_left_motor{0}, front_right_motor{0}, back_right_motor{0};
};

// Helper function
double clamp(double num, double upper_num, double lower_num)
{
    if (num < lower_num)
    {
        return lower_num;
    }

    else if (num > upper_num)
    {
        return upper_num;
    }
    else
    {
        return num;
    }
}

// Clamps angles between -pi and pi
auto wrap = [](double a)
{
    return atan2(sin(a), cos(a));
};

// Creates a flag to mark if motor values are maxed out
//  If motor values are maxed out, don't update integral error
bool maxed_motors = false;
const double max_motor_voltage = 22.5;

// Variables to be moved lator
linear_vector expected_loc; // Comes from UI (longitude/latittude/z?)
linear_vector actual_loc;   // This will come from the Kalman filter
                            // Position Constants
double p_p = 0;
double p_i = 0;
double p_d = 0;

// Velocity constants
double v_p = 0;
double v_i = 0;
double v_d = 0;

// Attitude constants
double a_p = 0;
double a_i = 0;
double a_d = 0;

// Angular rate constants
double ar_p = 0;
double ar_i = 0;
double ar_d = 0;

// linear to angular Coversion constants/gains
double pitch_offset = 1;
double roll_offset = 1;

class FullPID
{
    // Variable boxes that need to hang around for a bit but not sure where to declare them.

    pidHistory position_history;
    pidHistory velocity_history;
    pidHistory attitude_history;
    pidHistory angular_rate_history;

public:
    // Position PID (Other variables can be passed by reference if we need to save space) // Call every 10 Hz
    // Called immidietly after Kalman Filter output
    std::vector<double> positionPID(std::vector<double> expected_loc, std::vector<double> actual_loc,
                                    pidHistory &position_history, double dt /*May need to be converted from Hz */)
    {

        if (dt <= 0)
        {
            return;
        }
        // Finds the difference between our current location and wanted location
        double error_x = expected_loc[0] - actual_loc[0];
        double error_y = expected_loc[1] - actual_loc[1];
        double error_z = expected_loc[2] - actual_loc[2];

        // Finds/updates the integral of error
        if (!maxed_motors)
        {
            position_history.x_integral += error_x * dt;
            position_history.y_integral += error_y * dt;
            position_history.z_integral += error_z * dt;
        }

        // finds the derivative of error
        position_history.x_derivative = (error_x - position_history.x_error) / dt;
        position_history.y_derivative = (error_y - position_history.y_error) / dt;
        position_history.z_derivative = (error_z - position_history.z_error) / dt;

        // Updates the history of the pid error
        position_history.x_error = error_x;
        position_history.y_error = error_y;
        position_history.z_error = error_z;

        // creates the new vector
        std::vector<double> output;
        output[0] = p_p * position_history.x_error + p_i * position_history.x_integral + p_d * position_history.x_derivative;
        output[1] = p_p * position_history.y_error + p_i * position_history.y_integral + p_d * position_history.y_derivative;
        output[2] = p_p * position_history.z_error + p_i * position_history.z_integral + p_d * position_history.z_derivative;

        return output;
    }
    // Outputs an expected attitude for the drone //Call every 20 Hz
    attitude_vector velcoityPID(std::vector<double> expected_lin_vel /*comes from position PID*/, std::vector<double> actual_lin_vel /*comes from Kalman filter*/,
                                pidHistory &velocity_history, double dt /*May need to be converted from Hz */)
    {
        if (dt <= 0)
        {
            return;
        }
        // Finds the difference between our current velocity and wanted velocity
        double error_x = expected_lin_vel[0] - actual_lin_vel[0];
        double error_y = expected_lin_vel[1] - actual_lin_vel[1];
        double error_z = expected_lin_vel[2] - actual_lin_vel[2];

        // Finds the integral of error
        if (!maxed_motors)
        {
            velocity_history.x_integral += error_x * dt;
            velocity_history.y_integral += error_y * dt;
            velocity_history.z_integral += error_z * dt;
        }

        // finds the derivative of error
        velocity_history.x_derivative = (error_x - velocity_history.x_error) / dt;
        velocity_history.y_derivative = (error_y - velocity_history.y_error) / dt;
        velocity_history.z_derivative = (error_z - velocity_history.z_error) / dt;

        // Updates the history of the pid error
        velocity_history.x_error = error_x;
        velocity_history.y_error = error_y;
        velocity_history.z_error = error_z;

        // creates the new vector
        attitude_vector output;

        // Gets the new acceleration vector outputs
        double accel_x = v_p * error_x + v_i * velocity_history.x_integral + v_d * velocity_history.x_derivative;
        double accel_y = v_p * error_y + v_i * velocity_history.y_integral + v_d * velocity_history.y_derivative;
        double accel_z = v_p * error_z + v_i * velocity_history.z_integral + v_d * velocity_history.z_derivative;

        // Converts to roll, pitch, and thrust
        // Arctan is used for higher accelerations, a_y/g can be used for small acceleration
        // 𝑟𝑜𝑙𝑙(𝜙)=k * 𝑡𝑎𝑛−1(𝑎_𝑦/𝑔) // k is suggested for extra precision
        // pitch(theta)=k * 𝑡𝑎𝑛−1(𝑎_x/𝑔)
        // Thrust should stay the same
        // Atan2(y,x) will handle cases when the denominator is zero unlike atan(y/x)

        double g = 9.80665;
        output.pitch = pitch_offset * atan2(accel_x, g);
        output.roll = roll_offset * -atan2(accel_y, g);
        output.thrust = accel_z + g; // thrust needs to push against gravity //In world frame but will get converted to body frame later

        // set yaw, desired ψ*=atan2(v_y​,v_x​) //currently drone will face direction of travel.
        double epsilon = 1e-17;
        if (hypot(expected_lin_vel[0], expected_lin_vel[1]) > epsilon)
        {
            output.yaw = atan2(expected_lin_vel[1], expected_lin_vel[0]);
        }
        else
        {
            output.yaw = prev_yaw; // This will be from complementary filter
        }

        return output;
    }

    // Outputs the expected angular rates of the drone in relation to the body RF(angular velocity vector)
    attitude_vector attitudePID(attitude_vector desired_orientation /*comes from velocity PID (Ground RF)*/, attitude_vector actual_angles /*comes from the complementry filter */,
                                pidHistory &attitude_history, double dt)
    {
        if (dt <= 0)
        {
            return;
        }
        // Thrust will not be used here, but it will be countinued to be passed through the attitude_vector
        // Finds the difference between our wanted angular orientation and actual angular orientation

        // Apply wraps to error to prevent saturated motors and spin
        double error_roll = wrap(desired_orientation.roll - actual_angles.roll);
        double error_pitch = wrap(desired_orientation.pitch - actual_angles.pitch);
        double error_yaw = wrap(desired_orientation.yaw - actual_angles.yaw);
        // double error_roll = desired_orientation.roll - actual_angles.roll;
        // double error_pitch = desired_orientation.pitch - actual_angles.pitch;
        // double error_yaw = desired_orientation.yaw - actual_angles.yaw;

        // Finds the integral of error
        if (!maxed_motors)
        {
            attitude_history.x_integral += error_roll * dt;
            attitude_history.y_integral += error_pitch * dt;
            attitude_history.z_integral += error_yaw * dt;
        }

        // finds the derivative of error
        attitude_history.x_derivative = (error_roll - attitude_history.x_error) / dt;
        attitude_history.y_derivative = (error_pitch - attitude_history.y_error) / dt;
        attitude_history.z_derivative = (error_yaw - attitude_history.z_error) / dt;

        // Updates the history of the pid error
        attitude_history.x_error = error_roll;
        attitude_history.y_error = error_pitch;
        attitude_history.z_error = error_yaw;

        // creates the new vector
        attitude_vector angular_rate;

        // Gets the new acceleration vector outputs
        double angular_vel_roll = a_p * error_roll + a_i * attitude_history.x_integral + a_d * attitude_history.x_derivative;
        double angular_vel_pitch = a_p * error_pitch + a_i * attitude_history.y_integral + a_d * attitude_history.y_derivative;
        double angular_vel_yaw = a_p * error_yaw + a_i * attitude_history.z_integral + a_d * attitude_history.z_derivative;

        // Transforms the angular velocitys with ground RF to body RF //See (φ*, θ*, ψ*) → (p*, q*, r*) transformation matrix
        angular_rate.roll = angular_vel_roll - sin(actual_angles.pitch) * angular_vel_yaw;                                                       // p*
        angular_rate.pitch = cos(actual_angles.roll) * angular_vel_pitch + cos(actual_angles.pitch) * sin(actual_angles.roll) * angular_vel_yaw; // q*
        angular_rate.yaw = -sin(actual_angles.roll) * angular_vel_pitch + cos(actual_angles.pitch) * cos(actual_angles.roll) * angular_vel_yaw;  // r*

        // Keep thrust in obj
        // Thrust gets converted to drones body frame
        double phi = actual_angles.roll;
        double theta = actual_angles.pitch;

        double thrust = desired_orientation.thrust / (cos(phi) * cos(theta));
        angular_rate.thrust = thrust;

        return angular_rate;
    }

    // outputs a motor torque vector 4D, one for each motor
    // Call as often as possible, however fast you can call gyroscope
    // Set motors immidietly after
    motor_vector angularRatePID(attitude_vector expected_rate, attitude_vector actual_rate /*comes from gyro*/,
                                pidHistory &angular_rate_history, double dt)
    {
        if (dt <= 0)
        {
            return;
        }
        // Finds the difference between angular rate and what we want angular rate to be
        double error_roll = wrap(expected_rate.roll - actual_rate.roll);
        double error_pitch = wrap(expected_rate.pitch - actual_rate.pitch);
        double error_yaw = wrap(expected_rate.yaw - actual_rate.yaw);

        // Finds the integral of error if motors are not maxed out
        if (!maxed_motors)
        {
            angular_rate_history.x_integral += error_roll * dt;
            angular_rate_history.y_integral += error_pitch * dt;
            angular_rate_history.z_integral += error_yaw * dt;
        }

        // finds the derivative of error
        angular_rate_history.x_derivative = (error_roll - angular_rate_history.x_error) / dt;
        angular_rate_history.y_derivative = (error_pitch - angular_rate_history.y_error) / dt;
        angular_rate_history.z_derivative = (error_yaw - angular_rate_history.z_error) / dt;

        // Updates the history of the pid error
        angular_rate_history.x_error = error_roll;
        angular_rate_history.y_error = error_pitch;
        angular_rate_history.z_error = error_yaw;

        // Gets the new acceleration vector outputs
        double roll = ar_p * error_roll + ar_i * angular_rate_history.x_integral + ar_d * angular_rate_history.x_derivative;
        double pitch = ar_p * error_pitch + ar_i * angular_rate_history.y_integral + ar_d * angular_rate_history.y_derivative;
        double yaw = ar_p * error_yaw + ar_i * angular_rate_history.z_integral + ar_d * angular_rate_history.z_derivative;

        // Converts angular acceleration into motor voltages (May possibly need a constant here, not sure yet)
        // TODO: Change clamps to
        motor_vector motor_voltage;
        motor_voltage.front_left_motor = clamp(expected_rate.thrust + pitch + roll - yaw, max_motor_voltage, 0);
        motor_voltage.front_right_motor = clamp(expected_rate.thrust + pitch - roll + yaw, max_motor_voltage, 0);
        motor_voltage.back_left_motor = clamp(expected_rate.thrust - pitch + roll + yaw, max_motor_voltage, 0);
        motor_voltage.back_right_motor = clamp(expected_rate.thrust - pitch - roll - yaw, max_motor_voltage, 0);

        // Checks to see in any of the motors have a maxed out volatage, used to stop integral wind up
        maxed_motors =
            motor_voltage.front_left_motor == max_motor_voltage ||
            motor_voltage.front_right_motor == max_motor_voltage ||
            motor_voltage.back_left_motor == max_motor_voltage ||
            motor_voltage.back_right_motor == max_motor_voltage;

        return motor_voltage;
    }
};