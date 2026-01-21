#include <iostream>
#include <cmath>
#include "pidStructs.h"

// TODOs:
/*
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
    std::vector<double> thrust;
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

        // Finds the difference between our current location and wanted location
        double error_x = expected_loc[0] - actual_loc[0];
        double error_y = expected_loc[1] - actual_loc[1];
        double error_z = expected_loc[2] - actual_loc[2];

        // creates the new velocity vector
        std::vector<double> output;
        output[0] = p_p * position_history.x_error; //+ p_i * position_history.x_integral + p_d * position_history.x_derivative;
        output[1] = p_p * position_history.y_error; //+ p_i * position_history.y_integral + p_d * position_history.y_derivative;
        output[2] = p_p * position_history.z_error; //+ p_i * position_history.z_integral + p_d * position_history.z_derivative;

        return output; // Outputs desired velocity in the world frame
    }
    // Outputs an expected attitude for the drone //Call every 20 Hz
    attitude_vector velcoityPID(std::vector<double> expected_lin_vel /*comes from position PID*/, std::vector<double> actual_lin_vel /*comes from Kalman filter*/,
                                pidHistory &velocity_history, double dt /*May need to be converted from Hz */)
    {
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

        // Converts to [ax,ay,az] world reference frame to [roll, pitch, and thrust] body relative to world

        double g = 9.80665;
        double drone_mass = 1;

        // Sets the thrust vector, used as a way to convert from world to body RF
        output.thrust = {
            drone_mass * accel_x,
            drone_mass * accel_y,
            drone_mass * (accel_z + g)};

        // Finds |T|
        double t_normalizer = sqrt(output.thrust[0] * output.thrust[0] + output.thrust[1] * output.thrust[1] + output.thrust[2] * output.thrust[2]);

        std::vector<double> z_des(3);

        // Z_des is the normalizing of the thrust vector, making it the main setting axis. Where we want the force on the drone
        z_des[0] = output.thrust[0] / t_normalizer;
        z_des[1] = output.thrust[1] / t_normalizer;
        z_des[2] = output.thrust[2] / t_normalizer;

        // Sets a yaw
        double yaw = atan2(expected_lin_vel[1], expected_lin_vel[0]);

        // Sets the y_des
        std::vector<double> y_des(3);
        double y_normalizer = sqrt(pow(-z_des[2] * sin(yaw), 2) + pow(z_des[2] * sin(yaw), 2) + pow((z_des[0] * sin(yaw) - z_des[1] * sin(yaw)), 2));
        y_des[0] = -z_des[2] * sin(yaw) / y_normalizer;
        y_des[1] = z_des[2] * sin(yaw) / y_normalizer;
        y_des[2] = (z_des[0] * sin(yaw) - z_des[1] * sin(yaw)) / y_normalizer;

        // Set x_des
        std::vector<double> x_des(3);
        x_des[0] = y_des[1] * z_des[2] - y_des[2] * z_des[1];
        x_des[1] = -(y_des[0] * z_des[2] - z_des[0] * y_des[2]);
        x_des[2] = y_des[0] * z_des[1] - z_des[0] * y_des[1];

        // Using the desired states, get the attitude vector
        output.roll = atan2(y_des[2], z_des[2]);
        output.pitch = -asin(x_des[2]);
        output.yaw = yaw;

        return output; // Expected attitude for the drone ([roll, pitch, and thrust] body relative to world)
    }

    // Outputs the expected angular rates of the drone in relation to the body RF(angular velocity vector)
    attitude_vector attitudePID(attitude_vector desired_orientation /*comes from velocity PID (Ground RF)*/, attitude_vector actual_angles /*comes from the complementry filter */,
                                pidHistory &attitude_history, double dt)
    {
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

        return angular_rate; // Angular rate relative to body
    }

    // outputs a motor torque vector 4D, one for each motor
    // Call as often as possible, however fast you can call gyroscope
    // Set motors immidietly after
    attitude_vector angularRatePID(
        const attitude_vector &desired_rate, /* body-frame angular rates [p*,q*,r*] */ const attitude_vector &measured_rate, // body-frame from gyro
        pidHistory &angular_rate_history, double dt)
    {
        attitude_vector torque_output;

        // Compute body-frame error
        double error_roll = desired_rate.roll - measured_rate.roll;
        double error_pitch = desired_rate.pitch - measured_rate.pitch;
        double error_yaw = desired_rate.yaw - measured_rate.yaw;

        // Update integral term (per-axis clamped)
        const double I_MAX = 1.0;
        const double I_MIN = -1.0;

        angular_rate_history.x_integral += error_roll * dt;
        angular_rate_history.x_integral = clamp(angular_rate_history.x_integral, I_MAX, I_MIN);

        angular_rate_history.y_integral += error_pitch * dt;
        angular_rate_history.y_integral = clamp(angular_rate_history.y_integral, I_MAX, I_MIN);

        angular_rate_history.z_integral += error_yaw * dt;
        angular_rate_history.z_integral = clamp(angular_rate_history.z_integral, I_MAX, I_MIN);

        // Derivative
        double d_roll = -(measured_rate.roll - angular_rate_history.x_error) / dt;
        double d_pitch = -(measured_rate.pitch - angular_rate_history.y_error) / dt;
        double d_yaw = -(measured_rate.yaw - angular_rate_history.z_error) / dt;

        // update previous measurements
        angular_rate_history.x_error = measured_rate.roll;
        angular_rate_history.y_error = measured_rate.pitch;
        angular_rate_history.z_error = measured_rate.yaw;

        // Compute PID torque output
        torque_output.roll = ar_p * error_roll + ar_i * angular_rate_history.x_integral + ar_d * d_roll;
        torque_output.pitch = ar_p * error_pitch + ar_i * angular_rate_history.y_integral + ar_d * d_pitch;
        torque_output.yaw = ar_p * error_yaw + ar_i * angular_rate_history.z_integral + ar_d * d_yaw;

        return torque_output;
    }
};
