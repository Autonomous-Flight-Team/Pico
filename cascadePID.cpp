// Import the constants header file + libraries
#include "cascadePID.h"
#include <iostream>
#include "customStructs.h"

/* Performs the PID calculation for a single 3D vector (x, y, z).
    Returns a planarVector containing the PID output for x, y, and z.
    Can only be used for a sensor that gives 3D outputs.

    All vectors are passed by reference becuase changes need to be made to some and its saves memory.
*/
class CascadePID
{
private:                                                                                                                                                   // These should only be called by the class itself
    planarVector computePID(const planarVector &actual, const planarVector &expected, updatingVector &state, double time, double kp, double ki, double kd) // Added different kp,ki,kd inputs for each sensor, these are in Constants.h
    {
        // Don't do anything if this function was called immidietly after last call or called with negative time.
        if (time <= 0)
        {
            return planarVector{0, 0, 0};
        }
        // Calculate proportional error (Used for big error differences)
        state.error_x = expected.x - actual.x;
        state.error_y = expected.y - actual.y;
        state.error_z = expected.z - actual.z;

        // Update integral error (Used for smaller )
        state.i_x += state.error_x * time;
        state.i_y += state.error_y * time;
        state.i_z += state.error_z * time;

        // Clamp integrals so you don't have a run away value when the error saturates
        state.i_x = clamp(state.i_x, -Constants::i_max, Constants::i_max);
        state.i_y = clamp(state.i_y, -Constants::i_max, Constants::i_max);
        state.i_z = clamp(state.i_z, -Constants::i_max, Constants::i_max);

        // Calculate derivative error (rate of change of error)
        double dx = -(actual.x - state.prev_x) / time;
        double dy = -(actual.y - state.prev_y) / time;
        double dz = -(actual.z - state.prev_z) / time;

        // Updates previous error for the next function call
        state.prev_x = actual.x;
        state.prev_y = actual.y;
        state.prev_z = actual.z;

        // Calculates the PID vector output and stores it in a planar vector struct.
        planarVector output;
        output.x = kp * state.error_x + ki * state.i_x + kd * dx;
        output.y = kp * state.error_y + ki * state.i_y + kd * dy;
        output.z = kp * state.error_z + ki * state.i_z + kd * dz; // Possibly have this have its own gain

        // Returns the computed PID correction vector
        return output;
    }

    planarVector linearPositionToVelocity(planarVector positional_PID)
    {
        // Assumes initially that the time to follow through the positional vector is t=1s
        planarVector expected_velocity{};
        expected_velocity.x = positional_PID.x / Constants::lin_velocity_divider;
        expected_velocity.y = positional_PID.y / Constants::lin_velocity_divider;
        expected_velocity.z = positional_PID.z / Constants::lin_velocity_divider;

        return expected_velocity;
    }

    planarVector angularPositionToVelocity(planarVector orientation_PID)
    {
        // Assumes initially that the time to follow through the positional vector is t=1s
        planarVector expected_velocity{};
        expected_velocity.x = orientation_PID.x / Constants::angular_velocity_divider;
        expected_velocity.y = orientation_PID.y / Constants::angular_velocity_divider;
        expected_velocity.z = orientation_PID.z / Constants::angular_velocity_divider;

        return expected_velocity;
    }

    /*Applies PID control across all three planar sensor vectors (position,velocity,
        and orientation). Each sensor uses its own updatingVector state for storing PID
        values between function calls.
        Returns a planarState containing the PID outputs for position, velocity, and orientation.
    */

public: // Only function that a consumer would call
    planarState plannerPid(const planarState &actual_state, const planarState &expected_state, updatingState &stateUpdate, double time)
    {
        // TODO: When this function is called, please call it with the time since the last time it was called

        // Create a return struct to hold the PID outputs for all vectors
        planarState planar_pid_output{};

        // Perform PID computation for each vector type, computePID handles all PID math

        // position
        planar_pid_output.position = computePID(actual_state.position, expected_state.position, stateUpdate.position, time,
                                                Constants::pos_kp, Constants::pos_ki, Constants::pos_kd);

        // Get the expected velocity vector from the positional_PID
        planarVector expected_velocity = linearPositionToVelocity(planar_pid_output.position);

        // velocity
        planar_pid_output.velocity = computePID(actual_state.velocity, expected_velocity, stateUpdate.velocity, time,
                                                Constants::vel_kp, Constants::vel_ki, Constants::vel_kd);

        // orientation
        planar_pid_output.orientation = computePID(actual_state.orientation, expected_state.orientation, stateUpdate.orientation, time,
                                                   Constants::ori_kp, Constants::ori_ki, Constants::ori_kd);

        planarVector expected_angular_velocity = angularPositionToVelocity(planar_pid_output.orientation);

        // Angular velocity
        planar_pid_output.angular_vel = computePID(actual_state.angular_vel, expected_angular_velocity, stateUpdate.angular_velocity, time,
                                                   Constants::angular_vel_kp, Constants::angular_vel_ki, Constants::angular_vel_kd);

        // Return the collection of PID output vectors in a planarState
        return planar_pid_output;
    }
};