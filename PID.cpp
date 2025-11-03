// Import the constants header file + libraries
#include "PID.h"
#include <iostream>
#include "customStructs.h"

// Holds the continuously changing PID states for all planar vectors
struct updatingState
{
    // PID state for the position
    updatingVector position;

    // PID state for the velocity
    updatingVector velocity;

    // PID state for the orientation
    updatingVector orientation;
};

// This probably shouldn’t be global, but it's used to store continuously updating PID values
updatingState stateUpdate;

/* Performs the PID calculation for a single 3D vector (x, y, z).
    Returns a planarVector containing the PID output for x, y, and z.
    Can only be used for a sensor that gives 3D outputs.

    All vectors are passed by reference becuase changes need to be made to some and its saves memory.
*/
planarVector computePID(const planarVector &actual, const planarVector &expected, updatingVector &state, double time,
                        double kp, double ki, double kd) // Added different kp,ki,kd inputs for each sensor, these are in Constants.h
{
    // Calculate proportional error (Used for big error differences)
    state.error_x = expected.x - actual.x;
    state.error_y = expected.y - actual.y;
    state.error_z = expected.z - actual.z;

    // Update integral error (Used for smaller )
    state.i_x += state.error_x * time;
    state.i_y += state.error_y * time;
    state.i_z += state.error_z * time;

    // Calculate derivative error (rate of change of error)
    double dx = (state.error_x - state.prev_error_x) / time;
    double dy = (state.error_y - state.prev_error_y) / time;
    double dz = (state.error_z - state.prev_error_z) / time;

    // Updates previous error for the next function call
    state.prev_error_x = state.error_x;
    state.prev_error_y = state.error_y;
    state.prev_error_z = state.error_z;

    // Calculates the PID vector output and stores it in a planar vector struct.
    planarVector output;
    output.x = kp * state.error_x + ki * state.i_x + kd * dx;
    output.y = kp * state.error_y + ki * state.i_y + kd * dy;
    output.z = kp * state.error_z + ki * state.i_z + kd * dz;

    // Returns the computed PID correction vector
    return output;
}

/*Applies PID control across all three planar sensor vectors (position,velocity,
    and orientation). Each sensor uses its own updatingVector state for storing PID
    values between function calls.

    Returns a planarState containing the PID outputs for position, velocity, and orientation.
*/
planarState plannerPid(const planarState &actual_state, const planarState &expected_state, updatingState &stateUpdate, double time)
{
    // TODO:Someway to update time (pointer or reference thing)

    // Create a return struct to hold the PID outputs for all vectors
    planarState planar_pid_output;

    // Perform PID computation for each vector type, computePID handles all PID math

    // position
    planar_pid_output.position = computePID(actual_state.position, expected_state.position, stateUpdate.position, time,
                                            Constants::pos_kp, Constants::pos_ki, Constants::pos_kd);

    // velocity
    planar_pid_output.velocity = computePID(actual_state.velocity, expected_state.velocity, stateUpdate.velocity, time,
                                            Constants::vel_kp, Constants::vel_ki, Constants::vel_kd);

    // orientation
    planar_pid_output.orientation = computePID(actual_state.orientation, expected_state.orientation, stateUpdate.orientation, time,
                                               Constants::ori_kp, Constants::ori_ki, Constants::ori_kd);

    // Return the collection of PID output vectors in a planarState
    return planar_pid_output;
}

/* int main()
{
    std::cout << "Compiled and ran";
    return 0;
} */
