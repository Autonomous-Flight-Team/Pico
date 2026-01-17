#include <iostream>
#include <cmath>
#include "pidStructs.h"

// Complementry holder info, will update with Kalaman Filter but also hold estimates between calls
// A Kalman filter call should also update this vector
struct complementeryFilterData
{
    // State vector: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    std::vector<double> stateVector;

    //
};

class ComplementeryFilter
{
    // These vectors need to be able to be gotten by other functions in another class
    // but only updated by functions in this class
    static std::vector<double> fullStateVector; // [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    static std::vector<double> positionState;   //[x, y, z]
    static std::vector<double> velocityState;   //[vx, vy, vz]
    static std::vector<double> attitudeState;   //[phi, theta, psi]
    static std::vector<double> angularState;    //[p, q, r]

    // updates the full State vector and each of the other vectors based on Kalman filter returns
    // Call immidietly after Kalman filter is done
public:
    void updateState()
    {
        fullStateVector = {1}; // Kalmanfilter integration here;

        // Update Vectors
        // updated position
        positionState.assign(fullStateVector.begin(), fullStateVector.begin() + 3);

        // updated velocity
        velocityState.assign(fullStateVector.begin() + 3, fullStateVector.begin() + 6);

        // Update attitude
        attitudeState.assign(fullStateVector.begin() + 6, fullStateVector.begin() + 9);

        // Update angular velocities
        angularState.assign(fullStateVector.begin() + 9, fullStateVector.end());
    }

    // Partially updates vectors based on sensor data, creates a low-level guess about where we're at
    // Call whenevere new sensor data is in (Set up flags)
    void updateState(const std::vector<double> &magnometer, const std::vector<double> &accelerometer,
                     const std::vector<double> &gyroscope, double dt)
    {

        // Compute attitude from
        std::vector<double> accelerometer_est = {
            atan2(accelerometer[1], accelerometer[2]),
            atan2(-accelerometer[0], std::sqrt(accelerometer[1] * accelerometer[1] + accelerometer[2] * accelerometer[2])),
            0 // set later
        };

        // Compute yaw attitude from the magnometer
        std::vector<double> magnometer_yaw = {
            magnometer[0] * cos(accelerometer_est[0]) + magnometer[1] * sin(accelerometer_est[0]) * cos(accelerometer_est[1]) + magnometer[2] * sin(accelerometer_est[0]) * cos(accelerometer_est[1]),
            magnometer[1] * cos(accelerometer_est[1]) - magnometer[2] * sin(accelerometer_est[1]),
            0 // Technically not 0, but we don't need this
        };

        // set yaw
        accelerometer_est[2] = atan2(-magnometer_yaw[1], magnometer_yaw[0]);

        // Calculate attitude from gyroscope
        std::vector<double> gyroscope_est = {
            attitudeState[0] + gyroscope[0] * dt,
            attitudeState[1] + gyroscope[1] * dt,
            attitudeState[2] + gyroscope[2] * dt};

        // Calculate new attitude estimate

        // Tells how much emphasis to put on each estimate, greater alpha, more emphasis on gyro_estimate (has less drift)
        const double alpha_rp = 0.98; // roll + pitch
        const double alpha_y = 0.95;  // yaw

        attitudeState = {
            alpha_rp * gyroscope_est[0] + (1 - alpha_rp) * accelerometer_est[0],
            alpha_rp * gyroscope_est[1] + (1 - alpha_rp) * accelerometer_est[1],
            alpha_y * gyroscope_est[2] + (1 - alpha_y) * accelerometer_est[2]};
    }
};