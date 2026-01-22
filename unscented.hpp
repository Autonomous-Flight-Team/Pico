#ifndef UNSCENTED_HPP
#define UNSCENTED_HPP

#include "Eigen/Dense"
#include <vector>
#include <cmath>

// ---------------------------------------------------------
// CONSTANTS & TYPEDEFS (Hard-coded for Float Optimization)
// ---------------------------------------------------------
constexpr int STATE_DIM = 12;      // x dimension
constexpr int MEAS_DIM = 10;       // z dimension
constexpr int SIGMA_POINTS = 25;   // 2 * n_x + 1

using Vector12f = Eigen::Matrix<float, STATE_DIM, 1>;
using Vector10f = Eigen::Matrix<float, MEAS_DIM, 1>;
using Matrix12f = Eigen::Matrix<float, STATE_DIM, STATE_DIM>;
using Matrix10f = Eigen::Matrix<float, MEAS_DIM, MEAS_DIM>;
using Matrix12x25f = Eigen::Matrix<float, STATE_DIM, SIGMA_POINTS>;
using Matrix10x25f = Eigen::Matrix<float, MEAS_DIM, SIGMA_POINTS>;
using Vector25f    = Eigen::Matrix<float, SIGMA_POINTS, 1>;
using Matrix12x10f = Eigen::Matrix<float, STATE_DIM, MEAS_DIM>; // For K matrix

using Eigen::Vector3f;
using Eigen::Matrix3f;

/**
 * @class UKF
 * Implements an Unscented Kalman Filter for drone state estimation
 *
 * This class estimates a 12-dimensional state vector:
 * x = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
 * Given the 10-dimensional measurement vector:
 * z = [x, y, z, p, q, r, ax, ay, az, s]
 */
class UKF {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Constructor
     * @param alpha Spread parameter for sigma points
     * @param beta Parameter to incorporate prior knowledge of the distribution
     * @param kappa Secondary spread parameter
     */
    UKF(float alpha = 1.0f, float beta = 2.0f, float kappa = -9.0f);

    /**
     * Initializes the filter with initial state, covariance, and noise matrices
     * @param x_in Initial state vector
     * @param P_in Initial state covariance matrix
     * @param Q_in Process noise covariance matrix
     * @param R_in Measurement noise covariance matrix
     */
    void init(const Vector12f& x_in, const Matrix12f& P_in, const Matrix12f& Q_in, const Matrix10f& R_in);

    /**
     * Sets the physical parameters of the drone
     * @param mass_in Mass of the drone in kg
     * @param J_in Inertia matrix (3x3)
     */
    void set_drone_params(float mass_in, const Matrix3f& J_in);

    /**
     * Performs the prediction step of the UKF
     * @param dt Time step (delta t) in seconds
     * @param moment_inputs Control inputs: Moments [Mx, My, Mz]
     * @param force_inputs Control inputs: Forces [Fx, Fy, Fz] (usually [0,0,Thrust])
     */
    void predict(float dt, const Vector3f& moment_inputs, const Vector3f& force_inputs);

    /**
     * Performs the update step of the UKF
     * @param z The measurement vector from the sensors
     */
    void update(const Vector10f& z);

    /**
     * Gets the current estimated state vector
     * @return const Vector12f& The state vector
     */
    const Vector12f& get_state() const { return x_; }

    /**
     * Gets the current state covariance matrix
     * @return const Matrix12f& The state covariance matrix
     */
    const Matrix12f& get_covariance() const { return P_; }

private:
    // Filter Dimensions (Implicit in types, kept for logic)
    int n_x_ = STATE_DIM;
    int n_z_ = MEAS_DIM;
    int n_sig_ = SIGMA_POINTS;

    // UKF Parameters
    float lambda_; // Sigma point scaling parameter
    
    // Drone Physical Parameters
    float mass_;
    Matrix3f J_;    // Inertia Matrix
    Matrix3f J_inv_; // Pre-calculated Inverse

    // Filter State & Covariance
    Vector12f x_; // State vector
    Matrix12f P_; // State covariance matrix

    // Noise Matrices
    Matrix12f Q_; // Process noise covariance matrix
    Matrix10f R_; // Measurement noise covariance matrix

    // Sigma Points
    Matrix12x25f X_sig_pred_; // Predicted sigma points matrix
    Vector25f weights_m_;  // Weights for calculating mean
    Vector25f weights_c_;  // Weights for calculating covariance

    // Helper Functions
    Matrix12x25f generate_sigma_points(const Vector12f& x, const Matrix12f& P);
    float normalize_angle(float angle);
    
    // The core physics and sensor models
    Vector12f process_function(const Vector12f& current, float dt, const Vector3f& moment_inputs, const Vector3f& force_inputs);
    Vector10f measurement_function(const Vector12f& current);
};

#endif // UNSCENTED_HPP