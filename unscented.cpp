#include "unscented.hpp"
#include <iostream>

// Constructor
UKF::UKF(float alpha, float beta, float kappa)
{
    // Define Lambda
    lambda_ = alpha * alpha * (static_cast<float>(n_x_) + kappa) - static_cast<float>(n_x_);

    // Initialize weights
    float n_x_f = static_cast<float>(n_x_);
    weights_m_(0) = lambda_ / (lambda_ + n_x_f);
    weights_c_(0) = lambda_ / (lambda_ + n_x_f) + (1.0f - alpha * alpha + beta);
    float weight_i = 0.5f / (lambda_ + n_x_f);
    for (int i = 1; i < n_sig_; ++i) {
        weights_m_(i) = weight_i;
        weights_c_(i) = weight_i;
    }

    // Default drone params (should be set via set_drone_params)
    mass_ = 1.0f;
    J_ = Matrix3f::Identity();
    J_inv_ = Matrix3f::Identity();
}

void UKF::set_drone_params(float mass_in, const Matrix3f& J_in) {
    mass_ = mass_in;
    J_ = J_in;
    J_inv_ = J_.inverse();
}

void UKF::init(const Vector12f& x_in, const Matrix12f& P_in, const Matrix12f& Q_in, const Matrix10f& R_in) {
    x_ = x_in;
    P_ = P_in;
    Q_ = Q_in;
    R_ = R_in;
}

// Helper to normalize angles
float UKF::normalize_angle(float angle) {
    const float PI_F = 3.14159265358979f;
    while (angle > PI_F) angle -= 2.0f * PI_F;
    while (angle < -PI_F) angle += 2.0f * PI_F;
    return angle;
}

// Generate Sigma Points
Matrix12x25f UKF::generate_sigma_points(const Vector12f& x, const Matrix12f& P) {
    Matrix12x25f X_sig;
    
    // Calculate square root of P using Cholesky decomposition 
    Matrix12f A = P.llt().matrixL();

    // Set first sigma point (the mean)
    X_sig.col(0) = x;

    // Set remaining sigma points
    float sqrt_lambda_n_x = std::sqrt(lambda_ + static_cast<float>(n_x_));
    for (int i = 0; i < n_x_; ++i) {
        X_sig.col(i + 1)        = x + sqrt_lambda_n_x * A.col(i);
        X_sig.col(i + 1 + n_x_) = x - sqrt_lambda_n_x * A.col(i);
    }
    return X_sig;
}

// Process Function
Vector12f UKF::process_function(const Vector12f& current, float dt, const Vector3f& moment_inputs, const Vector3f& force_inputs) {
    // Unpack State: [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r]
    Vector3f pos_old = current.segment<3>(0);
    Vector3f translational_vel_old = current.segment<3>(3);
    Vector3f euler_old = current.segment<3>(6);
    Vector3f angular_vel_old = current.tail<3>();

    // Angular Acceleration
    Vector3f angular_acc = J_inv_ * (moment_inputs - angular_vel_old.cross(J_ * angular_vel_old));

    // Angular Velocity Transformation
    float phi = euler_old.x();
    float theta = euler_old.y();
    // Transformation matrix from body frame to inertial frame for angular rates
    Matrix3f W;
    W << 1.0f, std::sin(phi)*std::tan(theta), std::cos(phi)*std::tan(theta),
         0.0f, std::cos(phi),                 -std::sin(phi),
         0.0f, std::sin(phi)/std::cos(theta), std::cos(phi)/std::cos(theta);
    Vector3f euler_rates = W * angular_vel_old;
    
    // Linear Velocity Transformation
    float psi = euler_old.z();
    Matrix3f R; // Rotation matrix from body to inertial frame
    R << std::cos(psi)*std::cos(theta), std::cos(psi)*std::sin(theta)*std::sin(phi) - std::sin(psi)*std::cos(phi), std::cos(psi)*std::sin(theta)*std::cos(phi) + std::sin(psi)*std::sin(phi),
         std::sin(psi)*std::cos(theta), std::sin(psi)*std::sin(theta)*std::sin(phi) + std::cos(psi)*std::cos(phi), std::sin(psi)*std::sin(theta)*std::cos(phi) - std::cos(psi)*std::sin(phi),
         -std::sin(theta),              std::cos(theta)*std::sin(phi),                                             std::cos(theta)*std::cos(phi);
    Vector3f vel_inertial = R * translational_vel_old;

    // Linear Acceleration (thrust + gravity - coriolis)
    Vector3f gravity_inertial(0.0f, 0.0f, -9.81f);
    Vector3f gravity_body = R.transpose() * gravity_inertial;
    Vector3f linear_acc = (force_inputs / mass_) + gravity_body - angular_vel_old.cross(translational_vel_old);

    // Integration
    Vector12f next_state;
    next_state.segment<3>(0) = pos_old + vel_inertial * dt;
    next_state.segment<3>(3) = translational_vel_old + linear_acc * dt;
    next_state.segment<3>(6) = euler_old + euler_rates * dt;
    next_state.tail<3>() = angular_vel_old + angular_acc * dt;

    return next_state;
}

// Predict Step
void UKF::predict(float dt, const Vector3f& moment_inputs, const Vector3f& force_inputs) {
    // Generate Sigma Points
    Matrix12x25f X_sig = generate_sigma_points(x_, P_);

    // Propagate Sigma Points
    for (int i = 0; i < n_sig_; ++i) {
        X_sig_pred_.col(i) = process_function(X_sig.col(i), dt, moment_inputs, force_inputs);
    }

    // Predict State Mean
    x_.setZero();
    for (int i = 0; i < n_sig_; ++i) {
        x_ += weights_m_(i) * X_sig_pred_.col(i);
    }

    // Predict State Covariance
    P_.setZero();
    for (int i = 0; i < n_sig_; ++i) {
        Vector12f x_diff = X_sig_pred_.col(i) - x_;
        // Normalize angles
        x_diff(6) = normalize_angle(x_diff(6));
        x_diff(7) = normalize_angle(x_diff(7));
        x_diff(8) = normalize_angle(x_diff(8));
        
        P_ += weights_c_(i) * x_diff * x_diff.transpose();
    }
    
    // Add process noise
    P_ += Q_;
}

// Measurement Function
// Measurement vector: [x, y, z, p, q, r, ax, ay, az, s] (arbitrary order, can be changed)
Vector10f UKF::measurement_function(const Vector12f& current) {
    Vector10f z;
    
    Vector3f pos = current.segment<3>(0);
    Vector3f translational_vel = current.segment<3>(3);
    Vector3f euler = current.segment<3>(6);
    Vector3f angular_vel = current.tail<3>();

    float phi = euler.x();
    float theta = euler.y();
    float psi = euler.z();
    float g = 9.81f;

    // Position 
    z.segment<3>(0) = pos;

    // Gyro
    z.segment<3>(3) = angular_vel;

    // Accel (Gravity Vector in Body Frame)
    z(6) =  g * std::sin(theta);
    z(7) = -g * std::cos(theta) * std::sin(phi);
    z(8) = -g * std::cos(theta) * std::cos(phi);

    // Ground speed
    Matrix3f R; // Rotation matrix from body to inertial frame
    R << std::cos(psi)*std::cos(theta), std::cos(psi)*std::sin(theta)*std::sin(phi) - std::sin(psi)*std::cos(phi), std::cos(psi)*std::sin(theta)*std::cos(phi) + std::sin(psi)*std::sin(phi),
         std::sin(psi)*std::cos(theta), std::sin(psi)*std::sin(theta)*std::sin(phi) + std::cos(psi)*std::cos(phi), std::sin(psi)*std::sin(theta)*std::cos(phi) - std::cos(psi)*std::sin(phi),
         -std::sin(theta),              std::cos(theta)*std::sin(phi),                                             std::cos(theta)*std::cos(phi);
    Vector3f vel_inertial = R * translational_vel;
    z(9) = std::sqrt(vel_inertial.x() * vel_inertial.x() + vel_inertial.y() * vel_inertial.y());

    return z;
}

// Update Step
void UKF::update(const Vector10f& z) {
    // Predict Measurement Sigma Points
    Matrix10x25f Z_sig;
    for (int i = 0; i < n_sig_; ++i) {
        Z_sig.col(i) = measurement_function(X_sig_pred_.col(i));
    }

    // Predict Measurement Mean
    Vector10f z_pred = Vector10f::Zero();
    for (int i = 0; i < n_sig_; ++i) {
        z_pred += weights_m_(i) * Z_sig.col(i);
    }

    // Predict Measurement Covariance (S) and Cross Covariance (Tc)
    Matrix10f S = Matrix10f::Zero();
    Matrix12x10f Tc = Matrix12x10f::Zero();

    for (int i = 0; i < n_sig_; ++i) {
        // Measurement difference
        Vector10f z_diff = Z_sig.col(i) - z_pred;
        
        // State difference
        Vector12f x_diff = X_sig_pred_.col(i) - x_;
        x_diff(6) = normalize_angle(x_diff(6));
        x_diff(7) = normalize_angle(x_diff(7));
        x_diff(8) = normalize_angle(x_diff(8));

        S += weights_c_(i) * z_diff * z_diff.transpose();
        Tc += weights_c_(i) * x_diff * z_diff.transpose();
    }
    
    // Add measurement noise
    S += R_;

    // Update State and Covariance
    Matrix12x10f K = S.llt().solve(Tc.transpose()).transpose();
    Vector10f z_residual = z - z_pred;
    
    x_ += K * z_residual;
    P_ -= K * S * K.transpose();
}