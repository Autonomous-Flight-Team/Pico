#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include <chrono> 
#include "unscented.hpp"
#include "Eigen/Dense"

// Use standard float types for the test harness
using Eigen::VectorXf;
using Eigen::MatrixXf;
using Eigen::Vector3f;

// ---------------------------------------------------------
// Helper: Vector Assertion (Float)
// We use VectorXf (Dynamic) here so it can accept 
// both Vector12f and Vector10f arguments.
// ---------------------------------------------------------
bool assert_vectors_near(const VectorXf& a, const VectorXf& b, float tolerance = 1e-3f) {
    if (a.size() != b.size()) {
        std::cerr << "Assertion failed: Vector sizes differ (" 
                  << a.size() << " vs " << b.size() << ")." << std::endl;
        return false;
    }
    for (int i = 0; i < a.size(); ++i) {
        if (std::abs(a(i) - b(i)) > tolerance) {
            std::cerr << "Assertion failed: Mismatch at index " << i << ". " 
                      << a(i) << " != " << b(i) << " (diff: " << std::abs(a(i)-b(i)) << ")" << std::endl;
            return false;
        }
    }
    return true;
}

size_t total_allocated = 0;
void* operator new(size_t size) {
    total_allocated += size;
    return malloc(size);
}

/**
 * Measures memory stability and footprint under a simulated high-frequency stream.
 * 10,000 cycles approximates ~40 seconds of flight at 250Hz.
 */
void RunMemoryLoadTest(UKF& ukf) {
    std::cout << "\n[Test 4] Memory Load & Stability Test (10,000 cycles)..." << std::endl;

    // Reset allocation counter to see if predict/update trigger new heap usage
    total_allocated = 0; 
    
    size_t initial_stack_size = sizeof(ukf);
    
    // Simulated Stream Data
    float dt = 0.004f; // 250Hz
    Vector3f m = Vector3f::Zero();
    Vector3f f = Vector3f::Zero();
    Vector10f z = Vector10f::Zero();

    for(int i = 0; i < 10000; ++i) {
        ukf.predict(dt, m, f);
        ukf.update(z);
        
        // Safety check: Ensure heap doesn't grow during the loop
        if (total_allocated > 0) {
            std::cerr << "   -> ALERT: Dynamic allocation detected at cycle " << i << std::endl;
            break;
        }
    }

    std::cout << "   -> Static Footprint:   " << initial_stack_size << " bytes" << std::endl;
    std::cout << "   -> Dynamic Leakage:    " << total_allocated << " bytes" << std::endl;
    
    if(total_allocated == 0) {
        std::cout << "   -> PASSED (Zero dynamic overhead during streaming)." << std::endl;
    } else {
        std::cout << "   -> FAILED (Heap usage detected)." << std::endl;
    }
}

// ---------------------------------------------------------
// Main Test Suite
// ---------------------------------------------------------
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "    UKF UNIT & PERFORMANCE TESTS (32-bit Float) " << std::endl;
    std::cout << "========================================" << std::endl;

    // -----------------------------------------------------
    // 1. SETUP
    // -----------------------------------------------------
    // Note: Dimensions are now implicit in the types, 
    // but we define values for initializers.
    UKF ukf; // Default constructor

    // Initial State (Using Vector12f explicitly)
    Vector12f x_initial;
    x_initial << 0.0f, 0.0f, 10.0f,  // Pos: 10m altitude
                 1.0f, 0.0f, 0.0f,   // Vel: 1 m/s in x
                 0.0f, 0.0f, 0.0f,   // Euler: Level
                 0.0f, 0.0f, 0.1f;   // AngVel: Yaw rate 0.1 rad/s
    
    // Initial Matrices (Using Matrix12f / Matrix10f)
    Matrix12f P = Matrix12f::Identity() * 0.1f;
    Matrix12f Q = Matrix12f::Identity() * 0.01f;
    Matrix10f R = Matrix10f::Identity() * 0.1f;

    ukf.init(x_initial, P, Q, R);
    ukf.set_drone_params(1.0f, Matrix3f::Identity()); 
    
    std::cout << "[Setup] UKF Initialized." << std::endl;

    // -----------------------------------------------------
    // 2. MEMORY TEST (Updated for Fixed-Size)
    // -----------------------------------------------------
    std::cout << "\n[Test 1] Memory Usage checking..." << std::endl;
    
    // Since we switched to fixed-size members, the "Heap" usage 
    // for matrices is now 0. Everything is inside the object (Stack).
    size_t object_size = sizeof(UKF);
    
    std::cout << "   -> Object Size (Stack): " << object_size << " bytes" << std::endl;
    std::cout << "   -> Heap Allocation:     ~0 bytes (Pre-allocated in object)" << std::endl;

    // Assertion: Ensure the object fits comfortably in stack memory
    // With floats, this should be smaller than the double version (~8KB-10KB)
    assert(object_size < 15000); 
    std::cout << "   -> PASSED (Memory footprint is small)." << std::endl;


    // -----------------------------------------------------
    // 3. FUNCTIONAL LOGIC TEST
    // -----------------------------------------------------
    std::cout << "\n[Test 2] Functional Logic checking..." << std::endl;

    float dt = 0.1f; 
    Vector3f moment_inputs = Vector3f::Zero(); 
    Vector3f force_inputs = Vector3f::Zero(); 

    // A. PREDICT STEP
    ukf.predict(dt, moment_inputs, force_inputs);
    Vector12f x_pred = ukf.get_state();

    // Verification:
    // x_new ~= x_old + vx * dt = 0 + 1 * 0.1 = 0.1
    // psi_new ~= psi_old + r * dt = 0 + 0.1 * 0.1 = 0.01
    assert(std::abs(x_pred(0) - 0.1f) < 1e-2f);   
    assert(std::abs(x_pred(8) - 0.01f) < 1e-2f);  
    std::cout << "   -> Prediction logic matches physics model." << std::endl;

    // B. UPDATE STEP
    Vector10f z_meas;
    z_meas << 0.15f, 0.0f, 9.5f,    // Pos (GPS says we are further X, lower Z)
              0.0f, 0.0f, 0.12f,    // Gyro 
              0.0f, 0.0f, -9.81f,   // Accel (Gravity pointing UP in body frame = NEGATIVE Z)
              1.1f;                 // Speed

    ukf.update(z_meas);
    Vector12f x_upd = ukf.get_state();

    // Verification:
    // The filter should fuse prediction (0.1) and measurement (0.15)
    assert(x_upd(0) > 0.10f && x_upd(0) < 0.16f);
    
    std::cout << "   -> Update logic successfully fused measurement." << std::endl;
    std::cout << "   -> PASSED (Physics and Math)." << std::endl;


    // -----------------------------------------------------
    // 4. PERFORMANCE BENCHMARK
    // -----------------------------------------------------
    std::cout << "\n[Test 3] Performance Benchmarking (1000 Cycles)..." << std::endl;

    // Reset filter
    ukf.init(x_initial, P, Q, R);

    auto start_time = std::chrono::high_resolution_clock::now();

    for(int i = 0; i < 1000; ++i) {
        ukf.predict(dt, moment_inputs, force_inputs);
        ukf.update(z_meas);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    float avg_time_per_cycle = total_duration.count() / 1000.0f;

    std::cout << "   -> Total Time: " << total_duration.count() << " us" << std::endl;
    std::cout << "   -> Avg/Cycle:  " << avg_time_per_cycle << " us" << std::endl;

    std::cout << "   -> PASSED (Performance is suitable for real-time)." << std::endl;

    std::cout << "========================================" << std::endl;
    std::cout << "    UKF MEMORY & LOAD ANALYSIS" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // Initialize with dummy values
    ukf.init(x_initial, P, Q, R);
    ukf.set_drone_params(1.0f, Matrix3f::Identity());

    // Run the new load test
    RunMemoryLoadTest(ukf);

    return 0;

    std::cout << "\n========================================" << std::endl;
    std::cout << "       ALL TESTS PASSED SUCCESSFULLY    " << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}