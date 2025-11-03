#include "PID.h"
#include <iostream>
#include "PID.cpp"
#include "customStructs.h"
// TESTING FUNCTIONS 

// tolerance for floating point comparisons
const double TOLERANCE = 1e-6;

// method to assert that two floating point numbers are nearly equal
#define ASSERT_NEAR(a, b, tol) \
    if (std::abs((a) - (b)) > (tol)) { \
        std::cerr << "Assertion failed: " << #a << " != " << #b << " (within " << tol << ")\n"; \
        exit(1); \
    } else { \
        std::cerr << "Assertion passed: " << #a << " == " << #b << " (within " << tol << ")\n"; \
    }

// Utility function to reset state for a new test
void resetState(updatingVector &state) {
    state = {}; // Zero-initialize the struct
}

void test_proportional_term()
{
    // std::cout << "--- Proportional Tests ---" << std::endl;
    updatingVector state = {};
    const double KP = 2.0;
    const double KI = 0.0;
    const double KD = 0.0;
    const double DT = 0.1; // Delta time

    planarVector actual = {10.0f, 5.0f, 0.0f};
    planarVector expected = {15.0f, 5.0f, -2.0f};

    planarVector output = computePID(actual, expected, state, DT, KP, KI, KD);

    // Expected error: X=5.0, Y=0.0, Z=-2.0
    // Expected output: P * Error
    // X: 2.0 * 5.0 = 10.0
    // Y: 2.0 * 0.0 = 0.0
    // Z: 2.0 * -2.0 = -4.0

    ASSERT_NEAR(output.x, 10.0, 1e-6);
    ASSERT_NEAR(output.y, 0.0, 1e-6);
    ASSERT_NEAR(output.z, -4.0, 1e-6);
}

void test_integral_term()
{
    // std::cout << "\n--- Integral Tests ---" << std::endl;
    updatingVector state = {};
    const double KP = 0.0;
    const double KI = 0.5;
    const double KD = 0.0;
    const double DT = 0.1; // Delta time

    planarVector actual_step1 = {10.0f, 0.0f, 0.0f};
    planarVector expected = {12.0f, 0.0f, 0.0f}; // Constant error of 2.0

    // Step 1: Accumulate Integral
    planarVector output1 = computePID(actual_step1, expected, state, DT, KP, KI, KD);
    // Error X = 2.0
    // I_X = 2.0 * 0.1 = 0.2
    // Output X = 0.5 * 0.2 = 0.1
    ASSERT_NEAR(output1.x, 0.1, 1e-6);
    ASSERT_NEAR(state.i_x, 0.2, 1e-6);

    // Step 2: Accumulate more Integral (second step)
    planarVector output2 = computePID(actual_step1, expected, state, DT, KP, KI, KD);
    // Error X = 2.0
    // I_X = 0.2 + (2.0 * 0.1) = 0.4
    // Output X = 0.5 * 0.4 = 0.2
    ASSERT_NEAR(output2.x, 0.2, 1e-6);
    ASSERT_NEAR(state.i_x, 0.4, 1e-6);
}

void test_derivative_term()
{
    // std::cout << "\n--- Derivative Tests ---" << std::endl;
    updatingVector state = {};
    const double KP = 0.0;
    const double KI = 0.0;
    const double KD = 4.0;
    const double DT = 0.05; // Delta time

    planarVector expected = {10.0f, 0.0f, 0.0f}; // Target is constant

    // Step 1: Set initial error (P term stores this in prev_error)
    planarVector actual_step1 = {12.0f, 0.0f, 0.0f}; // Error = -2.0
    computePID(actual_step1, expected, state, DT, KP, KI, KD);
    // Prev Error X is now -2.0. D term output should be 0 (no change yet).

    // Step 2: Error decreases (controller is working well)
    planarVector actual_step2 = {11.0f, 0.0f, 0.0f}; // New Error = -1.0
    planarVector output2 = computePID(actual_step2, expected, state, DT, KP, KI, KD);

    // Delta Error = New Error - Prev Error = (-1.0) - (-2.0) = +1.0
    // Derivative = Delta Error / DT = 1.0 / 0.05 = 20.0
    // Output X = KD * Derivative = 4.0 * 20.0 = 80.0 (Positive output to slow the rate of change)
    ASSERT_NEAR(output2.x, 80.0, 1e-6);

    // Step 3: Error increases (controller is not working well)
    planarVector actual_step3 = {13.0f, 0.0f, 0.0f}; // New Error = -3.0
    planarVector output3 = computePID(actual_step3, expected, state, DT, KP, KI, KD);

    // Prev Error X is now -1.0 (from step 2)
    // Delta Error = New Error - Prev Error = (-3.0) - (-1.0) = -2.0
    // Derivative = Delta Error / DT = -2.0 / 0.05 = -40.0
    // Output X = KD * Derivative = 4.0 * -40.0 = -160.0 (Negative output to reverse the increase)
    ASSERT_NEAR(output3.x, -160.0, 1e-6); 
}

void test_full_planner_integration()
{
    // std::cout << "\n--- Running Full Planner Integration Test ---" << std::endl;
    updatingState state = {};
    double DT = 0.05;

    // Test a scenario where gyroscope has an error on X (pitch rate)
    planarState actual_state = {
        /* gyroscope */ {0.5f, 0.0f, 0.0f}, // Actual pitch rate is 0.5 rad/s
        /* magnometer */ {0.0f, 0.0f, 0.0f},
        /* acceleration */ {0.0f, 0.0f, 0.0f}
    };
    planarState expected_state = {
        /* gyroscope */ {0.0f, 0.0f, 0.0f}, // Expected pitch rate is 0.0 (hold steady)
        /* magnometer */ {0.0f, 0.0f, 0.0f},
        /* acceleration */ {0.0f, 0.0f, 0.0f}
    };

    planarState output = plannerPid(actual_state, expected_state, state, DT);

    // Gyroscope PID (using Constants::gyro_kp = 0.8 from PID.h update)
    // Error X = -0.5
    // P = Constants::gyro_kp * -0.5 = 0.8 * -0.5 = -0.4
    // I = Constants::gyro_ki * (-0.5 * 0.05) = 0.005 * -0.025 = -0.000125
    // D = Constants::gyro_kd * ((-0.5 - 0.0) / 0.05) = 0.05 * -10.0 = -0.5
    // Total Expected Output X = -0.4 - 0.000125 - 0.5 = -0.900125

    ASSERT_NEAR(output.gyroscope.x, -0.900125, 1e-6);
    ASSERT_NEAR(output.magnometer.x, 0.0, 1e-6);
    ASSERT_NEAR(output.acceleration.x, 0.0, 1e-6);

    std::cout << "Planner test successful, Gyro X output: " << output.gyroscope.x << std::endl;
}

void runTests()
{
    test_proportional_term();
    test_integral_term();
    test_derivative_term();
    test_full_planner_integration();
    std::cout << "\nAll PID tests completed successfully." << std::endl;
}

int main()
{
    std::ios::sync_with_stdio(false);
    runTests();
    return 0;
}