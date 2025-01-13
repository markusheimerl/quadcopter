#ifndef QUAD_H
#define QUAD_H

#include "vmath.h"

// Constants
#define K_F 0.0004905
#define K_M 0.00004905
#define L 0.25
#define L_SQRT2 (L / sqrtf(2.0))
#define G 9.81
#define M 0.5
#define OMEGA_MIN 30.0
#define OMEGA_MAX 70.0
#define OMEGA_STABLE 50.0

#define ACCEL_NOISE_STDDEV 0.1
#define GYRO_NOISE_STDDEV 0.01
#define ACCEL_BIAS 0.05
#define GYRO_BIAS 0.005

// Environment variables
double wind_force_W[3] = {0.0, 0.0, 0.0};
double target_wind_W[3] = {0.0, 0.0, 0.0};

// State variables
double omega[4] = {0.0, 0.0, 0.0, 0.0};
double linear_position_W[3] = {0.0, 0.0, 0.0};
double linear_velocity_W[3] = {0.0, 0.0, 0.0};
double angular_velocity_B[3] = {0.0, 0.0, 0.0};
double R_W_B[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
double I[3] = {0.01, 0.02, 0.01};

// Control variables
double omega_next[4] = {0.0, 0.0, 0.0, 0.0};
double linear_position_d_W[3] = {0.0, 0.0, 0.0};
double linear_velocity_d_W[3] = {0.0, 0.0, 0.0};
double linear_acceleration_d_W[3] = {0.0, 0.0, 0.0};
double angular_velocity_d_B[3] = {0.0, 0.0, 0.0};
double angular_acceleration_d_B[3] = {0.0, 0.0, 0.0};
double yaw_d = 0.0;

// Sensor variables
double linear_acceleration_B_s[3] = {0.0, 0.0, 0.0}; // Accelerometer
double angular_velocity_B_s[3] = {0.0, 0.0, 0.0}; // Gyroscope
double accel_bias[3] = {0.0, 0.0, 0.0};
double gyro_bias[3] = {0.0, 0.0, 0.0};

const double k_p = 0.2;
const double k_v = 0.7;
const double k_R = 0.7;
const double k_w = 0.7;

static double gaussian_noise(double stddev) {
    double u1 = (double)rand() / RAND_MAX, u2 = (double)rand() / RAND_MAX;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * M_PI * u2) * stddev;
}

void update_drone_physics(double dt) {
    // 1. Declare arrays and calculate rotor forces/moments
    double f[4], m[4];
    for(int i = 0; i < 4; i++) {
        omega[i] = fmax(fmin(omega[i], OMEGA_MAX), OMEGA_MIN);
        double omega_sq = omega[i] * fabs(omega[i]);
        f[i] = K_F * omega_sq;
        m[i] = K_M * omega_sq;
    }

    // 2. Calculate total thrust force in body frame (only y component is non-zero)
    double f_B_thrust[3] = {0, f[0] + f[1] + f[2] + f[3], 0};

    // 3. Initialize with drag torque (only y component is non-zero)
    double tau_B[3] = {0, m[0] - m[1] + m[2] - m[3], 0};

    // 4. Add thrust torques
    for(int i = 0; i < 4; i++) {
        double f_vector[3] = {0, f[i], 0};
        double tau_thrust[3];
        crossVec3f((double [4][3]){{-L, 0,  L}, { L, 0,  L}, { L, 0, -L}, {-L, 0, -L}}[i], f_vector, tau_thrust);
        addVec3f(tau_B, tau_thrust, tau_B);
    }

    // 5. Transform thrust to world frame and calculate linear acceleration
    double f_thrust_W[3];
    multMatVec3f(R_W_B, f_B_thrust, f_thrust_W);
    
    double linear_acceleration_W[3];
    for(int i = 0; i < 3; i++) {
        linear_acceleration_W[i] = f_thrust_W[i] / M;
    }
    linear_acceleration_W[1] -= G;  // Add gravity
/*
    if ((double)rand() / RAND_MAX < 0.01)  // 1% chance to change target wind
        for(int i = 0; i < 3; i++) if (i != 1) target_wind_W[i] = ((double)rand() / RAND_MAX - 0.5) * 1.0;  // ±0.5N
    
    for(int i = 0; i < 3; i++) {
        wind_force_W[i] = 0.995 * wind_force_W[i] + 0.005 * target_wind_W[i];
        linear_acceleration_W[i] += wind_force_W[i] / M;
    }
*/
    // 6. Calculate angular acceleration
    double I_mat[9];
    vecToDiagMat3f(I, I_mat);
    
    double h_B[3];
    multMatVec3f(I_mat, angular_velocity_B, h_B);
    
    double w_cross_h[3];
    crossVec3f(angular_velocity_B, h_B, w_cross_h);
    
    double angular_acceleration_B[3];
    for(int i = 0; i < 3; i++) {
        angular_acceleration_B[i] = (-w_cross_h[i] + tau_B[i]) / I[i];
    }

    // 7. Update states with Euler integration
    for(int i = 0; i < 3; i++) {
        linear_velocity_W[i] += dt * linear_acceleration_W[i];
        linear_position_W[i] += dt * linear_velocity_W[i];
        angular_velocity_B[i] += dt * angular_acceleration_B[i];
    }

    // Ensure the quadcopter doesn't go below ground level
    if (linear_position_W[1] < 0.0) linear_position_W[1] = 0.0;

    // 8. Update rotation matrix
    double w_hat[9];
    so3hat(angular_velocity_B, w_hat);
    
    double R_dot[9];
    multMat3f(R_W_B, w_hat, R_dot);
    
    double R_dot_scaled[9];
    multScalMat3f(dt, R_dot, R_dot_scaled);
    
    double R_new[9];
    addMat3f(R_W_B, R_dot_scaled, R_new);
    for (int i = 0; i < 9; i++) R_W_B[i] = R_new[i];

    // 9. Ensure rotation matrix stays orthonormal
    orthonormalize_rotation_matrix(R_W_B);

    // 10. Calculate sensor readings with noise
    double linear_acceleration_B[3], R_B_W[9];
    transpMat3f(R_W_B, R_B_W);
    multMatVec3f(R_B_W, linear_acceleration_W, linear_acceleration_B);
    double gravity_B[3];
    multMatVec3f(R_B_W, (double[3]){0, G, 0}, gravity_B);
    subVec3f(linear_acceleration_B, gravity_B, linear_acceleration_B);
    for(int i = 0; i < 3; i++) {
        linear_acceleration_B_s[i] = linear_acceleration_B[i] + gaussian_noise(ACCEL_NOISE_STDDEV) + accel_bias[i];
        angular_velocity_B_s[i] = angular_velocity_B[i] + gaussian_noise(GYRO_NOISE_STDDEV) + gyro_bias[i];
    }
}

void update_rotor_speeds(void) {
    for(int i = 0; i < 4; i++) {
        omega[i] = fmax(OMEGA_MIN, fmin(OMEGA_MAX, omega_next[i]));
    }
}

#endif // QUAD_H