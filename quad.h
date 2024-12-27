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
#define DT (1.0 / 60.0)
#define OMEGA_MIN 30.0
#define OMEGA_MAX 70.0
#define OMEGA_STABLE 50.0

// State variables
double omega[4];
double angular_velocity_B[3];
double linear_velocity_W[3];
double linear_position_W[3];
double linear_acceleration_B[3];
double R_W_B[9];
double I[3] = {0.01, 0.02, 0.01};

// Control variables
double linear_position_d_W[3] = {-1.0, 0.5, -0.5};
double linear_velocity_d_W[3] = {0.0, 0.0, 0.0};
double linear_acceleration_d_W[3] = {0.0, 0.0, 0.0};
double angular_velocity_d_B[3] = {0.0, 0.0, 0.0};
double angular_acceleration_d_B[3] = {0.0, 0.0, 0.0};
double yaw_d = 3.14 / 2.0;

const double k_p = 0.5;
const double k_v = 1.0;
const double k_R = 1.0;
const double k_w = 1.0;

void init_drone_state(void) {
    // 1. Initialize rotor speeds to stable hover
    for(int i = 0; i < 4; i++) {
        omega[i] = OMEGA_STABLE;
    }
    
    // 2. Initialize velocities to zero
    double zero_vec[3] = {0.0, 0.0, 0.0};
    memcpy(angular_velocity_B, zero_vec, 3 * sizeof(double));
    memcpy(linear_velocity_W, zero_vec, 3 * sizeof(double));
    
    // 3. Initialize position to (0, 1, 0)
    double init_pos[3] = {0.0, 1.0, 0.0};
    memcpy(linear_position_W, init_pos, 3 * sizeof(double));
    
    // 4. Initialize rotation matrix to identity (level orientation)
    memcpy(R_W_B, (double[9]){1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, 9 * sizeof(double));
}

void update_drone_physics(void) {
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
        linear_velocity_W[i] += DT * linear_acceleration_W[i];
        linear_position_W[i] += DT * linear_velocity_W[i];
        angular_velocity_B[i] += DT * angular_acceleration_B[i];
    }

    // 8. Update rotation matrix
    double w_hat[9];
    so3hat(angular_velocity_B, w_hat);
    
    double R_dot[9];
    multMat3f(R_W_B, w_hat, R_dot);
    
    double R_dot_scaled[9];
    multScalMat3f(DT, R_dot, R_dot_scaled);
    
    double R_new[9];
    addMat3f(R_W_B, R_dot_scaled, R_new);
    memcpy(R_W_B, R_new, 9 * sizeof(double));

    // 9. Ensure rotation matrix stays orthonormal
    orthonormalize_rotation_matrix(R_W_B);

    // 10. Calculate linear acceleration in body frame (for accelerometer simulation)
    double gravity_W[3] = {0, -G, 0}; // Gravity in world frame
    double gravity_B[3]; // Gravity in body frame

    // Transform gravity to body frame
    multMatVec3f(R_W_B, gravity_W, gravity_B);

    // Transform linear acceleration from world frame to body frame
    double linear_acceleration_W_minus_gravity[3];
    for (int i = 0; i < 3; i++) {
        linear_acceleration_W_minus_gravity[i] = linear_acceleration_W[i] - gravity_W[i];
    }
    multMatVec3f(R_W_B, linear_acceleration_W_minus_gravity, linear_acceleration_B);
}

void update_drone_control(void) {
    // 1. Calculate position and velocity errors
    double error_p[3], error_v[3];
    subVec3f(linear_position_W, linear_position_d_W, error_p);
    subVec3f(linear_velocity_W, linear_velocity_d_W, error_v);

    // 2. Calculate desired force vector in world frame
    double z_W_d[3], temp[3];
    multScalVec3f(-k_p, error_p, z_W_d);
    multScalVec3f(-k_v, error_v, temp);
    addVec3f(z_W_d, temp, z_W_d);
    
    // Add gravity compensation and desired acceleration
    double gravity_term[3] = {0, M * G, 0};
    addVec3f(z_W_d, gravity_term, z_W_d);
    
    double accel_term[3];
    multScalVec3f(M, linear_acceleration_d_W, accel_term);
    addVec3f(z_W_d, accel_term, z_W_d);

    // 3. Calculate thrust magnitude
    double z_W_B[3];
    double y_body[3] = {0, 1, 0};
    multMatVec3f(R_W_B, y_body, z_W_B);
    double f_z_B_control = dotVec3f(z_W_d, z_W_B);

    // 4. Calculate desired rotation matrix
    double x_tilde_d_W[3] = {sin(yaw_d), 0, cos(yaw_d)};
    double temp_cross1[3], temp_cross2[3];
    double R_W_d_column_0[3], R_W_d_column_1[3], R_W_d_column_2[3];
    
    crossVec3f(z_W_d, x_tilde_d_W, temp_cross1);
    crossVec3f(temp_cross1, z_W_d, temp_cross2);
    normVec3f(temp_cross2, R_W_d_column_0);
    normVec3f(temp_cross1, R_W_d_column_1);
    normVec3f(z_W_d, R_W_d_column_2);

    double R_W_d[9] = {
        R_W_d_column_1[0], R_W_d_column_2[0], R_W_d_column_0[0],
        R_W_d_column_1[1], R_W_d_column_2[1], R_W_d_column_0[1],
        R_W_d_column_1[2], R_W_d_column_2[2], R_W_d_column_0[2]
    };

    // 5. Calculate rotation error
    double R_W_d_T[9], R_W_B_T[9], temp_mat1[9], temp_mat2[9], temp_mat3[9];
    transpMat3f(R_W_d, R_W_d_T);
    transpMat3f(R_W_B, R_W_B_T);
    
    multMat3f(R_W_d_T, R_W_B, temp_mat1);
    multMat3f(R_W_B_T, R_W_d, temp_mat2);
    subMat3f(temp_mat1, temp_mat2, temp_mat3);

    double error_r[3];
    so3vee(temp_mat3, error_r);
    multScalVec3f(0.5, error_r, error_r);

    // 6. Calculate angular velocity error
    double temp_vec[3], error_w[3];
    multMat3f(R_W_d_T, R_W_B, temp_mat1);
    multMatVec3f(temp_mat1, angular_velocity_d_B, temp_vec);
    subVec3f(angular_velocity_B, temp_vec, error_w);

    // 7. Calculate control torque
    double tau_B_control[3], temp_vec2[3];
    multScalVec3f(-k_R, error_r, tau_B_control);
    multScalVec3f(-k_w, error_w, temp_vec2);
    addVec3f(tau_B_control, temp_vec2, tau_B_control);

    // Add angular momentum terms
    double I_mat[9], temp_vec3[3], temp_vec4[3];
    vecToDiagMat3f(I, I_mat);
    multMatVec3f(I_mat, angular_velocity_B, temp_vec3);
    crossVec3f(angular_velocity_B, temp_vec3, temp_vec4);
    addVec3f(tau_B_control, temp_vec4, tau_B_control);

    // Add feedforward terms
    double term_0[3], term_1[3], temp_vec5[3];
    multMatVec3f(R_W_d, angular_acceleration_d_B, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, term_0);
    
    multMatVec3f(R_W_d, angular_velocity_d_B, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, temp_vec2);
    crossVec3f(angular_velocity_B, temp_vec2, term_1);
    
    subVec3f(term_1, term_0, temp_vec5);
    multMatVec3f(I_mat, temp_vec5, temp_vec);
    subVec3f(tau_B_control, temp_vec, tau_B_control);

    // 8. Calculate rotor speeds
    double F_bar[16] = {
        K_F, K_F, K_F, K_F,   // Thrust coefficients
        0, 0, 0, 0,           // Roll moments
        K_M, -K_M, K_M, -K_M, // Yaw moments
        0, 0, 0, 0            // Pitch moments
    };

    // Calculate roll and pitch moments
    for(int i = 0; i < 4; i++) {
        double moment[3];
        double pos_scaled[3];
        multScalVec3f(K_F, (double [4][3]){{-L, 0,  L}, { L, 0,  L}, { L, 0, -L}, {-L, 0, -L}}[i], pos_scaled);
        crossVec3f(pos_scaled, (double[3]){0, 1, 0}, moment);
        F_bar[4 + i]  = moment[0];  // Roll
        F_bar[12 + i] = moment[2];  // Pitch
    }

    // 9. Calculate and update rotor speeds
    double F_bar_inv[16];
    inv4Mat4f(F_bar, F_bar_inv);

    double control_input[4] = {f_z_B_control, tau_B_control[0], tau_B_control[1], tau_B_control[2]};
    double omega_sign_square[4];
    multMatVec4f(F_bar_inv, control_input, omega_sign_square);

    for(int i = 0; i < 4; i++) {
        omega[i] = sqrt(fabs(omega_sign_square[i]));
    }
}

#endif // QUAD_H