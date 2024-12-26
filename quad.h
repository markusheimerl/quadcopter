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
double R_W_B[9];
double I[3] = {0.01, 0.02, 0.01};

// Control variables
double linear_position_d_W[3] = {-1.0, 0.5, -0.5};
double linear_velocity_d_W[3] = {0.0, 0.0, 0.0};
double linear_acceleration_d_W[3] = {0.0, 0.0, 0.0};
double angular_velocity_d_B[3] = {0.0, 0.0, 0.0};
double angular_acceleration_d_B[3] = {0.0, 0.0, 0.0};
double yaw_d = 0.0;
double omega_control[4];

const double k_p = 0.5;
const double k_v = 1.0;
const double k_R = 1.0;
const double k_w = 1.0;

void init_drone_state(void) {
    // Initialize rotor speeds
    for(int i = 0; i < 4; i++) {
        omega[i] = OMEGA_STABLE;
    }
    
    // Initialize angular velocity
    for(int i = 0; i < 3; i++) {
        angular_velocity_B[i] = 0.0;
    }
    
    // Initialize linear velocity
    for(int i = 0; i < 3; i++) {
        linear_velocity_W[i] = 0.0;
    }
    
    // Initialize position
    linear_position_W[0] = 0.0;
    linear_position_W[1] = 1.0;
    linear_position_W[2] = 0.0;
    
    // Initialize rotation matrix
    double temp[9];
    double result[9];
    xRotMat3f(0.0, temp);
    yRotMat3f(0.0, result);
    multMat3f(temp, result, R_W_B);
    zRotMat3f(0.0, temp);
    multMat3f(R_W_B, temp, R_W_B);
}

void update_drone_physics(void) {
    // 1. Clamp motor speeds
    for(int i = 0; i < 4; i++) {
        omega[i] = fmax(fmin(omega[i], OMEGA_MAX), OMEGA_MIN);
    }

    // 2. Calculate rotor forces and moments
    double f[4], m[4];
    double total_thrust = 0;
    double total_moment = 0;
    for(int i = 0; i < 4; i++) {
        f[i] = K_F * omega[i] * fabs(omega[i]);
        m[i] = K_M * omega[i] * fabs(omega[i]);
        total_thrust += f[i];
        total_moment += (i % 2 == 0) ? m[i] : -m[i];
    }

    // 3. Calculate thrust and torques
    double f_B_thrust[3] = {0, total_thrust, 0};
    double tau_B_drag[3] = {0, total_moment, 0};
    
    // Calculate thrust torques
    const double rotor_pos[4][3] = {
        {-L, 0, L},
        {L, 0, L},
        {L, 0, -L},
        {-L, 0, -L}
    };
    
    double tau_B_thrust[3] = {0, 0, 0};
    for(int i = 0; i < 4; i++) {
        double f_vector[3] = {0, f[i], 0};
        double tau_temp[3];
        crossVec3f(rotor_pos[i], f_vector, tau_temp);
        addVec3f(tau_B_thrust, tau_temp, tau_B_thrust);
    }

    // 4. Combine torques
    double tau_B[3];
    addVec3f(tau_B_drag, tau_B_thrust, tau_B);

    // 5. Transform forces to world frame
    double f_thrust_W[3];
    multMatVec3f((double*)R_W_B, f_B_thrust, f_thrust_W);
    
    // 6. Calculate linear acceleration (including gravity)
    double linear_acceleration_W[3];
    for(int i = 0; i < 3; i++) {
        linear_acceleration_W[i] = f_thrust_W[i] / M;
    }
    linear_acceleration_W[1] -= G;  // Add gravity in Y direction

    // 7. Calculate angular acceleration
    double I_mat[9];
    vecToDiagMat3f(I, I_mat);
    
    double h_B[3];
    multMatVec3f(I_mat, angular_velocity_B, h_B);
    
    double w_cross_h[3];
    double neg_angular_velocity[3];
    multScalVec3f(-1, angular_velocity_B, neg_angular_velocity);
    crossVec3f(neg_angular_velocity, h_B, w_cross_h);

    double angular_acceleration_B[3];
    for(int i = 0; i < 3; i++) {
        angular_acceleration_B[i] = (w_cross_h[i] + tau_B[i]) / I[i];
    }

    // 8. Update state variables using Euler integration
    for(int i = 0; i < 3; i++) {
        linear_velocity_W[i] += DT * linear_acceleration_W[i];
        linear_position_W[i] += DT * linear_velocity_W[i];
        angular_velocity_B[i] += DT * angular_acceleration_B[i];
    }

    // 9. Update rotation matrix
    double w_hat[9];
    so3hat(angular_velocity_B, w_hat);
    
    double R_dot[9];
    multMat3f((double*)R_W_B, w_hat, R_dot);
    
    double R_dot_scaled[9];
    multScalMat3f(DT, R_dot, R_dot_scaled);
    
    double R_new[9];
    addMat3f((double*)R_W_B, R_dot_scaled, R_new);
    
    memcpy(R_W_B, R_new, 9 * sizeof(double));
    orthonormalize_rotation_matrix(R_W_B);
}

void update_drone_control(void) {
    // 1. Position and Velocity Error Calculation
    double error_p[3], error_v[3];
    subVec3f(linear_position_W, linear_position_d_W, error_p);
    subVec3f(linear_velocity_W, linear_velocity_d_W, error_v);

    // 2. Desired Force Calculation
    double z_W_d[3];
    // Combine position and velocity feedback
    multScalVec3f(-k_p, error_p, z_W_d);
    double temp[3];
    multScalVec3f(-k_v, error_v, temp);
    addVec3f(z_W_d, temp, z_W_d);
    
    // Add gravity compensation and desired acceleration
    double gravity_term[3] = {0, M * G, 0};
    addVec3f(z_W_d, gravity_term, z_W_d);
    multScalVec3f(M, linear_acceleration_d_W, temp);
    addVec3f(z_W_d, temp, z_W_d);

    // 3. Thrust Calculation
    double y_body[3] = {0, 1, 0};
    double z_W_B[3];
    multMatVec3f((double*)R_W_B, y_body, z_W_B);
    double f_z_B_control = dotVec3f(z_W_d, z_W_B);

    // 4. Desired Rotation Matrix Calculation
    double x_tilde_d_W[3] = {sin(yaw_d), 0, cos(yaw_d)};
    double R_W_d[9];
    
    // Calculate desired rotation matrix columns
    double temp_cross1[3], temp_cross2[3];
    double R_W_d_col0[3], R_W_d_col1[3], R_W_d_col2[3];
    
    crossVec3f(z_W_d, x_tilde_d_W, temp_cross1);
    crossVec3f(temp_cross1, z_W_d, temp_cross2);
    
    normVec3f(temp_cross2, R_W_d_col0);
    normVec3f(temp_cross1, R_W_d_col1);
    normVec3f(z_W_d, R_W_d_col2);
    
    // Assemble rotation matrix
    for(int i = 0; i < 3; i++) {
        R_W_d[i*3 + 0] = R_W_d_col1[i];
        R_W_d[i*3 + 1] = R_W_d_col2[i];
        R_W_d[i*3 + 2] = R_W_d_col0[i];
    }

    // 5. Rotation Error Calculation
    double R_W_d_T[9], R_W_B_T[9], temp_mat[9];
    transpMat3f(R_W_d, R_W_d_T);
    transpMat3f((double*)R_W_B, R_W_B_T);
    
    double error_r[3];
    multMat3f(R_W_d_T, (double*)R_W_B, temp_mat);
    double temp_mat2[9];
    multMat3f(R_W_B_T, R_W_d, temp_mat2);
    subMat3f(temp_mat, temp_mat2, temp_mat);
    so3vee(temp_mat, error_r);
    multScalVec3f(0.5, error_r, error_r);

    // 6. Angular Velocity Error Calculation
    double error_w[3], temp_vec[3];
    multMat3f(R_W_d_T, (double*)R_W_B, temp_mat);
    multMatVec3f(temp_mat, angular_velocity_d_B, temp_vec);
    subVec3f(angular_velocity_B, temp_vec, error_w);

    // 7. Torque Control Calculation
    double tau_B_control[3];
    multScalVec3f(-k_R, error_r, tau_B_control);
    multScalVec3f(-k_w, error_w, temp_vec);
    addVec3f(tau_B_control, temp_vec, tau_B_control);

    // Add angular momentum compensation
    double I_mat[9];
    vecToDiagMat3f(I, I_mat);
    multMatVec3f(I_mat, angular_velocity_B, temp_vec);
    crossVec3f(angular_velocity_B, temp_vec, temp_vec);
    addVec3f(tau_B_control, temp_vec, tau_B_control);

    // Add desired angular acceleration compensation
    double term[3];
    multMatVec3f(R_W_d, angular_acceleration_d_B, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, term);
    
    multMatVec3f(R_W_d, angular_velocity_d_B, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, temp_vec);
    crossVec3f(angular_velocity_B, temp_vec, temp_vec);
    subVec3f(temp_vec, term, temp_vec);
    
    multMatVec3f(I_mat, temp_vec, temp_vec);
    subVec3f(tau_B_control, temp_vec, tau_B_control);

    // 8. Motor Speed Calculation
    double F_bar[16] = {
        K_F,    K_F,    K_F,    K_F,
        -K_F*L, K_F*L,  K_F*L,  -K_F*L,
        K_M,    -K_M,   K_M,    -K_M,
        K_F*L,  K_F*L,  -K_F*L, -K_F*L
    };

    double F_bar_inv[16];
    inv4Mat4f(F_bar, F_bar_inv);

    double control_input[4] = {
        f_z_B_control,
        tau_B_control[0],
        tau_B_control[1],
        tau_B_control[2]
    };

    double omega_sign_square[4];
    multMatVec4f(F_bar_inv, control_input, omega_sign_square);

    for(int i = 0; i < 4; i++) {
        omega_control[i] = sqrt(fabs(omega_sign_square[i]));
    }
}

void update_omega(void) {
    for(int i = 0; i < 4; i++) {
        omega[i] = omega_control[i];
    }
}

#endif // QUAD_H