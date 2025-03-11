#ifndef QUAD_H
#define QUAD_H

#include <math.h>

// 3x3 Matrix Operations
void multMat3f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            result[i*3 + j] = a[i*3]*b[j] + a[i*3+1]*b[j+3] + a[i*3+2]*b[j+6];
}

void multMatVec3f(const double* m, const double* v, double* result) {
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
    result[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
    result[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

void vecToDiagMat3f(const double* v, double* result) {
    for(int i = 0; i < 9; i++) result[i] = 0;
    result[0] = v[0];
    result[4] = v[1];
    result[8] = v[2];
}

void transpMat3f(const double* m, double* result) {
    result[0] = m[0]; result[1] = m[3]; result[2] = m[6];
    result[3] = m[1]; result[4] = m[4]; result[5] = m[7];
    result[6] = m[2]; result[7] = m[5]; result[8] = m[8];
}

void so3hat(const double* v, double* result) {
    result[0]=0; result[1]=-v[2]; result[2]=v[1];
    result[3]=v[2]; result[4]=0; result[5]=-v[0];
    result[6]=-v[1]; result[7]=v[0]; result[8]=0;
}

void so3vee(const double* m, double* result) {
    result[0] = m[7];
    result[1] = m[2];
    result[2] = m[3];
}

// Matrix arithmetic
void addMat3f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 9; i++) result[i] = a[i] + b[i];
}

void subMat3f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 9; i++) result[i] = a[i] - b[i];
}

void multScalMat3f(double s, const double* m, double* result) {
    for(int i = 0; i < 9; i++) result[i] = s * m[i];
}

// Vector Operations
void crossVec3f(const double* a, const double* b, double* result) {
    result[0] = a[1]*b[2] - a[2]*b[1];
    result[1] = a[2]*b[0] - a[0]*b[2];
    result[2] = a[0]*b[1] - a[1]*b[0];
}

void multScalVec3f(double s, const double* v, double* result) {
    for(int i = 0; i < 3; i++) result[i] = s * v[i];
}

void addVec3f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 3; i++) result[i] = a[i] + b[i];
}

void subVec3f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 3; i++) result[i] = a[i] - b[i];
}

double dotVec3f(const double* a, const double* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

void normVec3f(const double* v, double* result) {
    double mag = sqrtf(dotVec3f(v, v));
    for(int i = 0; i < 3; i++) result[i] = v[i]/mag;
}

void inv4Mat4f(const double* m, double* result) {
    double s0 = m[0]*m[5] - m[4]*m[1];
    double s1 = m[0]*m[6] - m[4]*m[2];
    double s2 = m[0]*m[7] - m[4]*m[3];
    double s3 = m[1]*m[6] - m[5]*m[2];
    double s4 = m[1]*m[7] - m[5]*m[3];
    double s5 = m[2]*m[7] - m[6]*m[3];

    double c5 = m[10]*m[15] - m[14]*m[11];
    double c4 = m[9]*m[15] - m[13]*m[11];
    double c3 = m[9]*m[14] - m[13]*m[10];
    double c2 = m[8]*m[15] - m[12]*m[11];
    double c1 = m[8]*m[14] - m[12]*m[10];
    double c0 = m[8]*m[13] - m[12]*m[9];

    double det = s0*c5 - s1*c4 + s2*c3 + s3*c2 - s4*c1 + s5*c0;
    
    if (det == 0.0) {
        // Handle error case
        return;
    }

    double invdet = 1.0/det;

    result[0] = (m[5]*c5 - m[6]*c4 + m[7]*c3)*invdet;
    result[1] = (-m[1]*c5 + m[2]*c4 - m[3]*c3)*invdet;
    result[2] = (m[13]*s5 - m[14]*s4 + m[15]*s3)*invdet;
    result[3] = (-m[9]*s5 + m[10]*s4 - m[11]*s3)*invdet;

    result[4] = (-m[4]*c5 + m[6]*c2 - m[7]*c1)*invdet;
    result[5] = (m[0]*c5 - m[2]*c2 + m[3]*c1)*invdet;
    result[6] = (-m[12]*s5 + m[14]*s2 - m[15]*s1)*invdet;
    result[7] = (m[8]*s5 - m[10]*s2 + m[11]*s1)*invdet;

    result[8] = (m[4]*c4 - m[5]*c2 + m[7]*c0)*invdet;
    result[9] = (-m[0]*c4 + m[1]*c2 - m[3]*c0)*invdet;
    result[10] = (m[12]*s4 - m[13]*s2 + m[15]*s0)*invdet;
    result[11] = (-m[8]*s4 + m[9]*s2 - m[11]*s0)*invdet;

    result[12] = (-m[4]*c3 + m[5]*c1 - m[6]*c0)*invdet;
    result[13] = (m[0]*c3 - m[1]*c1 + m[2]*c0)*invdet;
    result[14] = (-m[12]*s3 + m[13]*s1 - m[14]*s0)*invdet;
    result[15] = (m[8]*s3 - m[9]*s1 + m[10]*s0)*invdet;
}

void multMatVec4f(const double* m, const double* v, double* result) {
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2] + m[3]*v[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6]*v[2] + m[7]*v[3];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
    result[3] = m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15]*v[3];
}

void orthonormalize_rotation_matrix(double* R) {
    double x[3], y[3], z[3];
    double temp[3];
    
    // Extract columns
    for(int i = 0; i < 3; i++) {
        x[i] = R[i];      // First column
        y[i] = R[i + 3];  // Second column
        z[i] = R[i + 6];  // Third column
    }
    
    // Normalize x
    double norm_x = sqrt(dotVec3f(x, x));
    multScalVec3f(1.0/norm_x, x, x);
    
    // Make y orthogonal to x
    double dot_xy = dotVec3f(x, y);
    multScalVec3f(dot_xy, x, temp);
    subVec3f(y, temp, y);
    // Normalize y
    double norm_y = sqrt(dotVec3f(y, y));
    multScalVec3f(1.0/norm_y, y, y);
    
    // Make z orthogonal to x and y using cross product
    crossVec3f(x, y, z);
    // z is automatically normalized since x and y are orthonormal
    
    // Put back into matrix
    for(int i = 0; i < 3; i++) {
        R[i] = x[i];      // First column
        R[i + 3] = y[i];  // Second column
        R[i + 6] = z[i];  // Third column
    }
}

// Constants
#define K_F 0.0004905
#define K_M 0.00004905
#define L 0.25
#define L_SQRT2 (L / sqrtf(2.0))
#define GRAVITY 9.81
#define MASS 0.5
#define OMEGA_MIN 30.0
#define OMEGA_MAX 70.0

#define K_P 0.2
#define K_V 0.6
#define K_R 0.6
#define K_W 0.6

typedef struct {
    double omega[4];
    double linear_position_W[3];
    double linear_velocity_W[3];
    double angular_velocity_B[3];
    double R_W_B[9];
    double inertia[3];
    double omega_next[4];
    
    double accel_measurement[3];
    double gyro_measurement[3];
    double accel_bias[3];
    double gyro_bias[3];
    double accel_scale[3];
    double gyro_scale[3];
} Quad;

Quad create_quad(double x, double y, double z, double yaw) {
    Quad quad;
    
    memcpy(quad.omega, (double[]){0.0, 0.0, 0.0, 0.0}, 4 * sizeof(double));
    memcpy(quad.linear_position_W, (double[]){x, y, z}, 3 * sizeof(double));
    memcpy(quad.linear_velocity_W, (double[]){0.0, 0.0, 0.0}, 3 * sizeof(double));
    memcpy(quad.angular_velocity_B, (double[]){0.0, 0.0, 0.0}, 3 * sizeof(double));
    
    // Create rotation matrix with initial yaw
    double cos_yaw = cos(yaw);
    double sin_yaw = sin(yaw);
    double R_yaw[9] = {
        cos_yaw, 0.0, sin_yaw,
        0.0, 1.0, 0.0,
        -sin_yaw, 0.0, cos_yaw
    };
    memcpy(quad.R_W_B, R_yaw, 9 * sizeof(double));
    
    memcpy(quad.inertia, (double[]){0.01, 0.02, 0.01}, 3 * sizeof(double));
    memcpy(quad.omega_next, (double[]){0.0, 0.0, 0.0, 0.0}, 4 * sizeof(double));
    
    memset(quad.accel_measurement, 0, 3 * sizeof(double));
    memset(quad.gyro_measurement, 0, 3 * sizeof(double));
    memset(quad.accel_bias, 0, 3 * sizeof(double));
    memset(quad.gyro_bias, 0, 3 * sizeof(double));
    
    for(int i = 0; i < 3; i++) {
        quad.accel_scale[i] = ((double)rand() / RAND_MAX - 0.5) * 0.02;
        quad.gyro_scale[i] = ((double)rand() / RAND_MAX - 0.5) * 0.02;
    }
    
    return quad;
}

void update_quad_states(
    // Current state
    const double* omega,            // omega[4]
    const double* linear_position_W,// linear_position_W[3]
    const double* linear_velocity_W,// linear_velocity_W[3]
    const double* angular_velocity_B,// angular_velocity_B[3]
    const double* R_W_B,           // R_W_B[9]
    const double* inertia,         // inertia[3]
    const double* accel_bias,      // accel_bias[3]
    const double* gyro_bias,       // gyro_bias[3]
    const double* accel_scale,     // accel_scale[3]
    const double* gyro_scale,      // gyro_scale[3]
    const double* omega_next,      // omega_next[4]
    double dt,                     // time step
    double rand1,                  // First random value
    double rand2,                  // Second random value
    double rand3,                  // Third random value
    double rand4,                  // Fourth random value
    // Output
    double* new_linear_position_W, // new_linear_position_W[3]
    double* new_linear_velocity_W, // new_linear_velocity_W[3]
    double* new_angular_velocity_B,// new_angular_velocity_B[3]
    double* new_R_W_B,            // new_R_W_B[9]
    double* accel_measurement,     // accel_measurement[3]
    double* gyro_measurement,      // gyro_measurement[3]
    double* new_accel_bias,        // new_accel_bias[3]
    double* new_gyro_bias,         // new_gyro_bias[3]
    double* new_omega             // new_omega[4]
) {
    // State variables:
    // p ∈ ℝ³     Position in world frame
    // v ∈ ℝ³     Velocity in world frame
    // R ∈ SO(3)  Rotation matrix from body to world
    // ω ∈ ℝ³     Angular velocity in body frame
    // ω_i ∈ ℝ    Rotor speeds

    // Rotor forces and moments:
    // f_i = k_f * |ω_i| * ω_i    (thrust force)
    // m_i = k_m * |ω_i| * ω_i    (drag moment)
    double f[4], m[4];
    for(int i = 0; i < 4; i++) {
        double omega_sq = omega[i] * fabs(omega[i]);
        f[i] = K_F * omega_sq;
        m[i] = K_M * omega_sq;
    }

    // Net thrust and torques:
    // T = Σf_i
    // τ = Σ(r_i × F_i) + τ_drag
    double thrust = f[0] + f[1] + f[2] + f[3];
    double tau_B[3] = {0, m[0] - m[1] + m[2] - m[3], 0};
    
    const double r[4][3] = {
        {-L, 0,  L},  // Rotor 0
        { L, 0,  L},  // Rotor 1
        { L, 0, -L},  // Rotor 2
        {-L, 0, -L}   // Rotor 3
    };
    
    for(int i = 0; i < 4; i++) {
        double f_vector[3] = {0, f[i], 0};
        double tau_thrust[3];
        crossVec3f(r[i], f_vector, tau_thrust);
        addVec3f(tau_B, tau_thrust, tau_B);
    }

    // Linear dynamics:
    // p̈ = 1/m * F_W + [0; -g; 0]
    // where F_W = R_W_B * [0; T; 0]
    double f_B_thrust[3] = {0, thrust, 0};
    double f_thrust_W[3];
    multMatVec3f(R_W_B, f_B_thrust, f_thrust_W);

    double linear_acceleration_W[3];
    for(int i = 0; i < 3; i++) {
        linear_acceleration_W[i] = f_thrust_W[i] / MASS;
    }
    linear_acceleration_W[1] -= GRAVITY;

    // State evolution:
    // v(t+dt) = v(t) + dt * v̇(t)
    // p(t+dt) = p(t) + dt * v(t+dt)
    for(int i = 0; i < 3; i++) {
        new_linear_velocity_W[i] = linear_velocity_W[i] + dt * linear_acceleration_W[i];
        new_linear_position_W[i] = linear_position_W[i] + dt * new_linear_velocity_W[i];
    }

    if (new_linear_position_W[1] < 0.0) new_linear_position_W[1] = 0.0;

    // Angular dynamics:
    // ω̇ = I⁻¹(τ_B - ω × (Iω))
    double I_mat[9];
    vecToDiagMat3f(inertia, I_mat);
    
    double h_B[3], w_cross_h[3];
    multMatVec3f(I_mat, angular_velocity_B, h_B);
    crossVec3f(angular_velocity_B, h_B, w_cross_h);

    // State evolution (Euler integration):
    // ω(t+dt) = ω(t) + dt * ω̇(t)
    for(int i = 0; i < 3; i++) {
        double angular_acc = (-w_cross_h[i] + tau_B[i]) / inertia[i];
        new_angular_velocity_B[i] = angular_velocity_B[i] + dt * angular_acc;
    }

    // Rotation dynamics:
    // Ṙ = R[ω]ₓ
    // where [ω]ₓ is the skew-symmetric matrix:
    // [ω]ₓ = [ 0   -ω₃   ω₂ ]
    //        [ ω₃   0   -ω₁ ]
    //        [-ω₂   ω₁   0  ]
    double w_hat[9];
    so3hat(angular_velocity_B, w_hat);
    double R_dot[9];
    multMat3f(R_W_B, w_hat, R_dot);

    // State evolution (Euler integration):
    // R(t+dt) = R(t) + dt * Ṙ(t)
    double R_dot_scaled[9];
    multScalMat3f(dt, R_dot, R_dot_scaled);
    addMat3f(R_W_B, R_dot_scaled, new_R_W_B);
    orthonormalize_rotation_matrix(new_R_W_B);

    // Update IMU measurements
    // Convert world frame acceleration to body frame for accelerometer
    double R_W_B_T[9];
    transpMat3f(new_R_W_B, R_W_B_T);
    multMatVec3f(R_W_B_T, linear_acceleration_W, accel_measurement);

    // Add gravity in body frame
    double gravity_B[3];
    multMatVec3f(R_W_B_T, (double[]){0, -GRAVITY, 0}, gravity_B);
    addVec3f(accel_measurement, gravity_B, accel_measurement);

    // Use the first two random values for accel and gyro bias updates
    double accel_walk_noise = (rand1 - 0.5) * 0.0001;
    double gyro_walk_noise = (rand2 - 0.5) * 0.0001;

    // Use the second two random values for accel and gyro measurement noise
    double accel_meas_noise = (rand3 - 0.5) * 0.01;
    double gyro_meas_noise = (rand4 - 0.5) * 0.01;

    // Update bias random walk and add noise to accelerometer
    for(int i = 0; i < 3; i++) {
        // Update bias with random walk
        new_accel_bias[i] = accel_bias[i] + accel_walk_noise * dt;
        // Apply scale factor error, add bias and white noise
        accel_measurement[i] = accel_measurement[i] * (1.0 + accel_scale[i]) + 
                                new_accel_bias[i] + 
                                accel_meas_noise;
    }

    // Update gyroscope measurements
    memcpy(gyro_measurement, new_angular_velocity_B, 3 * sizeof(double));
    for(int i = 0; i < 3; i++) {
        // Update bias with random walk
        new_gyro_bias[i] = gyro_bias[i] + gyro_walk_noise * dt;
        // Apply scale factor error, add bias and white noise
        gyro_measurement[i] = gyro_measurement[i] * (1.0 + gyro_scale[i]) + 
                                new_gyro_bias[i] + 
                                gyro_meas_noise;
    }

    // Rotor speed update with saturation:
    // ω_i(t+dt) = clamp(ω_i_next, ω_min, ω_max)
    for(int i = 0; i < 4; i++) {
        new_omega[i] = fmax(OMEGA_MIN, fmin(OMEGA_MAX, omega_next[i]));
    }
}

void control_quad_commands(
    // Current state
    const double* position,     // linear_position_W[3]
    const double* velocity,     // linear_velocity_W[3]
    const double* R_W_B,       // R_W_B[9]
    const double* omega,        // angular_velocity_B[3]
    const double* inertia,     // inertia[3]
    // Target state
    const double* control_input,// target state[7]
    // Output
    double* omega_next         // omega_next[4]
) {
    // 1. Calculate position and velocity errors
    double error_p[3], error_v[3];
    subVec3f(position, (double[]){control_input[0], control_input[1], control_input[2]}, error_p);
    subVec3f(velocity, (double[]){control_input[3], control_input[4], control_input[5]}, error_v);

    // 2. Calculate desired force vector in world frame
    double z_W_d[3], temp[3];
    multScalVec3f(-K_P, error_p, z_W_d);
    multScalVec3f(-K_V, error_v, temp);
    addVec3f(z_W_d, temp, z_W_d);
    
    // Add gravity compensation and desired acceleration
    double gravity_term[3] = {0, MASS * GRAVITY, 0};
    addVec3f(z_W_d, gravity_term, z_W_d);
    
    double accel_term[3];
    multScalVec3f(MASS, (double[]){0.0, 0.0, 0.0}, accel_term);
    addVec3f(z_W_d, accel_term, z_W_d);

    // 3. Calculate thrust magnitude
    double z_W_B[3];
    double y_body[3] = {0, 1, 0};
    multMatVec3f(R_W_B, y_body, z_W_B);
    double thrust = dotVec3f(z_W_d, z_W_B);

    // 4. Calculate desired rotation matrix
    double x_tilde_d_W[3] = {sin(control_input[6]), 0.0, cos(control_input[6])};
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
    multMatVec3f(temp_mat1, (double[]){0.0, 0.0, 0.0}, temp_vec);
    subVec3f(omega, temp_vec, error_w);

    // 7. Calculate control torque
    double tau_B_control[3], temp_vec2[3];
    multScalVec3f(-K_R, error_r, tau_B_control);
    multScalVec3f(-K_W, error_w, temp_vec2);
    addVec3f(tau_B_control, temp_vec2, tau_B_control);

    // Add angular momentum terms
    double I_mat[9], temp_vec3[3], temp_vec4[3];
    vecToDiagMat3f(inertia, I_mat);
    multMatVec3f(I_mat, omega, temp_vec3);
    crossVec3f(omega, temp_vec3, temp_vec4);
    addVec3f(tau_B_control, temp_vec4, tau_B_control);

    // Add feedforward terms
    double term_0[3], term_1[3], temp_vec5[3];
    multMatVec3f(R_W_d, (double[]){0.0, 0.0, 0.0}, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, term_0);

    multMatVec3f(R_W_d, (double[]){0.0, 0.0, 0.0}, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, temp_vec2);
    crossVec3f((double[]){0.0, 0.0, 0.0}, temp_vec2, term_1);

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
    double omega_sign_square[4];
    multMatVec4f(F_bar_inv, (double[]){thrust, tau_B_control[0], tau_B_control[1], tau_B_control[2]}, omega_sign_square);

    for(int i = 0; i < 4; i++) {
        omega_next[i] = sqrt(fabs(omega_sign_square[i]));
    }
}

typedef struct {
    double R[9];                    // Estimated rotation matrix
    double angular_velocity[3];     // Estimated angular velocity
    double gyro_bias[3];           // Estimated gyro bias
} StateEstimator;

void update_estimator(
    const double *gyro, 
    const double *accel, 
    double dt, 
    StateEstimator *state
) {
    // Correction gains
    const double k_R = 0.1;    // Attitude correction gain
    const double k_angular = 2.0; // Angular velocity correction gain
    const double k_bias = 0.01; // Bias estimation gain

    // 1. Normalize accelerometer reading
    double acc_norm = sqrt(dotVec3f(accel, accel));
    double a_norm[3] = {
        accel[0] / acc_norm,
        accel[1] / acc_norm,
        accel[2] / acc_norm
    };

    // 2. Calculate error between measured and expected gravity direction
    double g_body[3];
    double R_T[9];
    transpMat3f(state->R, R_T);
    multMatVec3f(R_T, (double[]){0, -1, 0}, g_body);
    
    double error[3];
    crossVec3f(a_norm, g_body, error);
    
    // 3. Update bias estimate
    for(int i = 0; i < 3; i++) {
        state->gyro_bias[i] -= k_bias * error[i] * dt;
    }

    // 4. Apply corrections to angular velocity estimate
    for(int i = 0; i < 3; i++) {
        // Remove bias from gyro measurement
        double unbiased_gyro = gyro[i] - state->gyro_bias[i];
        // Update angular velocity estimate with bias-corrected gyro and attitude error
        state->angular_velocity[i] = unbiased_gyro + k_angular * error[i];
    }
    
    // 5. Update rotation matrix
    double angular_velocity_hat[9];
    so3hat(state->angular_velocity, angular_velocity_hat);
    double R_dot[9];
    multMat3f(state->R, angular_velocity_hat, R_dot);
    
    // Add attitude correction term
    double correction[9];
    so3hat(error, correction);
    multScalMat3f(k_R, correction, correction);
    addMat3f(R_dot, correction, R_dot);
    
    // Integrate and orthonormalize
    for(int i = 0; i < 9; i++) {
        state->R[i] += dt * R_dot[i];
    }
    orthonormalize_rotation_matrix(state->R);
}

#endif // QUAD_H