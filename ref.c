#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

void multScalMat3f(double s, const double* m, double* result) {
    for(int i = 0; i < 9; i++) {
        result[i] = s * m[i];
    }
}

void addMat3f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 9; i++) {
        result[i] = a[i] + b[i];
    }
}

void subMat3f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 9; i++) {
        result[i] = a[i] - b[i];
    }
}

int inv4Mat4f(const double* m, double* result) {
    double det = m[0] * (
        m[5] * (m[10] * m[15] - m[11] * m[14]) -
        m[9] * (m[6] * m[15] - m[7] * m[14]) +
        m[13] * (m[6] * m[11] - m[7] * m[10])
    ) - m[4] * (
        m[1] * (m[10] * m[15] - m[11] * m[14]) -
        m[9] * (m[2] * m[15] - m[3] * m[14]) +
        m[13] * (m[2] * m[11] - m[3] * m[10])
    ) + m[8] * (
        m[1] * (m[6] * m[15] - m[7] * m[14]) -
        m[5] * (m[2] * m[15] - m[3] * m[14]) +
        m[13] * (m[2] * m[7] - m[3] * m[6])
    ) - m[12] * (
        m[1] * (m[6] * m[11] - m[7] * m[10]) -
        m[5] * (m[2] * m[11] - m[3] * m[10]) +
        m[9] * (m[2] * m[7] - m[3] * m[6])
    );

    if (fabs(det) < 1e-10) {
        printf("Matrix is singular or nearly singular: det=%e\n", det);
        return 0;
    }

    double invDet = 1.0 / det;

    // Adjugate matrix
    result[0] = invDet * (
        m[5] * (m[10] * m[15] - m[11] * m[14]) -
        m[9] * (m[6] * m[15] - m[7] * m[14]) +
        m[13] * (m[6] * m[11] - m[7] * m[10])
    );
    result[1] = -invDet * (
        m[1] * (m[10] * m[15] - m[11] * m[14]) -
        m[9] * (m[2] * m[15] - m[3] * m[14]) +
        m[13] * (m[2] * m[11] - m[3] * m[10])
    );
    result[2] = invDet * (
        m[1] * (m[6] * m[15] - m[7] * m[14]) -
        m[5] * (m[2] * m[15] - m[3] * m[14]) +
        m[13] * (m[2] * m[7] - m[3] * m[6])
    );
    result[3] = -invDet * (
        m[1] * (m[6] * m[11] - m[7] * m[10]) -
        m[5] * (m[2] * m[11] - m[3] * m[10]) +
        m[9] * (m[2] * m[7] - m[3] * m[6])
    );

    result[4] = -invDet * (
        m[4] * (m[10] * m[15] - m[11] * m[14]) -
        m[8] * (m[6] * m[15] - m[7] * m[14]) +
        m[12] * (m[6] * m[11] - m[7] * m[10])
    );
    result[5] = invDet * (
        m[0] * (m[10] * m[15] - m[11] * m[14]) -
        m[8] * (m[2] * m[15] - m[3] * m[14]) +
        m[12] * (m[2] * m[11] - m[3] * m[10])
    );
    result[6] = -invDet * (
        m[0] * (m[6] * m[15] - m[7] * m[14]) -
        m[4] * (m[2] * m[15] - m[3] * m[14]) +
        m[12] * (m[2] * m[7] - m[3] * m[6])
    );
    result[7] = invDet * (
        m[0] * (m[6] * m[11] - m[7] * m[10]) -
        m[4] * (m[2] * m[11] - m[3] * m[10]) +
        m[8] * (m[2] * m[7] - m[3] * m[6])
    );

    result[8] = invDet * (
        m[4] * (m[9] * m[15] - m[11] * m[13]) -
        m[8] * (m[5] * m[15] - m[7] * m[13]) +
        m[12] * (m[5] * m[11] - m[7] * m[9])
    );
    result[9] = -invDet * (
        m[0] * (m[9] * m[15] - m[11] * m[13]) -
        m[8] * (m[1] * m[15] - m[3] * m[13]) +
        m[12] * (m[1] * m[11] - m[3] * m[9])
    );
    result[10] = invDet * (
        m[0] * (m[5] * m[15] - m[7] * m[13]) -
        m[4] * (m[1] * m[15] - m[3] * m[13]) +
        m[12] * (m[1] * m[7] - m[3] * m[5])
    );
    result[11] = -invDet * (
        m[0] * (m[5] * m[11] - m[7] * m[9]) -
        m[4] * (m[1] * m[11] - m[3] * m[9]) +
        m[8] * (m[1] * m[7] - m[3] * m[5])
    );

    result[12] = -invDet * (
        m[4] * (m[9] * m[14] - m[10] * m[13]) -
        m[8] * (m[5] * m[14] - m[6] * m[13]) +
        m[12] * (m[5] * m[10] - m[6] * m[9])
    );
    result[13] = invDet * (
        m[0] * (m[9] * m[14] - m[10] * m[13]) -
        m[8] * (m[1] * m[14] - m[2] * m[13]) +
        m[12] * (m[1] * m[10] - m[2] * m[9])
    );
    result[14] = -invDet * (
        m[0] * (m[5] * m[14] - m[6] * m[13]) -
        m[4] * (m[1] * m[14] - m[2] * m[13]) +
        m[12] * (m[1] * m[6] - m[2] * m[5])
    );
    result[15] = invDet * (
        m[0] * (m[5] * m[10] - m[6] * m[9]) -
        m[4] * (m[1] * m[10] - m[2] * m[9]) +
        m[8] * (m[1] * m[6] - m[2] * m[5])
    );

    return 1;
}

void multMatVec4f(const double* m, const double* v, double* result) {
    for(int i = 0; i < 4; i++) {
        result[i] = m[i*4] * v[0] + m[i*4+1] * v[1] + 
                    m[i*4+2] * v[2] + m[i*4+3] * v[3];
    }
}

void so3hat(const double* v, double* result) {
    result[0] = 0.0f;   result[1] = -v[2];  result[2] = v[1];
    result[3] = v[2];   result[4] = 0.0f;   result[5] = -v[0];
    result[6] = -v[1];  result[7] = v[0];   result[8] = 0.0f;
}

void so3vee(const double* m, double* result) {
    result[0] = m[7];
    result[1] = m[2];
    result[2] = m[3];
}

void multMat3f(const double* a, const double* b, double* result) {
    result[0] = a[0] * b[0] + a[1] * b[3] + a[2] * b[6];
    result[1] = a[0] * b[1] + a[1] * b[4] + a[2] * b[7];
    result[2] = a[0] * b[2] + a[1] * b[5] + a[2] * b[8];

    result[3] = a[3] * b[0] + a[4] * b[3] + a[5] * b[6];
    result[4] = a[3] * b[1] + a[4] * b[4] + a[5] * b[7];
    result[5] = a[3] * b[2] + a[4] * b[5] + a[5] * b[8];

    result[6] = a[6] * b[0] + a[7] * b[3] + a[8] * b[6];
    result[7] = a[6] * b[1] + a[7] * b[4] + a[8] * b[7];
    result[8] = a[6] * b[2] + a[7] * b[5] + a[8] * b[8];
}

void multMatVec3f(const double* m, const double* v, double* result) {
    result[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    result[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    result[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

void vecToDiagMat3f(const double* v, double* result) {
    result[0] = v[0]; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = v[1]; result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = v[2];
}

int invMat3f(const double* m, double* result) {
    double det = 
        m[0] * (m[4] * m[8] - m[7] * m[5]) -
        m[1] * (m[3] * m[8] - m[5] * m[6]) +
        m[2] * (m[3] * m[7] - m[4] * m[6]);

    if (det == 0) {
        return 0;  // Matrix is not invertible
    }

    double invDet = 1.0f / det;

    result[0] = invDet * (m[4] * m[8] - m[7] * m[5]);
    result[1] = invDet * (m[2] * m[7] - m[1] * m[8]);
    result[2] = invDet * (m[1] * m[5] - m[2] * m[4]);
    result[3] = invDet * (m[5] * m[6] - m[3] * m[8]);
    result[4] = invDet * (m[0] * m[8] - m[2] * m[6]);
    result[5] = invDet * (m[3] * m[2] - m[0] * m[5]);
    result[6] = invDet * (m[3] * m[7] - m[6] * m[4]);
    result[7] = invDet * (m[6] * m[1] - m[0] * m[7]);
    result[8] = invDet * (m[0] * m[4] - m[3] * m[1]);

    return 1;  // Success
}

void transpMat3f(const double* m, double* result) {
    result[0] = m[0]; result[1] = m[3]; result[2] = m[6];
    result[3] = m[1]; result[4] = m[4]; result[5] = m[7];
    result[6] = m[2]; result[7] = m[5]; result[8] = m[8];
}

void identMat3f(double* result) {
    result[0] = 1.0f; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = 1.0f; result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = 1.0f;
}

void xRotMat3f(double rads, double* result) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = 1.0f; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = c;    result[5] = -s;
    result[6] = 0.0f; result[7] = s;    result[8] = c;
}

void yRotMat3f(double rads, double* result) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = c;    result[1] = 0.0f; result[2] = s;
    result[3] = 0.0f; result[4] = 1.0f; result[5] = 0.0f;
    result[6] = -s;   result[7] = 0.0f; result[8] = c;
}

void zRotMat3f(double rads, double* result) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = c;    result[1] = -s;   result[2] = 0.0f;
    result[3] = s;    result[4] = c;    result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = 1.0f;
}

// Vector operations
void crossVec3f(const double* v1, const double* v2, double* result) {
    result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void multScalVec3f(double s, const double* v, double* result) {
    result[0] = v[0] * s;
    result[1] = v[1] * s;
    result[2] = v[2] * s;
}

void addVec3f(const double* v1, const double* v2, double* result) {
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
}

void subVec3f(const double* v1, const double* v2, double* result) {
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
}

double dotVec3f(const double* v1, const double* v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void normVec3f(const double* v, double* result) {
    double magnitude = sqrtf(dotVec3f(v, v));
    result[0] = v[0] / magnitude;
    result[1] = v[1] / magnitude;
    result[2] = v[2] / magnitude;
}

// Constants
const double k_f = 0.0004905f;
const double k_m = 0.00004905f;
const double L = 0.25f;
const double l = (L / sqrtf(2.0f));
const double I[3] = {0.01f, 0.02f, 0.01f};
const double g = 9.81f;
const double m = 0.5f;
const double dt = 0.01f;
const double omega_min = 30.0f;
const double omega_max = 70.0f;
const double omega_stable = 50.0f;

// Global state variables
double omega_1, omega_2, omega_3, omega_4;
double angular_velocity_B[3];
double linear_velocity_W[3];
double linear_position_W[3];
double R_W_B[9];
double loc_I_mat[9];
double loc_I_mat_inv[9];

void init_state() {
    // Initialize motor speeds
    omega_1 = omega_stable;
    omega_2 = omega_stable;
    omega_3 = omega_stable;
    omega_4 = omega_stable;

    // Initialize velocities and position
    for(int i = 0; i < 3; i++) {
        angular_velocity_B[i] = 0.0f;
        linear_velocity_W[i] = 0.0f;
        linear_position_W[i] = 0.0f;
    }
    linear_position_W[1] = 1.0f;  // Initial height

    // Initialize rotation matrix
    double temp_x[9], temp_y[9], temp_z[9], temp[9];
    xRotMat3f(0.0f, temp_x);
    yRotMat3f(0.0f, temp_y);
    zRotMat3f(0.0f, temp_z);
    multMat3f(temp_x, temp_y, temp);
    multMat3f(temp, temp_z, R_W_B);

    // Initialize inertia matrices
    vecToDiagMat3f(I, loc_I_mat);
    invMat3f(loc_I_mat, loc_I_mat_inv);
}

void update_dynamics() {
    // Limit motor speeds
    omega_1 = fmaxf(fminf(omega_1, omega_max), omega_min);
    omega_2 = fmaxf(fminf(omega_2, omega_max), omega_min);
    omega_3 = fmaxf(fminf(omega_3, omega_max), omega_min);
    omega_4 = fmaxf(fminf(omega_4, omega_max), omega_min);

    // Forces and moments
    double F1 = k_f * omega_1 * fabsf(omega_1);
    double F2 = k_f * omega_2 * fabsf(omega_2);
    double F3 = k_f * omega_3 * fabsf(omega_3);
    double F4 = k_f * omega_4 * fabsf(omega_4);

    double M1 = k_m * omega_1 * fabsf(omega_1);
    double M2 = k_m * omega_2 * fabsf(omega_2);
    double M3 = k_m * omega_3 * fabsf(omega_3);
    double M4 = k_m * omega_4 * fabsf(omega_4);

    // Thrust
    double f_B_thrust[3] = {0.0f, F1 + F2 + F3 + F4, 0.0f};

    // Torque calculations
    double tau_B_drag[3] = {0.0f, M1 - M2 + M3 - M4, 0.0f};
    
    double p1[3] = {-L, 0.0f, L};
    double p2[3] = {L, 0.0f, L};
    double p3[3] = {L, 0.0f, -L};
    double p4[3] = {-L, 0.0f, -L};
    
    double f1[3] = {0.0f, F1, 0.0f};
    double f2[3] = {0.0f, F2, 0.0f};
    double f3[3] = {0.0f, F3, 0.0f};
    double f4[3] = {0.0f, F4, 0.0f};

    double tau_B_thrust_1[3], tau_B_thrust_2[3], tau_B_thrust_3[3], tau_B_thrust_4[3];
    double tau_B_thrust[3], tau_B[3];

    crossVec3f(p1, f1, tau_B_thrust_1);
    crossVec3f(p2, f2, tau_B_thrust_2);
    crossVec3f(p3, f3, tau_B_thrust_3);
    crossVec3f(p4, f4, tau_B_thrust_4);

    addVec3f(tau_B_thrust_1, tau_B_thrust_2, tau_B_thrust);
    double temp[3];
    addVec3f(tau_B_thrust_3, tau_B_thrust_4, temp);
    addVec3f(tau_B_thrust, temp, tau_B_thrust);
    addVec3f(tau_B_drag, tau_B_thrust, tau_B);

    // Accelerations
    double gravity_force[3] = {0.0f, -g * m, 0.0f};
    double thrust_W[3];
    multMatVec3f(R_W_B, f_B_thrust, thrust_W);
    double linear_acceleration_W[3];
    addVec3f(gravity_force, thrust_W, linear_acceleration_W);
    multScalVec3f(1.0f/m, linear_acceleration_W, linear_acceleration_W);

    // Angular acceleration
    double temp1[3], temp2[3], angular_acceleration_B[3];
    multMatVec3f(loc_I_mat, angular_velocity_B, temp1);
    crossVec3f(angular_velocity_B, temp1, temp2);
    multScalVec3f(-1.0f, temp2, temp2);
    addVec3f(temp2, tau_B, angular_acceleration_B);
    
    angular_acceleration_B[0] /= I[0];
    angular_acceleration_B[1] /= I[1];
    angular_acceleration_B[2] /= I[2];

    // State update
    double vel_increment[3], pos_increment[3], ang_vel_increment[3];
    multScalVec3f(dt, linear_acceleration_W, vel_increment);
    addVec3f(linear_velocity_W, vel_increment, linear_velocity_W);

    multScalVec3f(dt, linear_velocity_W, pos_increment);
    addVec3f(linear_position_W, pos_increment, linear_position_W);

    multScalVec3f(dt, angular_acceleration_B, ang_vel_increment);
    addVec3f(angular_velocity_B, ang_vel_increment, angular_velocity_B);

    // Rotation matrix update
    double omega_hat[9], temp_mat[9], increment[9];
    so3hat(angular_velocity_B, omega_hat);
    multMat3f(R_W_B, omega_hat, temp_mat);
    multScalMat3f(dt, temp_mat, increment);
    addMat3f(R_W_B, increment, R_W_B);
}

// Control parameters
double linear_position_d_W[3] = {2.0f, 2.0f, 2.0f};
double linear_velocity_d_W[3] = {0.0f, 0.0f, 0.0f};
double linear_acceleration_d_W[3] = {0.0f, 0.0f, 0.0f};
double angular_velocity_d_B[3] = {0.0f, 0.0f, 0.0f};
double angular_acceleration_d_B[3] = {0.0f, 0.0f, 0.0f};
double yaw_d = 0.0f;

const double k_p = 0.05f;
const double k_v = 0.5f;
const double k_R = 0.5f;
const double k_w = 0.5f;

void update_control() {
    // --- LINEAR CONTROL ---
    double error_p[3], error_v[3], z_W_d[3], z_W_B[3];
    double temp_vec[3], temp_vec2[3];

    subVec3f(linear_position_W, linear_position_d_W, error_p);
    subVec3f(linear_velocity_W, linear_velocity_d_W, error_v);

    // Calculate z_W_d
    multScalVec3f(-k_p, error_p, z_W_d);
    multScalVec3f(-k_v, error_v, temp_vec);
    addVec3f(z_W_d, temp_vec, z_W_d);
    addVec3f(z_W_d, (double[3]){0.0f, m * g, 0.0f}, z_W_d);
    multScalVec3f(m, linear_acceleration_d_W, temp_vec);
    addVec3f(z_W_d, temp_vec, z_W_d);

    // Calculate f_z_B_control
    double y_axis_B[3] = {0.0f, 1.0f, 0.0f};
    multMatVec3f(R_W_B, y_axis_B, z_W_B);
    double f_z_B_control = dotVec3f(z_W_d, z_W_B);

    // --- ATTITUDE CONTROL ---
    double x_tilde_d_W[3] = {sinf(yaw_d), 0.0f, cosf(yaw_d)};
    double R_W_d_column_0[3], R_W_d_column_1[3], R_W_d_column_2[3];
    double temp_cross1[3], temp_cross2[3];

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

    // Calculate error_r and error_w
    double R_W_d_T[9], R_W_B_T[9], temp_mat1[9], temp_mat2[9], diff_mat[9];
    double error_r[3], error_w[3];

    transpMat3f(R_W_d, R_W_d_T);
    transpMat3f(R_W_B, R_W_B_T);
    multMat3f(R_W_d_T, R_W_B, temp_mat1);
    multMat3f(R_W_B_T, R_W_d, temp_mat2);
    subMat3f(temp_mat1, temp_mat2, diff_mat);
    so3vee(diff_mat, error_r);
    multScalVec3f(0.5f, error_r, error_r);

    multMat3f(R_W_d_T, R_W_B, temp_mat1);
    multMatVec3f(temp_mat1, angular_velocity_d_B, temp_vec);
    subVec3f(angular_velocity_B, temp_vec, error_w);

    // Calculate tau_B_control
    double tau_B_control[3];
    multScalVec3f(-k_R, error_r, tau_B_control);
    multScalVec3f(-k_w, error_w, temp_vec);
    addVec3f(tau_B_control, temp_vec, tau_B_control);

    multMatVec3f(loc_I_mat, angular_velocity_B, temp_vec);
    crossVec3f(angular_velocity_B, temp_vec, temp_vec2);
    addVec3f(tau_B_control, temp_vec2, tau_B_control);

    // Calculate term_0 and term_1
    double term_0[3], term_1[3];
    multMatVec3f(R_W_d, angular_acceleration_d_B, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, term_0);

    multMatVec3f(R_W_d, angular_velocity_d_B, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, temp_vec2);
    crossVec3f(angular_velocity_B, temp_vec2, term_1);

    subVec3f(term_1, term_0, temp_vec);
    multMatVec3f(loc_I_mat, temp_vec, temp_vec2);
    subVec3f(tau_B_control, temp_vec2, tau_B_control);

    // Scale factors to improve numerical conditioning
    const double SCALE_F = 1e6;  // Scale factor for forces
    const double SCALE_M = 1e6;  // Scale factor for moments
    const double SCALE_W = 1e2;  // Scale factor for omega calculation

    // Construct scaled F_bar matrix
    double F_bar[16] = {
        k_f * SCALE_F,    k_f * SCALE_F,    k_f * SCALE_F,    k_f * SCALE_F,
        L*k_f * SCALE_M,  L*k_f * SCALE_M, -L*k_f * SCALE_M, -L*k_f * SCALE_M,
        -L*k_f * SCALE_M, L*k_f * SCALE_M,  L*k_f * SCALE_M, -L*k_f * SCALE_M,
        k_m * SCALE_M,   -k_m * SCALE_M,    k_m * SCALE_M,   -k_m * SCALE_M
    };

    // Scale the control input vector
    double scaled_control_input[4] = {
        f_z_B_control * SCALE_F,
        tau_B_control[0] * SCALE_M,
        tau_B_control[1] * SCALE_M,
        tau_B_control[2] * SCALE_M
    };

    // Print debug information
    printf("Control inputs before scaling: f_z=%f, tau=[%f, %f, %f]\n",
           f_z_B_control, tau_B_control[0], tau_B_control[1], tau_B_control[2]);

    printf("Scaled F_bar matrix:\n");
    for(int i = 0; i < 16; i++) {
        printf("%f ", F_bar[i]);
        if((i+1) % 4 == 0) printf("\n");
    }

    // Calculate and print determinant
    double det = F_bar[0] * (F_bar[5] * F_bar[10] * F_bar[15] - F_bar[5] * F_bar[11] * F_bar[14] -
                            F_bar[9] * F_bar[6] * F_bar[15] + F_bar[9] * F_bar[7] * F_bar[14] +
                            F_bar[13] * F_bar[6] * F_bar[11] - F_bar[13] * F_bar[7] * F_bar[10]);
    printf("F_bar determinant: %e\n", det);

    // Invert F_bar matrix
    double F_bar_inv[16];
    if (!inv4Mat4f(F_bar, F_bar_inv)) {
        printf("Matrix inversion failed!\n");
        omega_1 = omega_2 = omega_3 = omega_4 = omega_stable;
        return;
    }

    // Calculate omega_sign_square with scaling compensation
    double omega_sign_square[4];
    multMatVec4f(F_bar_inv, scaled_control_input, omega_sign_square);

    // Compensate for scaling and add base speed offset
    const double omega_base = omega_stable * omega_stable;  // Square of base speed
    for(int i = 0; i < 4; i++) {
        omega_sign_square[i] = omega_base + (omega_sign_square[i] / SCALE_F) * SCALE_W;
    }

    printf("Final omega_sign_square: %f %f %f %f\n", 
           omega_sign_square[0], omega_sign_square[1], 
           omega_sign_square[2], omega_sign_square[3]);

    // Calculate final rotor speeds
    omega_1 = sqrt(fabs(omega_sign_square[0]));
    omega_2 = sqrt(fabs(omega_sign_square[1]));
    omega_3 = sqrt(fabs(omega_sign_square[2]));
    omega_4 = sqrt(fabs(omega_sign_square[3]));

    // Print rotor speeds before limiting
    printf("Rotor speeds before limiting: %f %f %f %f\n", 
           omega_1, omega_2, omega_3, omega_4);

    // Limit rotor speeds
    omega_1 = fmax(fmin(omega_1, omega_max), omega_min);
    omega_2 = fmax(fmin(omega_2, omega_max), omega_min);
    omega_3 = fmax(fmin(omega_3, omega_max), omega_min);
    omega_4 = fmax(fmin(omega_4, omega_max), omega_min);
}

// Global variables for timing
time_t start_time;
bool should_exit = false;

// Function to print the current state
void print_state() {
    time_t current_time;
    char time_str[26];
    time(&current_time);
    ctime_r(&current_time, time_str);
    time_str[24] = '\0';  // Remove newline

    printf("Time: %s\n", time_str);
    printf("Linear Position: [%.3f, %.3f, %.3f]\n", 
           linear_position_W[0], linear_position_W[1], linear_position_W[2]);
    printf("Linear Velocity: [%.3f, %.3f, %.3f]\n", 
           linear_velocity_W[0], linear_velocity_W[1], linear_velocity_W[2]);
    printf("Angular Velocity: [%.3f, %.3f, %.3f]\n", 
           angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2]);
    printf("Rotor Speeds: %.3f, %.3f, %.3f, %.3f\n", 
           omega_1, omega_2, omega_3, omega_4);
    printf("\n");
}

// Function to check if target position is reached
bool is_target_reached() {
    return (fabsf(linear_position_W[0] - linear_position_d_W[0]) < 0.1f &&
            fabsf(linear_position_W[1] - linear_position_d_W[1]) < 0.1f &&
            fabsf(linear_position_W[2] - linear_position_d_W[2]) < 0.1f);
}

// Signal handler for timeout
void timeout_handler(int signum) {
    printf("\nTimeout reached (5 seconds). Exiting...\n");
    should_exit = true;
}

int main() {
    // Initialize state
    init_state();

    // Set up timeout signal handler
    signal(SIGALRM, timeout_handler);
    alarm(5);  // Set 5-second timeout

    // Record start time
    time(&start_time);

    // Variables for timing control
    struct timespec last_print_time, current_time;
    clock_gettime(CLOCK_MONOTONIC, &last_print_time);

    while (!should_exit) {
        // Update dynamics and control
        update_dynamics();
        update_control();

        // Get current time
        clock_gettime(CLOCK_MONOTONIC, &current_time);

        // Check if 100ms has passed since last print
        double time_diff = (current_time.tv_sec - last_print_time.tv_sec) +
                          (current_time.tv_nsec - last_print_time.tv_nsec) / 1e9;

        if (time_diff >= 0.1) {  // 100ms
            print_state();
            last_print_time = current_time;
        }

        // Check if target is reached
        if (is_target_reached()) {
            printf("\nTarget position reached! Exiting...\n");
            break;
        }

        // Sleep for dt seconds
        usleep(dt * 1000000);
    }

    return 0;
}