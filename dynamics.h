#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

// Vector and matrix operations
void crossVec3f(const double v1[3], const double v2[3], double result[3]) {
    result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void multScalVec3f(double s, const double v[3], double result[3]) {
    result[0] = v[0] * s;
    result[1] = v[1] * s;
    result[2] = v[2] * s;
}

void addVec3f(const double v1[3], const double v2[3], double result[3]) {
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
}

void multMat3f(const double a[9], const double b[9], double result[9]) {
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

void multMatVec3f(const double m[9], const double v[3], double result[3]) {
    result[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    result[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    result[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

void vecToDiagMat3f(const double v[3], double result[9]) {
    result[0] = v[0]; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = v[1]; result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = v[2];
}

void xRotMat3f(double rads, double result[9]) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = 1.0f; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = c;    result[5] = -s;
    result[6] = 0.0f; result[7] = s;    result[8] = c;
}

void yRotMat3f(double rads, double result[9]) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = c;    result[1] = 0.0f; result[2] = s;
    result[3] = 0.0f; result[4] = 1.0f; result[5] = 0.0f;
    result[6] = -s;   result[7] = 0.0f; result[8] = c;
}

void zRotMat3f(double rads, double result[9]) {
    double s = sinf(rads);
    double c = cosf(rads);
    result[0] = c;    result[1] = -s;   result[2] = 0.0f;
    result[3] = s;    result[4] = c;    result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = 1.0f;
}

void so3hat(const double v[3], double result[9]) {
    result[0] = 0.0f;  result[1] = -v[2]; result[2] = v[1];
    result[3] = v[2];  result[4] = 0.0f;  result[5] = -v[0];
    result[6] = -v[1]; result[7] = v[0];  result[8] = 0.0f;
}

void addMat3f(const double a[9], const double b[9], double result[9]) {
    for(int i = 0; i < 9; i++) {
        result[i] = a[i] + b[i];
    }
}

void multScalMat3f(double s, const double m[9], double result[9]) {
    for(int i = 0; i < 9; i++) {
        result[i] = s * m[i];
    }
}

void orthonormalize_rotation_matrix(double R[9]) {
    // Gram-Schmidt orthogonalization
    double x[3] = {R[0], R[3], R[6]};
    double y[3] = {R[1], R[4], R[7]};
    double z[3] = {R[2], R[5], R[8]};
    
    // Normalize x
    double len = sqrt(x[0]*x[0] + x[1]*x[1] + x[2]*x[2]);
    if (len > 0) {
        x[0] /= len; x[1] /= len; x[2] /= len;
    }
    
    // Make y orthogonal to x
    double dot = x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
    y[0] -= dot*x[0]; y[1] -= dot*x[1]; y[2] -= dot*x[2];
    
    // Normalize y
    len = sqrt(y[0]*y[0] + y[1]*y[1] + y[2]*y[2]);
    if (len > 0) {
        y[0] /= len; y[1] /= len; y[2] /= len;
    }
    
    // z = x × y
    z[0] = x[1]*y[2] - x[2]*y[1];
    z[1] = x[2]*y[0] - x[0]*y[2];
    z[2] = x[0]*y[1] - x[1]*y[0];
    
    // Store back in matrix
    R[0] = x[0]; R[3] = x[1]; R[6] = x[2];
    R[1] = y[0]; R[4] = y[1]; R[7] = y[2];
    R[2] = z[0]; R[5] = z[1]; R[8] = z[2];
}

// Constants
const double k_f = 0.0004905f;
const double k_m = 0.00004905f;
const double L = 0.25f;
const double l = L / sqrtf(2);
const double I[3] = {0.01f, 0.02f, 0.01f};
const double g = 9.81f;
const double m = 0.5f;
const double dt = 0.01f;
const double omega_min = 30.0f;
const double omega_max = 70.0f;
const double omega_stable = 50.0f;

typedef struct {
    double omega[4];  // Motor speeds
    double angular_velocity_B[3];  // Angular velocity in body frame
    double linear_velocity_W[3];   // Linear velocity in world frame
    double linear_position_W[3];   // Position in world frame
    double R_W_B[9];              // Rotation matrix
    double I_mat[9];              // Inertia matrix
} Quad;

Quad* create_quad(double initial_height) {
    Quad* quad = (Quad*)malloc(sizeof(Quad));
    if (!quad) return NULL;

    // Initialize motor speeds to stable hover
    quad->omega[0] = omega_stable;
    quad->omega[1] = omega_stable;
    quad->omega[2] = omega_stable;
    quad->omega[3] = omega_stable;

    // Initialize velocities to zero
    memset(quad->angular_velocity_B, 0, 3 * sizeof(double));
    memset(quad->linear_velocity_W, 0, 3 * sizeof(double));

    // Set initial position
    quad->linear_position_W[0] = 0.0f;  // x
    quad->linear_position_W[1] = initial_height;  // y (height)
    quad->linear_position_W[2] = 0.0f;  // z

    // Initialize rotation matrix to identity (no initial rotation)
    double temp1[9], temp2[9];
    xRotMat3f(0, temp1);
    yRotMat3f(0, temp2);
    multMat3f(temp1, temp2, temp1);
    zRotMat3f(0, temp2);
    multMat3f(temp1, temp2, quad->R_W_B);

    // Initialize inertia matrix
    vecToDiagMat3f(I, quad->I_mat);

    return quad;
}

void update_dynamics(Quad* q) {
    // Limit motor speeds
    for(int i = 0; i < 4; i++) {
        q->omega[i] = fminf(fmaxf(q->omega[i], omega_min), omega_max);
    }

    // Forces and moments
    double F[4], M[4];
    for(int i = 0; i < 4; i++) {
        F[i] = k_f * q->omega[i] * fabsf(q->omega[i]);
        M[i] = k_m * q->omega[i] * fabsf(q->omega[i]);
    }

    // Thrust
    double f_B_thrust[3] = {0, F[0] + F[1] + F[2] + F[3], 0};

    // Torque
    double tau_B_drag[3] = {0, M[0] - M[1] + M[2] - M[3], 0};
    
    // Motor positions and forces
    double positions[4][3] = {
        {-L, 0, L},   // Motor 1
        {L, 0, L},    // Motor 2
        {L, 0, -L},   // Motor 3
        {-L, 0, -L}   // Motor 4
    };
    
    double forces[4][3] = {
        {0, F[0], 0}, // Force 1
        {0, F[1], 0}, // Force 2
        {0, F[2], 0}, // Force 3
        {0, F[3], 0}  // Force 4
    };

    // Calculate thrust torques
    double tau_B_thrust[3] = {0, 0, 0};
    double temp_thrust[3];
    for(int i = 0; i < 4; i++) {
        double tau_temp[3];
        crossVec3f(positions[i], forces[i], tau_temp);
        if(i == 0) {
            memcpy(tau_B_thrust, tau_temp, 3 * sizeof(double));
        } else {
            addVec3f(tau_B_thrust, tau_temp, temp_thrust);
            memcpy(tau_B_thrust, temp_thrust, 3 * sizeof(double));
        }
    }

    // Total torque
    double tau_B[3];
    addVec3f(tau_B_drag, tau_B_thrust, tau_B);

    // Accelerations
    double gravity_force[3] = {0, -g * m, 0};
    double rotated_thrust[3];
    multMatVec3f(q->R_W_B, f_B_thrust, rotated_thrust);
    
    double linear_acceleration_W[3];
    addVec3f(gravity_force, rotated_thrust, linear_acceleration_W);
    multScalVec3f(1.0f / m, linear_acceleration_W, linear_acceleration_W);

    // Angular acceleration
    double temp_vec[3], temp_vec2[3], cross_result[3];
    multMatVec3f(q->I_mat, q->angular_velocity_B, temp_vec);
    multScalVec3f(-1.0f, q->angular_velocity_B, temp_vec2);
    crossVec3f(temp_vec2, temp_vec, cross_result);
    
    double angular_acceleration_B[3];
    addVec3f(cross_result, tau_B, angular_acceleration_B);
    angular_acceleration_B[0] /= I[0];
    angular_acceleration_B[1] /= I[1];
    angular_acceleration_B[2] /= I[2];

    // State integration
    double temp_vel[3], temp_pos[3], temp_ang[3];
    
    // Update linear velocity and position
    multScalVec3f(dt, linear_acceleration_W, temp_vel);
    addVec3f(q->linear_velocity_W, temp_vel, q->linear_velocity_W);
    
    multScalVec3f(dt, q->linear_velocity_W, temp_pos);
    addVec3f(q->linear_position_W, temp_pos, q->linear_position_W);

    // Prevent drone from going below the ground
    if (q->linear_position_W[1] < -0.5f) {
        q->linear_position_W[1] = -0.5f;
        q->linear_velocity_W[1] = 0.0f;
    }

    // Update angular velocity
    multScalVec3f(dt, angular_acceleration_B, temp_ang);
    addVec3f(q->angular_velocity_B, temp_ang, q->angular_velocity_B);

    // Update rotation matrix
    double so3_result[9], temp_mat[9], temp_mat2[9];
    so3hat(q->angular_velocity_B, so3_result);
    multMat3f(q->R_W_B, so3_result, temp_mat);
    multScalMat3f(dt, temp_mat, temp_mat2);
    addMat3f(q->R_W_B, temp_mat2, q->R_W_B);

    // Orthonormalize rotation matrix
    orthonormalize_rotation_matrix(q->R_W_B);
}