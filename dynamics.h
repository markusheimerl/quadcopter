#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

// Vector and matrix operations
void crossVec3f(const float v1[3], const float v2[3], float result[3]) {
    result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void multScalVec3f(float s, const float v[3], float result[3]) {
    result[0] = v[0] * s;
    result[1] = v[1] * s;
    result[2] = v[2] * s;
}

void addVec3f(const float v1[3], const float v2[3], float result[3]) {
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
}

void multMat3f(const float a[9], const float b[9], float result[9]) {
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

void multMatVec3f(const float m[9], const float v[3], float result[3]) {
    result[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    result[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    result[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

void vecToDiagMat3f(const float v[3], float result[9]) {
    result[0] = v[0]; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = v[1]; result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = v[2];
}

void xRotMat3f(float rads, float result[9]) {
    float s = sinf(rads);
    float c = cosf(rads);
    result[0] = 1.0f; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = c;    result[5] = -s;
    result[6] = 0.0f; result[7] = s;    result[8] = c;
}

void yRotMat3f(float rads, float result[9]) {
    float s = sinf(rads);
    float c = cosf(rads);
    result[0] = c;    result[1] = 0.0f; result[2] = s;
    result[3] = 0.0f; result[4] = 1.0f; result[5] = 0.0f;
    result[6] = -s;   result[7] = 0.0f; result[8] = c;
}

void zRotMat3f(float rads, float result[9]) {
    float s = sinf(rads);
    float c = cosf(rads);
    result[0] = c;    result[1] = -s;   result[2] = 0.0f;
    result[3] = s;    result[4] = c;    result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = 1.0f;
}

void so3hat(const float v[3], float result[9]) {
    result[0] = 0.0f;  result[1] = -v[2]; result[2] = v[1];
    result[3] = v[2];  result[4] = 0.0f;  result[5] = -v[0];
    result[6] = -v[1]; result[7] = v[0];  result[8] = 0.0f;
}

void addMat3f(const float a[9], const float b[9], float result[9]) {
    for(int i = 0; i < 9; i++) {
        result[i] = a[i] + b[i];
    }
}

void multScalMat3f(float s, const float m[9], float result[9]) {
    for(int i = 0; i < 9; i++) {
        result[i] = s * m[i];
    }
}

// Constants
const float k_f = 0.0004905f;
const float k_m = 0.00004905f;
const float L = 0.25f;
const float l = L / sqrtf(2);
const float I[3] = {0.01f, 0.02f, 0.01f};
const float g = 9.81f;
const float m = 0.5f;
const float dt = 0.01f;
const float omega_min = 30.0f;
const float omega_max = 70.0f;
const float omega_stable = 50.0f;

typedef struct {
    float omega[4];  // Motor speeds
    float angular_velocity_B[3];  // Angular velocity in body frame
    float linear_velocity_W[3];   // Linear velocity in world frame
    float linear_position_W[3];   // Position in world frame
    float R_W_B[9];              // Rotation matrix
    float I_mat[9];              // Inertia matrix
} Quad;

Quad* create_quad(float initial_height) {
    Quad* quad = (Quad*)malloc(sizeof(Quad));
    if (!quad) return NULL;

    // Initialize motor speeds to stable hover
    quad->omega[0] = omega_stable;
    quad->omega[1] = omega_stable;
    quad->omega[2] = omega_stable;
    quad->omega[3] = omega_stable;

    // Initialize velocities to zero
    memset(quad->angular_velocity_B, 0, 3 * sizeof(float));
    memset(quad->linear_velocity_W, 0, 3 * sizeof(float));

    // Set initial position
    quad->linear_position_W[0] = 0.0f;  // x
    quad->linear_position_W[1] = initial_height;  // y (height)
    quad->linear_position_W[2] = 0.0f;  // z

    // Initialize rotation matrix to identity (no initial rotation)
    float temp1[9], temp2[9];
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
    float F[4], M[4];
    for(int i = 0; i < 4; i++) {
        F[i] = k_f * q->omega[i] * fabsf(q->omega[i]);
        M[i] = k_m * q->omega[i] * fabsf(q->omega[i]);
    }

    // Thrust
    float f_B_thrust[3] = {0, F[0] + F[1] + F[2] + F[3], 0};

    // Torque
    float tau_B_drag[3] = {0, M[0] - M[1] + M[2] - M[3], 0};
    
    // Motor positions and forces
    float positions[4][3] = {
        {-L, 0, L},   // Motor 1
        {L, 0, L},    // Motor 2
        {L, 0, -L},   // Motor 3
        {-L, 0, -L}   // Motor 4
    };
    
    float forces[4][3] = {
        {0, F[0], 0}, // Force 1
        {0, F[1], 0}, // Force 2
        {0, F[2], 0}, // Force 3
        {0, F[3], 0}  // Force 4
    };

    // Calculate thrust torques
    float tau_B_thrust[3] = {0, 0, 0};
    float temp_thrust[3];
    for(int i = 0; i < 4; i++) {
        float tau_temp[3];
        crossVec3f(positions[i], forces[i], tau_temp);
        if(i == 0) {
            memcpy(tau_B_thrust, tau_temp, 3 * sizeof(float));
        } else {
            addVec3f(tau_B_thrust, tau_temp, temp_thrust);
            memcpy(tau_B_thrust, temp_thrust, 3 * sizeof(float));
        }
    }

    // Total torque
    float tau_B[3];
    addVec3f(tau_B_drag, tau_B_thrust, tau_B);

    // Accelerations
    float gravity_force[3] = {0, -g * m, 0};
    float rotated_thrust[3];
    multMatVec3f(q->R_W_B, f_B_thrust, rotated_thrust);
    
    float linear_acceleration_W[3];
    addVec3f(gravity_force, rotated_thrust, linear_acceleration_W);
    multScalVec3f(1.0f / m, linear_acceleration_W, linear_acceleration_W);

    // Angular acceleration
    float temp_vec[3], temp_vec2[3], cross_result[3];
    multMatVec3f(q->I_mat, q->angular_velocity_B, temp_vec);
    multScalVec3f(-1.0f, q->angular_velocity_B, temp_vec2);
    crossVec3f(temp_vec2, temp_vec, cross_result);
    
    float angular_acceleration_B[3];
    addVec3f(cross_result, tau_B, angular_acceleration_B);
    angular_acceleration_B[0] /= I[0];
    angular_acceleration_B[1] /= I[1];
    angular_acceleration_B[2] /= I[2];

    // State integration
    float temp_vel[3], temp_pos[3], temp_ang[3];
    
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
    float so3_result[9], temp_mat[9], temp_mat2[9];
    so3hat(q->angular_velocity_B, so3_result);
    multMat3f(q->R_W_B, so3_result, temp_mat);
    multScalMat3f(dt, temp_mat, temp_mat2);
    addMat3f(q->R_W_B, temp_mat2, q->R_W_B);
}