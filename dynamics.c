#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

// gcc -O3 dynamics.c -lm && ./a.out

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

void invMat3f(const float m[9], float result[9]) {
    float det = m[0] * (m[4] * m[8] - m[7] * m[5]) -
                m[1] * (m[3] * m[8] - m[5] * m[6]) +
                m[2] * (m[3] * m[7] - m[4] * m[6]);

    if (det == 0) {
        fprintf(stderr, "Matrix is not invertible\n");
        exit(1);
    }

    float invDet = 1.0f / det;

    result[0] = invDet * (m[4] * m[8] - m[7] * m[5]);
    result[1] = invDet * (m[2] * m[7] - m[1] * m[8]);
    result[2] = invDet * (m[1] * m[5] - m[2] * m[4]);
    result[3] = invDet * (m[5] * m[6] - m[3] * m[8]);
    result[4] = invDet * (m[0] * m[8] - m[2] * m[6]);
    result[5] = invDet * (m[3] * m[2] - m[0] * m[5]);
    result[6] = invDet * (m[3] * m[7] - m[6] * m[4]);
    result[7] = invDet * (m[6] * m[1] - m[0] * m[7]);
    result[8] = invDet * (m[0] * m[4] - m[3] * m[1]);
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

int main() {
    // State variables
    float omega_1 = omega_stable;
    float omega_2 = omega_stable;
    float omega_3 = omega_stable;
    float omega_4 = omega_stable;

    float angular_velocity_B[3] = {0, 0, 0};
    float linear_velocity_W[3] = {0, 0, 0};
    float linear_position_W[3] = {0, 1, 0};

    float R_W_B[9];
    float temp1[9], temp2[9];
    xRotMat3f(0, temp1);
    yRotMat3f(0, temp2);
    multMat3f(temp1, temp2, temp1);
    zRotMat3f(0, temp2);
    multMat3f(temp1, temp2, R_W_B);

    float loc_I_mat[9];
    vecToDiagMat3f(I, loc_I_mat);
    float loc_I_mat_inv[9];
    invMat3f(loc_I_mat, loc_I_mat_inv);

    // Simulation loop
    for(int iteration = 0; iteration < 1000; iteration++) {
        // Limit motor speeds
        omega_1 = fminf(fmaxf(omega_1, omega_min), omega_max);
        omega_2 = fminf(fmaxf(omega_2, omega_min), omega_max);
        omega_3 = fminf(fmaxf(omega_3, omega_min), omega_max);
        omega_4 = fminf(fmaxf(omega_4, omega_min), omega_max);

        // Forces and moments
        float F1 = k_f * omega_1 * fabsf(omega_1);
        float F2 = k_f * omega_2 * fabsf(omega_2);
        float F3 = k_f * omega_3 * fabsf(omega_3);
        float F4 = k_f * omega_4 * fabsf(omega_4);

        float M1 = k_m * omega_1 * fabsf(omega_1);
        float M2 = k_m * omega_2 * fabsf(omega_2);
        float M3 = k_m * omega_3 * fabsf(omega_3);
        float M4 = k_m * omega_4 * fabsf(omega_4);

        // Thrust
        float f_B_thrust[3] = {0, F1 + F2 + F3 + F4, 0};

        // Torque
        float tau_B_drag[3] = {0, M1 - M2 + M3 - M4, 0};
        
        float pos1[3] = {-L, 0, L};
        float force1[3] = {0, F1, 0};
        float tau_B_thrust_1[3];
        crossVec3f(pos1, force1, tau_B_thrust_1);

        float pos2[3] = {L, 0, L};
        float force2[3] = {0, F2, 0};
        float tau_B_thrust_2[3];
        crossVec3f(pos2, force2, tau_B_thrust_2);

        float pos3[3] = {L, 0, -L};
        float force3[3] = {0, F3, 0};
        float tau_B_thrust_3[3];
        crossVec3f(pos3, force3, tau_B_thrust_3);

        float pos4[3] = {-L, 0, -L};
        float force4[3] = {0, F4, 0};
        float tau_B_thrust_4[3];
        crossVec3f(pos4, force4, tau_B_thrust_4);

        float tau_B_thrust[3];
        float temp_thrust[3];
        addVec3f(tau_B_thrust_1, tau_B_thrust_2, tau_B_thrust);
        addVec3f(tau_B_thrust, tau_B_thrust_3, temp_thrust);
        addVec3f(temp_thrust, tau_B_thrust_4, tau_B_thrust);

        float tau_B[3];
        addVec3f(tau_B_drag, tau_B_thrust, tau_B);

        // Accelerations
        float gravity_force[3] = {0, -g * m, 0};
        float rotated_thrust[3];
        multMatVec3f(R_W_B, f_B_thrust, rotated_thrust);
        
        float linear_acceleration_W[3];
        addVec3f(gravity_force, rotated_thrust, linear_acceleration_W);
        multScalVec3f(1.0f / m, linear_acceleration_W, linear_acceleration_W);

        float temp_vec[3];
        float temp_vec2[3];
        multMatVec3f(loc_I_mat, angular_velocity_B, temp_vec);
        multScalVec3f(-1.0f, angular_velocity_B, temp_vec2);
        float cross_result[3];
        crossVec3f(temp_vec2, temp_vec, cross_result);
        
        float angular_acceleration_B[3];
        addVec3f(cross_result, tau_B, angular_acceleration_B);
        angular_acceleration_B[0] /= I[0];
        angular_acceleration_B[1] /= I[1];
        angular_acceleration_B[2] /= I[2];

        // Advance state
        float temp_vel[3];
        multScalVec3f(dt, linear_acceleration_W, temp_vel);
        addVec3f(linear_velocity_W, temp_vel, linear_velocity_W);

        float temp_pos[3];
        multScalVec3f(dt, linear_velocity_W, temp_pos);
        addVec3f(linear_position_W, temp_pos, linear_position_W);

        float temp_ang[3];
        multScalVec3f(dt, angular_acceleration_B, temp_ang);
        addVec3f(angular_velocity_B, temp_ang, angular_velocity_B);

        // Update rotation matrix
        float so3_result[9];
        float temp_mat[9];
        float temp_mat2[9];
        so3hat(angular_velocity_B, so3_result);
        multMat3f(R_W_B, so3_result, temp_mat);
        multScalMat3f(dt, temp_mat, temp_mat2);
        addMat3f(R_W_B, temp_mat2, R_W_B);

        // Print state
        printf("Iteration %d:\n", iteration);
        printf("Position: [%.3f, %.3f, %.3f]\n", 
               linear_position_W[0], linear_position_W[1], linear_position_W[2]);
        printf("Velocity: [%.3f, %.3f, %.3f]\n", 
               linear_velocity_W[0], linear_velocity_W[1], linear_velocity_W[2]);
        printf("Angular Velocity: [%.3f, %.3f, %.3f]\n", 
               angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2]);
        printf("---\n");

        usleep(dt * 1000000);  // Sleep for dt seconds
    }

    return 0;
}