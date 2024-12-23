#ifndef DYNAMICS_H
#define DYNAMICS_H

#include <stdio.h>
#include <stdlib.h>
#include "vmath.h"

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
const double k_f = 0.0004905;
const double k_m = 0.00004905;
const double L = 0.25;
const double l = L / sqrt(2);
const double I[3] = {0.01, 0.02, 0.01};
const double g = 9.81;
const double m = 0.5;
const double dt = 0.01;
const double omega_min = 30.0;
const double omega_max = 70.0;
const double omega_stable = 50.0;

typedef struct {
    double omega[4];              // Motor speeds
    double angular_velocity_B[3];  // Angular velocity in body frame
    double linear_velocity_W[3];   // Linear velocity in world frame
    double linear_position_W[3];   // Position in world frame
    double R_W_B[9];              // Rotation matrix
    double I_mat[9];              // Inertia matrix
} Quad;

Quad* create_quad(double initial_height) {
    Quad* q = (Quad*)malloc(sizeof(Quad));
    
    // Initialize motor speeds
    for(int i = 0; i < 4; i++) {
        q->omega[i] = omega_stable;
    }
    
    // Initialize velocities and position
    for(int i = 0; i < 3; i++) {
        q->angular_velocity_B[i] = 0.0;
        q->linear_velocity_W[i] = 0.0;
        q->linear_position_W[i] = 0.0;
    }
    q->linear_position_W[1] = initial_height;  // Y is up
    
    // Initialize rotation matrix to identity
    identMat3f(q->R_W_B);
    
    // Initialize inertia matrix
    vecToDiagMat3f(I, q->I_mat);
    
    return q;
}

void update_dynamics(Quad* q) {
    // Limit motor speeds
    for(int i = 0; i < 4; i++) {
        if(q->omega[i] > omega_max) q->omega[i] = omega_max;
        if(q->omega[i] < omega_min) q->omega[i] = omega_min;
    }
    
    // Forces and moments
    double F[4], M[4];
    for(int i = 0; i < 4; i++) {
        F[i] = k_f * q->omega[i] * fabs(q->omega[i]);
        M[i] = k_m * q->omega[i] * fabs(q->omega[i]);
    }
    
    // Thrust
    double f_B_thrust[3] = {0.0, F[0] + F[1] + F[2] + F[3], 0.0};
    
    // Torques
    double tau_B_drag[3] = {0.0, M[0] - M[1] + M[2] - M[3], 0.0};
    
    // Compute thrust torques
    double pos1[3] = {-L, 0.0, L};
    double pos2[3] = {L, 0.0, L};
    double pos3[3] = {L, 0.0, -L};
    double pos4[3] = {-L, 0.0, -L};
    
    double force1[3] = {0.0, F[0], 0.0};
    double force2[3] = {0.0, F[1], 0.0};
    double force3[3] = {0.0, F[2], 0.0};
    double force4[3] = {0.0, F[3], 0.0};
    
    double tau_B_thrust[3], temp[3], temp2[3];
    
    crossVec3f(pos1, force1, temp);
    crossVec3f(pos2, force2, temp2);
    addVec3f(temp, temp2, tau_B_thrust);
    
    crossVec3f(pos3, force3, temp);
    addVec3f(tau_B_thrust, temp, temp2);
    
    crossVec3f(pos4, force4, temp);
    addVec3f(temp2, temp, tau_B_thrust);
    
    // Total torque
    double tau_B[3];
    addVec3f(tau_B_drag, tau_B_thrust, tau_B);
    
    // Accelerations
    double gravity[3] = {0.0, -g * m, 0.0};
    double temp3[3], linear_acceleration_W[3];
    multMatVec3f(q->R_W_B, f_B_thrust, temp);
    addVec3f(gravity, temp, temp2);
    multScalVec3f(1.0 / m, temp2, linear_acceleration_W);
    
    // Angular acceleration
    double temp4[3], temp5[3], angular_acceleration_B[3];
    multMatVec3f(q->I_mat, q->angular_velocity_B, temp);
    multScalVec3f(-1.0, q->angular_velocity_B, temp2);
    crossVec3f(temp2, temp, temp3);
    addVec3f(temp3, tau_B, angular_acceleration_B);
    
    for(int i = 0; i < 3; i++) {
        angular_acceleration_B[i] /= I[i];
    }
    
    // Advance state
    double temp6[3];
    multScalVec3f(dt, linear_acceleration_W, temp);
    addVec3f(q->linear_velocity_W, temp, q->linear_velocity_W);
    
    multScalVec3f(dt, q->linear_velocity_W, temp);
    addVec3f(q->linear_position_W, temp, q->linear_position_W);
    
    multScalVec3f(dt, angular_acceleration_B, temp);
    addVec3f(q->angular_velocity_B, temp, q->angular_velocity_B);
    
    // Update rotation matrix
    double skew[9], temp7[9], temp8[9];
    so3hat(q->angular_velocity_B, skew);
    multMat3f(q->R_W_B, skew, temp7);
    multScalMat3f(dt, temp7, temp8);
    addMat3f(q->R_W_B, temp8, q->R_W_B);

    // Orthonormalize rotation matrix
    orthonormalize_rotation_matrix(q->R_W_B);
}

#endif // DYNAMICS_H