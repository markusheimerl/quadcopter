#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

// Vector operations
void crossVec3f(const float v1[3], const float v2[3], float result[3]) {
    result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    result[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

void addScalVec3f(float s, const float v[3], float result[3]) {
    result[0] = v[0] + s;
    result[1] = v[1] + s;
    result[2] = v[2] + s;
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

void subVec3f(const float v1[3], const float v2[3], float result[3]) {
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
}

float dotVec3f(const float v1[3], const float v2[3]) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

void normVec3f(const float v[3], float result[3]) {
    float magnitude = sqrt(dotVec3f(v, v));
    result[0] = v[0] / magnitude;
    result[1] = v[1] / magnitude;
    result[2] = v[2] / magnitude;
}

// Matrix operations
void inv4Mat4f(const float m[16], float result[16]) {
    float s0 = m[0] * m[5] - m[4] * m[1];
    float s1 = m[0] * m[6] - m[4] * m[2];
    float s2 = m[0] * m[7] - m[4] * m[3];
    float s3 = m[1] * m[6] - m[5] * m[2];
    float s4 = m[1] * m[7] - m[5] * m[3];
    float s5 = m[2] * m[7] - m[6] * m[3];

    float c5 = m[10] * m[15] - m[14] * m[11];
    float c4 = m[9] * m[15] - m[13] * m[11];
    float c3 = m[9] * m[14] - m[13] * m[10];
    float c2 = m[8] * m[15] - m[12] * m[11];
    float c1 = m[8] * m[14] - m[12] * m[10];
    float c0 = m[8] * m[13] - m[12] * m[9];

    // Should check for 0 determinant
    float invDet = 1.0f / (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);

    result[0] = (m[5] * c5 - m[6] * c4 + m[7] * c3) * invDet;
    result[1] = (-m[1] * c5 + m[2] * c4 - m[3] * c3) * invDet;
    result[2] = (m[13] * s5 - m[14] * s4 + m[15] * s3) * invDet;
    result[3] = (-m[9] * s5 + m[10] * s4 - m[11] * s3) * invDet;

    result[4] = (-m[4] * c5 + m[6] * c2 - m[7] * c1) * invDet;
    result[5] = (m[0] * c5 - m[2] * c2 + m[3] * c1) * invDet;
    result[6] = (-m[12] * s5 + m[14] * s2 - m[15] * s1) * invDet;
    result[7] = (m[8] * s5 - m[10] * s2 + m[11] * s1) * invDet;

    result[8] = (m[4] * c4 - m[5] * c2 + m[7] * c0) * invDet;
    result[9] = (-m[0] * c4 + m[1] * c2 - m[3] * c0) * invDet;
    result[10] = (m[12] * s4 - m[13] * s2 + m[15] * s0) * invDet;
    result[11] = (-m[8] * s4 + m[9] * s2 - m[11] * s0) * invDet;

    result[12] = 0.0f;
    result[13] = 0.0f;
    result[14] = 0.0f;
    result[15] = 1.0f;
}

void multMatVec4f(const float m[16], const float v[4], float result[4]) {
    result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2] + m[3]*v[3];
    result[1] = m[4]*v[0] + m[5]*v[1] + m[6]*v[2] + m[7]*v[3];
    result[2] = m[8]*v[0] + m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
    result[3] = m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15]*v[3];
}

void multMat3f(const float a[9], const float b[9], float result[9]) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            result[i*3 + j] = 0;
            for(int k = 0; k < 3; k++) {
                result[i*3 + j] += a[i*3 + k] * b[k*3 + j];
            }
        }
    }
}

void multMatVec3f(const float m[9], const float v[3], float result[3]) {
    result[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
    result[1] = m[3] * v[0] + m[4] * v[1] + m[5] * v[2];
    result[2] = m[6] * v[0] + m[7] * v[1] + m[8] * v[2];
}

void transpMat3f(const float m[9], float result[9]) {
    result[0] = m[0]; result[1] = m[3]; result[2] = m[6];
    result[3] = m[1]; result[4] = m[4]; result[5] = m[7];
    result[6] = m[2]; result[7] = m[5]; result[8] = m[8];
}

void xRotMat3f(float rads, float result[9]) {
    float s = sin(rads);
    float c = cos(rads);
    result[0] = 1.0f; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = c;    result[5] = -s;
    result[6] = 0.0f; result[7] = s;    result[8] = c;
}

void yRotMat3f(float rads, float result[9]) {
    float s = sin(rads);
    float c = cos(rads);
    result[0] = c;    result[1] = 0.0f; result[2] = s;
    result[3] = 0.0f; result[4] = 1.0f; result[5] = 0.0f;
    result[6] = -s;   result[7] = 0.0f; result[8] = c;
}

void zRotMat3f(float rads, float result[9]) {
    float s = sin(rads);
    float c = cos(rads);
    result[0] = c;    result[1] = -s;   result[2] = 0.0f;
    result[3] = s;    result[4] = c;    result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = 1.0f;
}

void so3hat(const float v[3], float result[9]) {
    result[0] = 0.0f;  result[1] = -v[2]; result[2] = v[1];
    result[3] = v[2];  result[4] = 0.0f;  result[5] = -v[0];
    result[6] = -v[1]; result[7] = v[0];  result[8] = 0.0f;
}

void so3vee(const float m[9], float result[3]) {
    result[0] = m[7];
    result[1] = m[2];
    result[2] = m[3];
}

void vecToDiagMat3f(const float v[3], float result[9]) {
    result[0] = v[0]; result[1] = 0.0f; result[2] = 0.0f;
    result[3] = 0.0f; result[4] = v[1]; result[5] = 0.0f;
    result[6] = 0.0f; result[7] = 0.0f; result[8] = v[2];
}

// Constants
#define K_F 0.0004905f
#define K_M 0.00004905f
#define L 0.25f
#define G 9.81f
#define MASS 0.5f
#define DT 0.01f
#define OMEGA_MIN 30.0f
#define OMEGA_MAX 70.0f
#define OMEGA_STABLE 50.0f

// Drone state
float omega[4] = {OMEGA_STABLE, OMEGA_STABLE, OMEGA_STABLE, OMEGA_STABLE};
float angular_velocity_B[3] = {0.0f, 0.0f, 0.0f};
float linear_velocity_W[3] = {0.0f, 0.0f, 0.0f};
float linear_position_W[3] = {0.0f, 1.0f, 0.0f};
float R_W_B[9];
float I[3] = {0.01f, 0.02f, 0.01f};
float loc_I_mat[9] = {0.01f, 0.0f, 0.0f, 0.0f, 0.02f, 0.0f, 0.0f, 0.0f, 0.01f};

// Control parameters
float linear_position_d_W[3] = {2.0f, 2.0f, 2.0f};
float linear_velocity_d_W[3] = {0.0f, 0.0f, 0.0f};
float linear_acceleration_d_W[3] = {0.0f, 0.0f, 0.0f};
float angular_velocity_d_B[3] = {0.0f, 0.0f, 0.0f};
float angular_acceleration_d_B[3] = {0.0f, 0.0f, 0.0f};
float yaw_d = 0.0f;

#define K_P 0.05f
#define K_V 0.5f
#define K_R 0.5f
#define K_W 0.5f

void update_dynamics() {
    // Limit motor speeds
    for(int i = 0; i < 4; i++) {
        if(omega[i] > OMEGA_MAX) omega[i] = OMEGA_MAX;
        if(omega[i] < OMEGA_MIN) omega[i] = OMEGA_MIN;
    }

    // Forces and moments
    float F[4], moments[4];
    for(int i = 0; i < 4; i++) {
        F[i] = K_F * omega[i] * fabs(omega[i]);
        moments[i] = K_M * omega[i] * fabs(omega[i]);
    }

    // Thrust
    float f_B_thrust[3] = {0.0f, F[0] + F[1] + F[2] + F[3], 0.0f};

    // Torque
    float tau_B_drag[3] = {0.0f, moments[0] - moments[1] + moments[2] - moments[3], 0.0f};
    
    float p1[3] = {-L, 0.0f, L};
    float p2[3] = {L, 0.0f, L};
    float p3[3] = {L, 0.0f, -L};
    float p4[3] = {-L, 0.0f, -L};
    
    float f1[3] = {0.0f, F[0], 0.0f};
    float f2[3] = {0.0f, F[1], 0.0f};
    float f3[3] = {0.0f, F[2], 0.0f};
    float f4[3] = {0.0f, F[3], 0.0f};

    float tau_B_thrust[3] = {0.0f, 0.0f, 0.0f};
    float temp[3];
    
    crossVec3f(p1, f1, temp);
    addVec3f(tau_B_thrust, temp, tau_B_thrust);
    crossVec3f(p2, f2, temp);
    addVec3f(tau_B_thrust, temp, tau_B_thrust);
    crossVec3f(p3, f3, temp);
    addVec3f(tau_B_thrust, temp, tau_B_thrust);
    crossVec3f(p4, f4, temp);
    addVec3f(tau_B_thrust, temp, tau_B_thrust);

    float tau_B[3];
    addVec3f(tau_B_drag, tau_B_thrust, tau_B);

    // Accelerations
    float gravity[3] = {0.0f, -G * MASS, 0.0f};
    float thrust_W[3];
    multMatVec3f(R_W_B, f_B_thrust, thrust_W);
    
    float linear_acceleration_W[3];
    addVec3f(gravity, thrust_W, linear_acceleration_W);
    multScalVec3f(1.0f/MASS, linear_acceleration_W, linear_acceleration_W);

    // Update state
    float delta_v[3], delta_p[3];
    multScalVec3f(DT, linear_acceleration_W, delta_v);
    addVec3f(linear_velocity_W, delta_v, linear_velocity_W);
    
    multScalVec3f(DT, linear_velocity_W, delta_p);
    addVec3f(linear_position_W, delta_p, linear_position_W);
}

void update_control() {
    // Position and velocity errors
    float error_p[3], error_v[3];
    subVec3f(linear_position_W, linear_position_d_W, error_p);
    subVec3f(linear_velocity_W, linear_velocity_d_W, error_v);

    // Desired acceleration computation
    float z_W_d[3];
    float temp[3];
    
    // -k_p * error_p
    multScalVec3f(-K_P, error_p, z_W_d);
    
    // -k_v * error_v
    multScalVec3f(-K_V, error_v, temp);
    addVec3f(z_W_d, temp, z_W_d);
    
    // Add gravity compensation
    float gravity[3] = {0.0f, MASS * G, 0.0f};
    addVec3f(z_W_d, gravity, z_W_d);
    
    // Add feedforward acceleration
    multScalVec3f(MASS, linear_acceleration_d_W, temp);
    addVec3f(z_W_d, temp, z_W_d);

    // Compute desired rotation matrix
    float x_tilde_d_W[3] = {sinf(yaw_d), 0.0f, cosf(yaw_d)};
    float R_W_d[9];
    float cross1[3], cross2[3];
    
    // Third column is normalized z_W_d
    float norm_z = sqrtf(dotVec3f(z_W_d, z_W_d));
    for(int i = 0; i < 3; i++) {
        R_W_d[i*3 + 2] = z_W_d[i] / norm_z;
    }
    
    // First column is cross(cross(z_W_d, x_tilde_d_W), z_W_d)
    crossVec3f(z_W_d, x_tilde_d_W, cross1);
    crossVec3f(cross1, z_W_d, cross2);
    float norm_x = sqrtf(dotVec3f(cross2, cross2));
    for(int i = 0; i < 3; i++) {
        R_W_d[i*3 + 0] = cross2[i] / norm_x;
    }
    
    // Second column is cross(z_W_d, x_tilde_d_W)
    float norm_y = sqrtf(dotVec3f(cross1, cross1));
    for(int i = 0; i < 3; i++) {
        R_W_d[i*3 + 1] = cross1[i] / norm_y;
    }

        // Rotation error
    float R_W_B_T[9], temp_mat[9], error_mat[9];
    float R_W_d_T[9];  // Move this declaration up
    transpMat3f(R_W_B, R_W_B_T);
    transpMat3f(R_W_d, R_W_d_T);  // Compute R_W_d_T before using it
    multMat3f(R_W_B_T, R_W_d, temp_mat);
    multMat3f(R_W_d_T, R_W_B, error_mat);
    
    float error_r[3];
    error_r[0] = 0.5f * (error_mat[7] - error_mat[5]);
    error_r[1] = 0.5f * (error_mat[2] - error_mat[6]);
    error_r[2] = 0.5f * (error_mat[3] - error_mat[1]);
    
    // Angular velocity error
    float temp_vec[3];
    multMat3f(R_W_d_T, R_W_B, temp_mat);
    multMatVec3f(temp_mat, angular_velocity_d_B, temp_vec);
    
    float error_w[3];
    subVec3f(angular_velocity_B, temp_vec, error_w);

    // Compute control torque
    float tau_B_control[3];
    multScalVec3f(-K_R, error_r, tau_B_control);
    
    multScalVec3f(-K_W, error_w, temp);
    addVec3f(tau_B_control, temp, tau_B_control);

    // Add angular velocity cross term
    float I_w[3];
    multMatVec3f(loc_I_mat, angular_velocity_B, I_w);
    crossVec3f(angular_velocity_B, I_w, temp);
    addVec3f(tau_B_control, temp, tau_B_control);

    // Add feedforward terms
    multMatVec3f(R_W_d_T, R_W_B, temp_mat);
    multMatVec3f(temp_mat, angular_acceleration_d_B, temp);
    
    crossVec3f(angular_velocity_B, temp_vec, cross1);
    subVec3f(cross1, temp, temp);
    multMatVec3f(loc_I_mat, temp, cross2);
    subVec3f(tau_B_control, cross2, tau_B_control);

    // Compute thrust
    float z_W_B[3];
    multMatVec3f(R_W_B, (float[]){0.0f, 1.0f, 0.0f}, z_W_B);
    float f_z_B_control = dotVec3f(z_W_d, z_W_B);

    // Compute individual motor speeds
    float F_bar[16] = {
        K_F, K_F, K_F, K_F,
        -L*K_F, L*K_F, L*K_F, -L*K_F,
        K_M, -K_M, K_M, -K_M,
        L*K_F, L*K_F, -L*K_F, -L*K_F
    };
    
    float F_bar_inv[16];
    inv4Mat4f(F_bar, F_bar_inv);
    
    float control_vec[4] = {f_z_B_control, tau_B_control[0], tau_B_control[1], tau_B_control[2]};
    float omega_squared[4];
    multMatVec4f(F_bar_inv, control_vec, omega_squared);

    // Set motor speeds with limits
    for(int i = 0; i < 4; i++) {
        omega[i] = sqrtf(fabs(omega_squared[i]));
        if(omega[i] > OMEGA_MAX) omega[i] = OMEGA_MAX;
        if(omega[i] < OMEGA_MIN) omega[i] = OMEGA_MIN;
    }
}

int main() {
    // Initialize rotation matrix
    float rx[9], ry[9], rz[9], temp[9];
    xRotMat3f(0.0f, rx);
    yRotMat3f(0.0f, ry);
    zRotMat3f(0.0f, rz);
    multMat3f(rx, ry, temp);
    multMat3f(temp, rz, R_W_B);

    // Main simulation loop
    float t = 0.0f;
    while(t < 50.0f) {  // Run for 10 seconds
        update_dynamics();
        update_control();
        
        // Print state every 100 timesteps
        if(fmod(t, 0.1f) < DT) {
            printf("t=%.2f pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f)\n",
                   t,
                   linear_position_W[0], linear_position_W[1], linear_position_W[2],
                   linear_velocity_W[0], linear_velocity_W[1], linear_velocity_W[2]);
        }

        // Check if target reached
        float pos_error[3];
        subVec3f(linear_position_W, linear_position_d_W, pos_error);
        if(fabs(pos_error[0]) < 0.1f && 
           fabs(pos_error[1]) < 0.1f && 
           fabs(pos_error[2]) < 0.1f) {
            printf("Target reached!\n");
            break;
        }

        t += DT;
    }

    return 0;
}