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

void invMat3f(const double* m, double* result) {
    double det = m[0]*(m[4]*m[8] - m[7]*m[5]) - 
                m[1]*(m[3]*m[8] - m[5]*m[6]) + 
                m[2]*(m[3]*m[7] - m[4]*m[6]);
    
    if (det == 0.0f) {
        // Handle error case
        return;
    }
    
    double invDet = 1.0f/det;
    result[0] = invDet*(m[4]*m[8] - m[7]*m[5]);
    result[1] = invDet*(m[2]*m[7] - m[1]*m[8]);
    result[2] = invDet*(m[1]*m[5] - m[2]*m[4]);
    result[3] = invDet*(m[5]*m[6] - m[3]*m[8]);
    result[4] = invDet*(m[0]*m[8] - m[2]*m[6]);
    result[5] = invDet*(m[3]*m[2] - m[0]*m[5]);
    result[6] = invDet*(m[3]*m[7] - m[6]*m[4]);
    result[7] = invDet*(m[6]*m[1] - m[0]*m[7]);
    result[8] = invDet*(m[0]*m[4] - m[3]*m[1]);
}

void transpMat3f(const double* m, double* result) {
    result[0] = m[0]; result[1] = m[3]; result[2] = m[6];
    result[3] = m[1]; result[4] = m[4]; result[5] = m[7];
    result[6] = m[2]; result[7] = m[5]; result[8] = m[8];
}

void identMat3f(double* result) {
    for(int i = 0; i < 9; i++) result[i] = 0;
    result[0] = result[4] = result[8] = 1.0;
}

void rotMat3f(char axis, double rads, double* result) {
    double s = sinf(rads), c = cosf(rads);
    switch(axis) {
        case 'x':
            result[0]=1; result[1]=0; result[2]=0;
            result[3]=0; result[4]=c; result[5]=-s;
            result[6]=0; result[7]=s; result[8]=c;
            break;
        case 'y':
            result[0]=c; result[1]=0; result[2]=s;
            result[3]=0; result[4]=1; result[5]=0;
            result[6]=-s; result[7]=0; result[8]=c;
            break;
        case 'z':
            result[0]=c; result[1]=-s; result[2]=0;
            result[3]=s; result[4]=c; result[5]=0;
            result[6]=0; result[7]=0; result[8]=1;
            break;
    }
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

// 4x4 Matrix Operations
void multMat4f(const double* a, const double* b, double* result) {
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            result[i*4 + j] = a[i*4]*b[j] + a[i*4+1]*b[j+4] + 
                             a[i*4+2]*b[j+8] + a[i*4+3]*b[j+12];
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

// Helper functions for rotation matrices
void xRotMat3f(double rads, double* result) {
    rotMat3f('x', rads, result);
}

void yRotMat3f(double rads, double* result) {
    rotMat3f('y', rads, result);
}

void zRotMat3f(double rads, double* result) {
    rotMat3f('z', rads, result);
}

// Constants
#define K_F 0.0004905
#define K_M 0.00004905
#define L 0.25
#define L_SQRT2 (L / sqrtf(2.0))
#define G 9.81
#define M 0.5
#define DT 0.01
#define OMEGA_MIN 30.0
#define OMEGA_MAX 70.0
#define OMEGA_STABLE 50.0

// State variables
double omega[4];
double angular_velocity_B[3];
double linear_velocity_W[3];
double linear_position_W[3];
double R_W_B[3][3];  // 3x3 rotation matrix
double I[3] = {0.01, 0.02, 0.01};

// Control variables
double linear_position_d_W[3] = {2.0, 2.0, 2.0};
double linear_velocity_d_W[3] = {0.0, 0.0, 0.0};
double linear_acceleration_d_W[3] = {0.0, 0.0, 0.0};
double angular_velocity_d_B[3] = {0.0, 0.0, 0.0};
double angular_acceleration_d_B[3] = {0.0, 0.0, 0.0};
double yaw_d = 0.0;

const double k_p = 0.05;
const double k_v = 0.5;
const double k_R = 0.5;
const double k_w = 0.5;

void init_drone_state(void) {
    // Initialize omegas
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
    
    // Initialize position (0, 1, 0)
    linear_position_W[0] = 0.0;
    linear_position_W[1] = 1.0;
    linear_position_W[2] = 0.0;
    
    // Initialize rotation matrix (identity matrix from rotation of 0 around all axes)
    double temp[3][3];
    double result[3][3];
    
    // Get rotation matrices for 0 rotation around each axis and multiply them
    xRotMat3f(0.0, temp);
    yRotMat3f(0.0, result);
    multMat3f(temp, result, R_W_B);
    zRotMat3f(0.0, temp);
    multMat3f(R_W_B, temp, R_W_B);
}

void update_drone_physics(void) {
    // Limit motor speeds
    for(int i = 0; i < 4; i++) {
        omega[i] = fmax(fmin(omega[i], OMEGA_MAX), OMEGA_MIN);
    }

    // Calculate individual rotor forces and moments
    double f[4], m[4];
    for(int i = 0; i < 4; i++) {
        f[i] = K_F * omega[i] * fabs(omega[i]);
        m[i] = K_M * omega[i] * fabs(omega[i]);
    }

    // Total thrust force in body frame
    double f_B_thrust[3] = {0, f[0] + f[1] + f[2] + f[3], 0};

    // Torques from drag and thrust
    double tau_B_drag[3] = {0, m[0] - m[1] + m[2] - m[3], 0};

    // Torques from thrust forces
    double rotor_pos[4][3] = {
        {-L, 0, L},
        {L, 0, L},
        {L, 0, -L},
        {-L, 0, -L}
    };
    double f_vectors[4][3] = {
        {0, f[0], 0},
        {0, f[1], 0},
        {0, f[2], 0},
        {0, f[3], 0}
    };
    
    double tau_individual[4][3];
    for(int i = 0; i < 4; i++) {
        crossVec3f(rotor_pos[i], f_vectors[i], tau_individual[i]);
    }

    double tau_B_thrust[3] = {0, 0, 0};
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 3; j++) {
            tau_B_thrust[j] += tau_individual[i][j];
        }
    }

    double tau_B[3];
    for(int i = 0; i < 3; i++) {
        tau_B[i] = tau_B_drag[i] + tau_B_thrust[i];
    }

    // Transform thrust to world frame and add gravity
    double f_thrust_W[3];
    multMatVec3f((double*)R_W_B, f_B_thrust, f_thrust_W);
    double f_gravity_W[3] = {0, -G * M, 0};

    // Calculate accelerations
    double linear_acceleration_W[3];
    for(int i = 0; i < 3; i++) {
        linear_acceleration_W[i] = (f_thrust_W[i] + f_gravity_W[i]) / M;
    }

    // Angular momentum terms
    double I_mat[9];
    vecToDiagMat3f(I, I_mat);
    double h_B[3];
    multMatVec3f(I_mat, angular_velocity_B, h_B);
    
    double neg_angular_velocity[3];
    multScalVec3f(-1, angular_velocity_B, neg_angular_velocity);
    
    double w_cross_h[3];
    crossVec3f(neg_angular_velocity, h_B, w_cross_h);

    double angular_acceleration_B[3];
    for(int i = 0; i < 3; i++) {
        angular_acceleration_B[i] = (w_cross_h[i] + tau_B[i]) / I[i];
    }

    // Update state variables
    for(int i = 0; i < 3; i++) {
        linear_velocity_W[i] += DT * linear_acceleration_W[i];
        linear_position_W[i] += DT * linear_velocity_W[i];
        angular_velocity_B[i] += DT * angular_acceleration_B[i];
    }

    // Update rotation matrix
    double w_hat[9];
    so3hat(angular_velocity_B, w_hat);
    double R_dot[9];
    multMat3f((double*)R_W_B, w_hat, R_dot);
    
    double R_dot_scaled[9];
    multScalMat3f(DT, R_dot, R_dot_scaled);
    
    double R_new[9];
    addMat3f((double*)R_W_B, R_dot_scaled, R_new);
    
    // Copy back to R_W_B
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            R_W_B[i][j] = R_new[i*3 + j];
        }
    }
}

void update_drone_control(void) {
    // --- LINEAR CONTROL ---
    double error_p[3], error_v[3];
    subVec3f(linear_position_W, linear_position_d_W, error_p);
    subVec3f(linear_velocity_W, linear_velocity_d_W, error_v);

    double z_W_d[3], temp[3];
    multScalVec3f(-k_p, error_p, z_W_d);
    multScalVec3f(-k_v, error_v, temp);
    addVec3f(z_W_d, temp, z_W_d);
    
    double gravity_term[3] = {0, M * G, 0};
    addVec3f(z_W_d, gravity_term, z_W_d);
    
    double accel_term[3];
    multScalVec3f(M, linear_acceleration_d_W, accel_term);
    addVec3f(z_W_d, accel_term, z_W_d);
    
    double z_W_B[3];
    double y_body[3] = {0, 1, 0};
    multMatVec3f((double*)R_W_B, y_body, z_W_B);
    
    double f_z_B_control = dotVec3f(z_W_d, z_W_B);

    // --- ATTITUDE CONTROL ---
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

    double R_W_d_T[9], R_W_B_T[9];
    transpMat3f(R_W_d, R_W_d_T);
    transpMat3f((double*)R_W_B, R_W_B_T);

    double temp_mat1[9], temp_mat2[9], temp_mat3[9];
    multMat3f(R_W_d_T, (double*)R_W_B, temp_mat1);
    multMat3f(R_W_B_T, R_W_d, temp_mat2);
    subMat3f(temp_mat1, temp_mat2, temp_mat3);

    double error_r[3];
    so3vee(temp_mat3, error_r);
    multScalVec3f(0.5, error_r, error_r);

    double temp_vec[3], error_w[3];
    multMat3f(R_W_d_T, (double*)R_W_B, temp_mat1);
    multMatVec3f(temp_mat1, angular_velocity_d_B, temp_vec);
    subVec3f(angular_velocity_B, temp_vec, error_w);

    double tau_B_control[3];
    multScalVec3f(-k_R, error_r, tau_B_control);
    
    double temp_vec2[3];
    multScalVec3f(-k_w, error_w, temp_vec2);
    addVec3f(tau_B_control, temp_vec2, tau_B_control);

    double I_mat[9], temp_vec3[3], temp_vec4[3];
    vecToDiagMat3f(I, I_mat);
    multMatVec3f(I_mat, angular_velocity_B, temp_vec3);
    crossVec3f(angular_velocity_B, temp_vec3, temp_vec4);
    addVec3f(tau_B_control, temp_vec4, tau_B_control);

    double term_0[3], term_1[3];
    multMatVec3f(R_W_d, angular_acceleration_d_B, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, term_0);

    multMatVec3f(R_W_d, angular_velocity_d_B, temp_vec);
    multMatVec3f(R_W_B_T, temp_vec, temp_vec2);
    crossVec3f(angular_velocity_B, temp_vec2, term_1);

    double temp_vec5[3];
    subVec3f(term_1, term_0, temp_vec5);
    multMatVec3f(I_mat, temp_vec5, temp_vec);
    subVec3f(tau_B_control, temp_vec, tau_B_control);

    // --- ROTOR SPEEDS ---
    double F_bar[16];
    // First row
    F_bar[0] = K_F;
    F_bar[1] = K_F;
    F_bar[2] = K_F;
    F_bar[3] = K_F;

    // Calculate columns
    double rotor_positions[4][3] = {
        {-L, 0, L},
        {L, 0, L},
        {L, 0, -L},
        {-L, 0, -L}
    };

    for(int i = 0; i < 4; i++) {
        double force[3] = {0, 1, 0};
        double moment[3];
        multScalVec3f(K_F, rotor_positions[i], temp_vec);
        crossVec3f(temp_vec, force, moment);
        
        double column[3];
        if(i % 2 == 0) {
            column[0] = moment[0];
            column[1] = K_M;
            column[2] = moment[2];
        } else {
            column[0] = moment[0];
            column[1] = -K_M;
            column[2] = moment[2];
        }
        
        F_bar[4 + i] = column[0];
        F_bar[8 + i] = column[1];
        F_bar[12 + i] = column[2];
    }

    double F_bar_inv[16];
    inv4Mat4f(F_bar, F_bar_inv);

    double control_input[4] = {f_z_B_control, tau_B_control[0], tau_B_control[1], tau_B_control[2]};
    double omega_sign_square[4];
    multMatVec4f(F_bar_inv, control_input, omega_sign_square);

    omega[0] = sqrt(fabs(omega_sign_square[0]));
    omega[1] = sqrt(fabs(omega_sign_square[1]));
    omega[2] = sqrt(fabs(omega_sign_square[2]));
    omega[3] = sqrt(fabs(omega_sign_square[3]));
}

#include <stdio.h>
#include <time.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

// Function to check if drone is at target position
bool is_at_target(void) {
    for(int i = 0; i < 3; i++) {
        if(fabs(linear_position_W[i] - linear_position_d_W[i]) >= 0.1) {
            return false;
        }
    }
    return true;
}

// Function to print drone state
void print_drone_state(void) {
    time_t now;
    char time_str[26];
    time(&now);
    ctime_r(&now, time_str);
    time_str[24] = '\0';  // Remove newline

    printf("\nTime: %s\n", time_str);
    printf("Linear Position: [%.3f, %.3f, %.3f]\n", 
           linear_position_W[0], linear_position_W[1], linear_position_W[2]);
    printf("Linear Velocity: [%.3f, %.3f, %.3f]\n", 
           linear_velocity_W[0], linear_velocity_W[1], linear_velocity_W[2]);
    printf("Angular Velocity: [%.3f, %.3f, %.3f]\n", 
           angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2]);
    printf("Rotor Speeds: %.3f, %.3f, %.3f, %.3f\n", 
           omega[0], omega[1], omega[2], omega[3]);
}

int main(void) {
    // Initialize drone state
    init_drone_state();

    // Get start time
    time_t start_time = time(NULL);
    time_t last_print = start_time;
    
    // Main control loop
    while(1) {
        // Get current time
        time_t current_time = time(NULL);
        
        // Update drone physics and control
        update_drone_physics();
        update_drone_control();
        
        // Print state every 100ms
        if(current_time - last_print >= 0.1) {  // 100ms
            print_drone_state();
            last_print = current_time;
        }
        
        // Check if at target position
        if(is_at_target()) {
            printf("\nTarget position reached! Exiting...\n");
            break;
        }
        
        // Check timeout (10 seconds)
        if(current_time - start_time >= 10) {
            printf("\nTimeout reached! Exiting...\n");
            break;
        }
        
        // Sleep for dt seconds
        usleep((useconds_t)(DT * 1000000));  // Convert to microseconds
    }
    
    return 0;
}