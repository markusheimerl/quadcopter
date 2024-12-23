#include "gif.h"
#include "rasterizer.h"
#include "dynamics.h"

#define STEPS 600

// Global control parameters
const double k_p = 0.05;
const double k_v = 0.5;
const double k_R = 0.5;
const double k_w = 0.5;

// Desired state globals
const double linear_position_d_W[3] = {2.0, 2.0, 2.0};
const double linear_velocity_d_W[3] = {0.0, 0.0, 0.0};
const double linear_acceleration_d_W[3] = {0.0, 0.0, 0.0};
const double angular_velocity_d_B[3] = {0.0, 0.0, 0.0};
const double angular_acceleration_d_B[3] = {0.0, 0.0, 0.0};
const double yaw_d = 0.0;

void update_control(Quad* q) {
    // Temporary vectors for calculations
    double temp1[3], temp2[3], temp3[3], temp4[3];
    double z_W_d[3], z_W_B[3];
    
    // --- LINEAR CONTROL ---
    subVec3f(q->linear_position_W, linear_position_d_W, temp1);  // error_p
    subVec3f(q->linear_velocity_W, linear_velocity_d_W, temp2);  // error_v
    
    multScalVec3f(-k_p, temp1, z_W_d);
    multScalVec3f(-k_v, temp2, temp3);
    addVec3f(z_W_d, temp3, z_W_d);
    
    temp1[0] = 0.0; temp1[1] = m * g; temp1[2] = 0.0;
    addVec3f(z_W_d, temp1, z_W_d);
    
    multScalVec3f(m, linear_acceleration_d_W, temp1);
    addVec3f(z_W_d, temp1, z_W_d);
    
    double unit_y[3] = {0.0, 1.0, 0.0};
    multMatVec3f(q->R_W_B, unit_y, z_W_B);
    
    double f_z_B_control = dotVec3f(z_W_d, z_W_B);
    
    // --- ATTITUDE CONTROL ---
    double x_tilde_d_W[3] = {sin(yaw_d), 0.0, cos(yaw_d)};
    double R_W_d[9], R_W_d_column_0[3], R_W_d_column_1[3], R_W_d_column_2[3];
    
    crossVec3f(z_W_d, x_tilde_d_W, temp1);
    crossVec3f(temp1, z_W_d, temp2);
    normVec3f(temp2, R_W_d_column_0);
    
    crossVec3f(z_W_d, x_tilde_d_W, temp1);
    normVec3f(temp1, R_W_d_column_1);
    
    normVec3f(z_W_d, R_W_d_column_2);
    
    // Construct R_W_d
    for(int i = 0; i < 3; i++) {
        R_W_d[i] = R_W_d_column_1[i];
        R_W_d[i + 3] = R_W_d_column_2[i];
        R_W_d[i + 6] = R_W_d_column_0[i];
    }
    
    // Error calculations
    double R_W_d_T[9], R_W_B_T[9], temp_mat1[9], temp_mat2[9], error_mat[9];
    transpMat3f(R_W_d, R_W_d_T);
    transpMat3f(q->R_W_B, R_W_B_T);
    
    multMat3f(R_W_d_T, q->R_W_B, temp_mat1);
    multMat3f(R_W_B_T, R_W_d, temp_mat2);
    subMat3f(temp_mat1, temp_mat2, error_mat);
    
    double error_r[3], error_w[3], tau_B_control[3];
    so3vee(error_mat, error_r);
    multScalVec3f(0.5, error_r, error_r);
    
    multMat3f(R_W_d_T, q->R_W_B, temp_mat1);
    multMatVec3f(temp_mat1, angular_velocity_d_B, temp1);
    subVec3f(q->angular_velocity_B, temp1, error_w);
    
    // Control torque calculation
    multScalVec3f(-k_R, error_r, tau_B_control);
    multScalVec3f(-k_w, error_w, temp1);
    addVec3f(tau_B_control, temp1, tau_B_control);
    
    multMatVec3f(q->I_mat, q->angular_velocity_B, temp1);
    crossVec3f(q->angular_velocity_B, temp1, temp2);
    addVec3f(tau_B_control, temp2, tau_B_control);
    
    multMatVec3f(R_W_d, angular_acceleration_d_B, temp1);
    multMatVec3f(R_W_B_T, temp1, temp2);  // term_0
    
    multMatVec3f(R_W_d, angular_velocity_d_B, temp1);
    multMatVec3f(R_W_B_T, temp1, temp3);
    crossVec3f(q->angular_velocity_B, temp3, temp4);  // term_1
    
    subVec3f(temp4, temp2, temp1);
    multMatVec3f(q->I_mat, temp1, temp2);
    subVec3f(tau_B_control, temp2, tau_B_control);
    
    // Rotor speed calculation
    double F_bar[16], F_bar_inv[16];
    double pos_vectors[4][3] = {
        {-L, 0.0, L},
        {L, 0.0, L},
        {L, 0.0, -L},
        {-L, 0.0, -L}
    };
    
    // First row of F_bar
    for(int i = 0; i < 4; i++) {
        F_bar[i] = k_f;
    }
    
    // Remaining rows of F_bar
    double unit_y_vec[3] = {0.0, 1.0, 0.0};
    for(int i = 0; i < 4; i++) {
        double temp_vec[3], cross_result[3];
        multScalVec3f(k_f, pos_vectors[i], temp_vec);
        crossVec3f(temp_vec, unit_y_vec, cross_result);
        
        double moment_vec[3] = {0.0, (i % 2 == 0) ? k_m : -k_m, 0.0};
        addVec3f(moment_vec, cross_result, temp_vec);
        
        F_bar[4 + i] = temp_vec[0];
        F_bar[8 + i] = temp_vec[1];
        F_bar[12 + i] = temp_vec[2];
    }
    
    inv4Mat4f(F_bar, F_bar_inv);
    
    double control_vec[4] = {f_z_B_control, tau_B_control[0], tau_B_control[1], tau_B_control[2]};
    double omega_sign_square[4];
    multMatVec4f(F_bar_inv, control_vec, omega_sign_square);
    
    // Update motor speeds
    q->omega[0] = sqrt(fabs(omega_sign_square[0]));
    q->omega[1] = sqrt(fabs(omega_sign_square[1]));
    q->omega[2] = sqrt(fabs(omega_sign_square[2]));
    q->omega[3] = sqrt(fabs(omega_sign_square[3]));
}

int main() {
    // Initialize meshes
    Mesh* meshes[] = {
        create_mesh("rasterizer/drone.obj", "rasterizer/drone.bmp"),
        create_mesh("rasterizer/ground.obj", "rasterizer/ground.bmp")
    };

    // Initialize visualization buffers
    uint8_t *frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    ge_GIF *gif = ge_new_gif("drone_simulation.gif", WIDTH, HEIGHT, 4, -1, 0);

    // Initialize camera
    double camera_pos[3] = {-2.0, 2.0, -2.0};
    double camera_target[3] = {0.0, 0.0, 0.0};

    // Create and initialize quad
    Quad* quad = create_quad(1.0f);

    // Transform ground mesh
    transform_mesh(meshes[1], (double[3]){0.0, -0.5, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});

    // Main simulation loop
    for(int step = 0; step < STEPS; step++) {
        // Update dynamics and transform drone mesh
        update_dynamics(quad);
        transform_mesh(meshes[0], (double[3]){quad->linear_position_W[0], quad->linear_position_W[1], quad->linear_position_W[2]}, 0.5, quad->R_W_B);

        // Control
        update_control(quad);

        // Render frame and add to GIF
        memset(frame_buffer, 0, WIDTH * HEIGHT * 3);
        vertex_shader(meshes, 2, camera_pos, camera_target);
        rasterize(frame_buffer, meshes, 2);
        ge_add_frame(gif, frame_buffer, 6);
        
        // Print state
        printf("Step %d/%d\n", step + 1, STEPS);
        printf("Position: [%.3f, %.3f, %.3f]\n", quad->linear_position_W[0], quad->linear_position_W[1], quad->linear_position_W[2]);
        printf("Angular Velocity: [%.3f, %.3f, %.3f]\n", quad->angular_velocity_B[0], quad->angular_velocity_B[1], quad->angular_velocity_B[2]);
        printf("---\n");
    }

    // Cleanup
    free(quad);
    free(frame_buffer);
    free_meshes(meshes, 2);
    ge_close_gif(gif);
    
    return 0;
}