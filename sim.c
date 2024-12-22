#include "gif.h"
#include "dynamics.h"
#include "rasterizer.h"

int main() {
    // Initialize meshes
    Mesh* meshes[] = {
        create_mesh("drone.obj", "drone.bmp"),
        create_mesh("ground.obj", "ground.bmp")
    };

    // Initialize visualization buffers
    uint8_t *frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    ge_GIF *gif = ge_new_gif("drone_simulation.gif", WIDTH, HEIGHT, 4, -1, 0);

    // Initialize camera
    double camera_pos[3] = {-2.0, 2.0, -2.0};
    double camera_target[3] = {0.0, 0.0, 0.0};
    double camera_up[3] = {0.0, 1.0, 0.0};

    // Initialize drone state
    Quad quad = {
        .omega = {omega_stable, omega_stable, omega_stable, omega_stable},
        .angular_velocity_B = {0, 0, 0},
        .linear_velocity_W = {0, 0, 0},
        .linear_position_W = {0, 1, 0}
    };
    
    // Initialize rotation matrix
    float temp1[9], temp2[9];
    xRotMat3f(0, temp1);
    yRotMat3f(0, temp2);
    multMat3f(temp1, temp2, temp1);
    zRotMat3f(0, temp2);
    multMat3f(temp1, temp2, quad.R_W_B);
    
    // Initialize inertia matrix
    vecToDiagMat3f(I, quad.I_mat);

    // Main simulation loop
    for(int frame = 0; frame < FRAMES; frame++) {
        // Clear frame buffer
        memset(frame_buffer, 0, WIDTH * HEIGHT * 3);

        // Update dynamics
        update_dynamics(&quad);
        
        // Update drone position and orientation for visualization
        double drone_pos[3] = {
            quad.linear_position_W[0],
            quad.linear_position_W[1],
            quad.linear_position_W[2]
        };
        
        // Extract rotation angle from R_W_B matrix
        double rotation_y = atan2(quad.R_W_B[2], quad.R_W_B[0]);
        
        // Transform meshes
        transform_mesh(meshes[0], drone_pos, 0.5, rotation_y);
        transform_mesh(meshes[1], (double[3]){0.0, -0.5, 0.0}, 1.0, 0.0);

        // Render frame
        vertex_shader(meshes, 2, camera_pos, camera_target, camera_up);
        rasterize(frame_buffer, meshes, 2);

        // Add frame to GIF
        ge_add_frame(gif, frame_buffer, 6);
        
        // Print state
        printf("Frame %d/%d\n", frame + 1, FRAMES);
        printf("Position: [%.3f, %.3f, %.3f]\n", 
               quad.linear_position_W[0], 
               quad.linear_position_W[1], 
               quad.linear_position_W[2]);
        printf("Angular Velocity: [%.3f, %.3f, %.3f]\n",
               quad.angular_velocity_B[0],
               quad.angular_velocity_B[1],
               quad.angular_velocity_B[2]);
        printf("---\n");
    }

    // Cleanup
    ge_close_gif(gif);
    free(frame_buffer);
    
    for (int i = 0; i < 2; i++) {
        if (meshes[i]) {
            free(meshes[i]->vertices);
            free(meshes[i]->initial_vertices);
            free(meshes[i]->texcoords);
            free(meshes[i]->triangles);
            free(meshes[i]->texcoord_indices);
            free(meshes[i]->texture_data);
            free(meshes[i]);
        }
    }

    return 0;
}