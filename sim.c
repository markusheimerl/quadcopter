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
    uint8_t *flipped_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    ge_GIF *gif = ge_new_gif("drone_simulation.gif", WIDTH, HEIGHT, 4, -1, 0);

    // Initialize camera
    double camera_pos[3] = {-2.0, 1.0, -2.0};
    double camera_target[3] = {0.0, 0.0, 0.0};
    double camera_up[3] = {0.0, 1.0, 0.0};

    // Initialize drone state
    DroneState drone_state = {
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
    multMat3f(temp1, temp2, drone_state.R_W_B);
    
    // Initialize inertia matrix
    vecToDiagMat3f(I, drone_state.I_mat);

    // Main simulation loop
    for(int frame = 0; frame < FRAMES; frame++) {
        // Clear frame buffer
        memset(frame_buffer, 0, WIDTH * HEIGHT * 3);

        // Update dynamics
        update_dynamics(&drone_state);
        
        // Update drone position and orientation for visualization
        double drone_pos[3] = {
            drone_state.linear_position_W[0],
            drone_state.linear_position_W[1],
            drone_state.linear_position_W[2]
        };
        
        // Extract rotation angle from R_W_B matrix
        double rotation_y = atan2(drone_state.R_W_B[2], drone_state.R_W_B[0]);
        
        // Transform meshes
        transform_mesh(meshes[0], drone_pos, 0.5, rotation_y);
        transform_mesh(meshes[1], (double[3]){0.0, -0.5, 0.0}, 1.0, 0.0);

        // Update camera to follow drone
        camera_target[0] = drone_state.linear_position_W[0];
        camera_target[1] = drone_state.linear_position_W[1];
        camera_target[2] = drone_state.linear_position_W[2];
        
        camera_pos[0] = drone_state.linear_position_W[0] - 2.0;
        camera_pos[1] = drone_state.linear_position_W[1] + 1.0;
        camera_pos[2] = drone_state.linear_position_W[2] - 2.0;

        // Render frame
        for (int i = 0; i < 2; i++) {
            vertex_shader(meshes[i], camera_pos, camera_target, camera_up);
        }
        rasterize(frame_buffer, meshes, 2);

        // Flip buffer vertically
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++) {
                int src_idx = (y * WIDTH + x) * 3;
                int dst_idx = ((HEIGHT - 1 - y) * WIDTH + x) * 3;
                flipped_buffer[dst_idx] = frame_buffer[src_idx];
                flipped_buffer[dst_idx + 1] = frame_buffer[src_idx + 1];
                flipped_buffer[dst_idx + 2] = frame_buffer[src_idx + 2];
            }
        }

        // Add frame to GIF
        ge_add_frame(gif, flipped_buffer, 6);
        
        // Print state
        printf("Frame %d/%d\n", frame + 1, FRAMES);
        printf("Position: [%.3f, %.3f, %.3f]\n", 
               drone_state.linear_position_W[0], 
               drone_state.linear_position_W[1], 
               drone_state.linear_position_W[2]);
        printf("Angular Velocity: [%.3f, %.3f, %.3f]\n",
               drone_state.angular_velocity_B[0],
               drone_state.angular_velocity_B[1],
               drone_state.angular_velocity_B[2]);
        printf("---\n");

        usleep(dt * 1000000);  // Sleep for dt seconds
    }

    // Cleanup
    ge_close_gif(gif);
    free(frame_buffer);
    free(flipped_buffer);
    
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