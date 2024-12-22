#include "dynamics.h"
#include "rasterizer.h"

int main() {
    // Initialize meshes
    Mesh* drone_mesh = create_mesh("drone.obj", "drone.bmp");
    
    // Create array of ground meshes for 10x10 grid
    const int GROUND_COUNT = 100;  // 10x10 grid
    Mesh* ground_meshes[GROUND_COUNT];
    for(int i = 0; i < GROUND_COUNT; i++) {
        ground_meshes[i] = create_mesh("ground.obj", "ground.bmp");
    }

    // Create array of all meshes (1 drone + 100 ground tiles)
    Mesh* all_meshes[GROUND_COUNT + 1];
    all_meshes[0] = drone_mesh;
    for(int i = 0; i < GROUND_COUNT; i++) {
        all_meshes[i + 1] = ground_meshes[i];
    }

    // Initialize visualization buffers
    uint8_t *frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    uint8_t *flipped_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    ge_GIF *gif = ge_new_gif("drone_simulation.gif", WIDTH, HEIGHT, 4, -1, 0);

    // Initialize camera
    double camera_pos[3] = {-2.0, 2.0, -2.0};
    double camera_target[3] = {0.0, 0.0, 0.0};
    double camera_up[3] = {0.0, 1.0, 0.0};

    // Initialize drone state
    DroneState drone_state = {
        .omega = {omega_stable+1.0, omega_stable+1.0, omega_stable+1.0, omega_stable+1.0},
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
        
        // Transform drone mesh
        transform_mesh(drone_mesh, drone_pos, 0.5, rotation_y);

        // Transform ground meshes in a 10x10 grid
        for(int z = 0; z < 10; z++) {
            for(int x = 0; x < 10; x++) {
                int idx = z * 10 + x;
                double ground_pos[3] = {(x - 5) * 2.0, -0.5, (z - 5) * 2.0};
                transform_mesh(ground_meshes[idx], ground_pos, 1.0, 0.0);
            }
        }

        // Apply vertex shader to all meshes
        vertex_shader(drone_mesh, camera_pos, camera_target, camera_up);
        for(int i = 0; i < GROUND_COUNT; i++) {
            vertex_shader(ground_meshes[i], camera_pos, camera_target, camera_up);
        }

        // Rasterize all meshes at once
        rasterize(frame_buffer, all_meshes, GROUND_COUNT + 1);

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
    }

    // Cleanup
    ge_close_gif(gif);
    free(frame_buffer);
    free(flipped_buffer);
    
    // Free drone mesh
    free(drone_mesh->vertices);
    free(drone_mesh->initial_vertices);
    free(drone_mesh->texcoords);
    free(drone_mesh->triangles);
    free(drone_mesh->texcoord_indices);
    free(drone_mesh->texture_data);
    free(drone_mesh);
    
    // Free ground meshes
    for(int i = 0; i < GROUND_COUNT; i++) {
        free(ground_meshes[i]->vertices);
        free(ground_meshes[i]->initial_vertices);
        free(ground_meshes[i]->texcoords);
        free(ground_meshes[i]->triangles);
        free(ground_meshes[i]->texcoord_indices);
        free(ground_meshes[i]->texture_data);
        free(ground_meshes[i]);
    }

    return 0;
}