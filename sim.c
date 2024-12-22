#include "gif.h"
#include "rasterizer.h"
#include "dynamics.h"

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

    // Create and initialize quad
    Quad* quad = create_quad(1.0f);

    // Transform ground mesh
    transform_mesh(meshes[1], (double[3]){0.0, -0.5, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});

    // Main simulation loop
    for(int frame = 0; frame < FRAMES; frame++) {
        // Clear frame buffer
        memset(frame_buffer, 0, WIDTH * HEIGHT * 3);

        // Update dynamics and transform drone mesh
        update_dynamics(quad);
        transform_mesh(meshes[0], (double[3]){quad->linear_position_W[0], quad->linear_position_W[1], quad->linear_position_W[2]}, 0.5, quad->R_W_B);

        // Render frame
        vertex_shader(meshes, 2, camera_pos, camera_target);
        rasterize(frame_buffer, meshes, 2);

        // Add frame to GIF
        ge_add_frame(gif, frame_buffer, 6);
        
        // Print state
        printf("Frame %d/%d\n", frame + 1, FRAMES);
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