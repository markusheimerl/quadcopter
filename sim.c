#include "gif.h"
#include "rasterizer.h"
#include "quad.h"

#define STEPS 180

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

    // Initialize drone state
    init_drone_state();

    // Transform ground mesh
    transform_mesh(meshes[1], (double[3]){0.0, -0.5, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});

    // Main simulation loop
    for(int step = 0; step < STEPS; step++) {
        // Update dynamics and transform drone mesh
        update_drone_physics();
        transform_mesh(meshes[0], (double[3]){linear_position_W[0], linear_position_W[1], linear_position_W[2]}, 0.5, R_W_B);

        // Control
        update_drone_control();

        // Render frame and add to GIF
        memset(frame_buffer, 0, WIDTH * HEIGHT * 3);
        vertex_shader(meshes, 2, camera_pos, camera_target);
        rasterize(frame_buffer, meshes, 2);
        ge_add_frame(gif, frame_buffer, 6);
        
        // Print state
        printf("Step %d/%d\n", step + 1, STEPS);
        printf("Position: [%.3f, %.3f, %.3f]\n", linear_position_W[0], linear_position_W[1], linear_position_W[2]);
        printf("Angular Velocity: [%.3f, %.3f, %.3f]\n", angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2]);
        printf("---\n");
    }

    // Cleanup
    free(frame_buffer);
    free_meshes(meshes, 2);
    ge_close_gif(gif);
    
    return 0;
}