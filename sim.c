#include "gif.h"
#include "rasterizer.h"
#include "quad.h"
#include <time.h>

#define STEPS 600

int main() {
    #ifdef RENDER
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

    // Transform ground mesh
    transform_mesh(meshes[1], (double[3]){0.0, -0.5, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});
    #endif

    #ifdef LOG
    // Open CSV file for logging and write header
    FILE *csv_file = fopen("drone_data.csv", "w");
    fprintf(csv_file, "step,linear_position_d_W[0],linear_position_d_W[1],linear_position_d_W[2],yaw_d,angular_velocity_B[0],angular_velocity_B[1],angular_velocity_B[2],linear_acceleration_B[0],linear_acceleration_B[1],linear_acceleration_B[2],omega_next[0],omega_next[1],omega_next[2],omega_next[3]\n");
    
    // Set desired state to random values
    srand(time(NULL));
    linear_position_d_W[0] = (double)rand() / RAND_MAX * 10 - 5; // [-5, 5]
    linear_position_d_W[1] = (double)rand() / RAND_MAX * 10;     // [0, 10]
    linear_position_d_W[2] = (double)rand() / RAND_MAX * 10 - 5; // [-5, 5]
    yaw_d = (double)rand() / RAND_MAX * 2 * M_PI;                // [0, 2pi]
    #endif

    // Main simulation loop
    for(int step = 0; step < STEPS; step++) {
        // Update dynamics
        update_drone_physics();

        // Control
        update_drone_control();

        #ifdef LOG
        // Log data to CSV
        fprintf(csv_file, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", step, linear_position_d_W[0], linear_position_d_W[1], linear_position_d_W[2], yaw_d, angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2], linear_acceleration_B[0], linear_acceleration_B[1], linear_acceleration_B[2], omega_next[0], omega_next[1], omega_next[2], omega_next[3]);
        #endif

        // Update rotor speeds
        update_rotor_speeds();

        #ifdef RENDER
        // Render frame and add to GIF and transform drone mesh
        transform_mesh(meshes[0], (double[3]){linear_position_W[0], linear_position_W[1], linear_position_W[2]}, 0.5, R_W_B);
        memset(frame_buffer, 0, WIDTH * HEIGHT * 3);
        vertex_shader(meshes, 2, camera_pos, camera_target);
        rasterize(frame_buffer, meshes, 2);
        ge_add_frame(gif, frame_buffer, 6);
        #endif
        
        // Print state
        printf("Step %d/%d\n", step + 1, STEPS);
        printf("Position: [%.3f, %.3f, %.3f]\n", linear_position_W[0], linear_position_W[1], linear_position_W[2]);
        printf("Angular Velocity: [%.3f, %.3f, %.3f]\n", angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2]);
        printf("---\n");
    }

    #ifdef LOG
    // Close CSV file
    fclose(csv_file);
    #endif

    #ifdef RENDER
    // Cleanup
    free(frame_buffer);
    free_meshes(meshes, 2);
    ge_close_gif(gif);
    #endif
    
    return 0;
}