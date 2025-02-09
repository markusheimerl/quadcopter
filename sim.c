#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include "quad.h"
#include "scene.h"

#define DT_PHYSICS  (1.0 / 1000.0)
#define DT_CONTROL  (1.0 / 60.0)
#define DT_RENDER   (1.0 / 24.0)

int main() {
    srand(time(NULL));
    
    // Initialize random target position and yaw
    double target_x = (double)rand() / RAND_MAX * 4.0 - 2.0;  // Range: -2 to 2
    double target_y = 1.5;  // Fixed height
    double target_z = (double)rand() / RAND_MAX * 4.0 - 2.0;  // Range: -2 to 2
    double target_yaw = (double)rand() / RAND_MAX * 2.0 * M_PI;  // Range: 0 to 2π
    
    printf("Target position: (%.2f, %.2f, %.2f) with yaw: %.2f rad\n", 
           target_x, target_y, target_z, target_yaw);
    
    // Initialize quadcopter
    Quad* quad = init_quad(0.0, 0.0, 0.0);
    
    // Initialize raytracer scene
    Scene scene = create_scene(400, 300, 10000, 24, 0.4f);
    
    // Set up camera
    set_scene_camera(&scene,
        (Vec3){-3.0f, 3.0f, -3.0f},
        (Vec3){0.0f, 0.0f, 0.0f},
        (Vec3){0.0f, 1.0f, 0.0f},
        60.0f
    );
    
    // Set up light
    set_scene_light(&scene,
        (Vec3){1.0f, 1.0f, -1.0f},     // Direction
        (Vec3){1.4f, 1.4f, 1.4f}       // White light
    );
    
    // Add meshes to scene
    Mesh drone = create_mesh("raytracer/drone.obj", "raytracer/drone.webp");
    add_mesh_to_scene(&scene, drone);
    
    Mesh ground = create_mesh("raytracer/ground.obj", "raytracer/ground.webp");
    add_mesh_to_scene(&scene, ground);

    // Initialize timers
    double t_physics = 0.0;
    double t_control = 0.0;
    double t_render = 0.0;
    int frame = 0;
    clock_t start_time = clock();

    // Main simulation loop
    while (frame < scene.frame_count) {
        // Physics update
        if (t_physics >= DT_PHYSICS) {
            update_quad(quad, DT_PHYSICS);
            t_physics = 0.0;
        }
        
        // Control update
        if (t_control >= DT_CONTROL) {
            double control_input[7] = {
                target_x, target_y, target_z,  // Position
                0.0, 0.0, 0.0,                 // Velocity
                target_yaw                     // Desired yaw angle
            };
            control_quad(quad, control_input);
            t_control = 0.0;
        }
        
        // Render update
        if (t_render >= DT_RENDER) {
            // Update drone position and orientation in the scene
            set_mesh_position(&scene.meshes[0], 
                (Vec3){(float)quad->linear_position_W[0], 
                       (float)quad->linear_position_W[1], 
                       (float)quad->linear_position_W[2]});
            
            // Convert rotation matrix to Euler angles for visualization
            float roll = atan2f(quad->R_W_B[7], quad->R_W_B[8]);
            float pitch = asinf(-quad->R_W_B[6]);
            float yaw = atan2f(quad->R_W_B[3], quad->R_W_B[0]);
            
            set_mesh_rotation(&scene.meshes[0], (Vec3){roll, pitch, yaw});
            
            // Render frame
            render_scene(&scene);
            next_frame(&scene);
            
            // Update progress bar
            update_progress_bar(frame, scene.frame_count, start_time);
            
            frame++;
            t_render = 0.0;
        }
        
        // Increment timers
        t_physics += DT_PHYSICS;
        t_control += DT_PHYSICS;
        t_render += DT_PHYSICS;
    }

    // Save animation
    char filename[64];
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_flight.webp", 
             localtime(&(time_t){time(NULL)}));
    save_scene(&scene, filename);

    // Cleanup
    destroy_scene(&scene);
    free(quad);
    
    printf("\nSimulation completed\n");
    return 0;
}