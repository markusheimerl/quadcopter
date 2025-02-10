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
#define SIM_TIME    10.0  // Simulation duration in seconds

int main() {
    srand(time(NULL));
    
    // Initialize random target position and yaw
    double target[7] = {
        (double)rand() / RAND_MAX * 4.0 - 2.0,    // x: [-2,2]
        (double)rand() / RAND_MAX * 2.0 + 0.5,    // y: [0.5,2.5]
        (double)rand() / RAND_MAX * 4.0 - 2.0,    // z: [-2,2]
        0.0, 0.0, 0.0,                            // Zero velocity target
        (double)rand() / RAND_MAX * 2.0 * M_PI    // yaw: [0,2π]
    };
    
    printf("Target position: (%.2f, %.2f, %.2f) with yaw: %.2f rad\n", 
           target[0], target[1], target[2], target[6]);
    
    // Initialize quadcopter
    Quad* quad = create_quad(0.0, 0.0, 0.0);
    
    // Initialize raytracer scene
    Scene scene = create_scene(400, 300, (int)(SIM_TIME * 1000), 24, 0.4f);
    
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
    clock_t start_time = clock();

    // Main simulation loop
    for (int t = 0; t < (int)(SIM_TIME / DT_PHYSICS); t++) {
        // Physics update
        if (t_physics >= DT_PHYSICS) {
            update_quad(quad, DT_PHYSICS);
            t_physics = 0.0;
        }
        
        // Control update
        if (t_control >= DT_CONTROL) {
            control_quad(quad, target);
            t_control = 0.0;
        }
        
        // Render update
        if (t_render >= DT_RENDER) {
            // Update drone position and orientation in the scene
            set_mesh_position(&scene.meshes[0], 
                (Vec3){(float)quad->linear_position_W[0], 
                       (float)quad->linear_position_W[1], 
                       (float)quad->linear_position_W[2]});
            
            set_mesh_rotation(&scene.meshes[0], 
                (Vec3){
                    atan2f(quad->R_W_B[7], quad->R_W_B[8]),  // roll
                    asinf(-quad->R_W_B[6]),                  // pitch
                    atan2f(quad->R_W_B[3], quad->R_W_B[0])   // yaw
                }
            );
            
            // Render frame
            render_scene(&scene);
            next_frame(&scene);
            
            // Update progress bar
            update_progress_bar((int)(t * DT_PHYSICS / DT_RENDER), (int)(SIM_TIME * 24), start_time);
            
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
    return 0;
}