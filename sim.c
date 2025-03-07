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

// Helper function to get random value in range [min, max]
double random_range(double min, double max) {
    return min + (double)rand() / RAND_MAX * (max - min);
}

int main() {
    srand(time(NULL));
    
    // Initialize drone with random position and orientation
    double drone_x = random_range(-2.0, 2.0);
    double drone_y = random_range(0.5, 2.0);
    double drone_z = random_range(-2.0, 2.0);
    double drone_yaw = random_range(-M_PI, M_PI);
    
    // Place target in front of drone with slight randomization
    double distance = random_range(3.0, 4.0);
    double angle_offset = random_range(-M_PI/6, M_PI/6); // Small random offset
    double target_yaw = drone_yaw + angle_offset;
    
    // Calculate target position
    double target_x = drone_x + sin(target_yaw) * distance;
    double target_z = drone_z + cos(target_yaw) * distance;
    double target_y = random_range(0.5, 2.5);
    
    // Keep target within boundaries
    target_x = fmax(-2.0, fmin(2.0, target_x));
    target_z = fmax(-2.0, fmin(2.0, target_z));
    
    // Create target array (position, velocity, and desired yaw)
    double target[7] = {
        target_x, target_y, target_z,    // Target position
        0.0, 0.0, 0.0,                   // Zero velocity target
        atan2(target_x - drone_x, target_z - drone_z)  // Yaw to face target
    };
    
    printf("Drone starts at (%.2f, %.2f, %.2f), target at (%.2f, %.2f, %.2f)\n", 
           drone_x, drone_y, drone_z, target_x, target_y, target_z);
    printf("Target is %.2f meters away\n", distance);
    
    // Initialize quadcopter
    Quad quad = create_quad(drone_x, drone_y, drone_z, drone_yaw);
    
    // Initialize state estimator
    StateEstimator estimator = {
        .angular_velocity = {0.0, 0.0, 0.0},
        .gyro_bias = {0.0, 0.0, 0.0}
    };
    memcpy(estimator.R, quad.R_W_B, 9 * sizeof(double));
    
    // Initialize raytracer scene
    Scene scene = create_scene(400, 300, (int)(SIM_TIME * 1000), 24, 0.4f);
    
    // Set up camera and lighting
    set_scene_camera(&scene,
        (Vec3){-3.0f, 3.0f, -3.0f},
        (Vec3){0.0f, 0.0f, 0.0f},
        (Vec3){0.0f, 1.0f, 0.0f},
        60.0f
    );
    
    set_scene_light(&scene,
        (Vec3){1.0f, 1.0f, -1.0f},
        (Vec3){1.4f, 1.4f, 1.4f}
    );
    
    // Add meshes to scene
    Mesh drone_mesh = create_mesh("raytracer/drone.obj", "raytracer/drone.webp");
    add_mesh_to_scene(&scene, drone_mesh);
    
    Mesh treasure = create_mesh("raytracer/treasure.obj", "raytracer/treasure.webp");
    add_mesh_to_scene(&scene, treasure);
    set_mesh_position(&scene.meshes[1], (Vec3){(float)target_x, (float)target_y, (float)target_z});
    
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
            update_quad(&quad, DT_PHYSICS);
            t_physics = 0.0;
        }
        
        // Control update
        if (t_control >= DT_CONTROL) {
            update_estimator(
                quad.gyro_measurement,
                quad.accel_measurement,
                DT_CONTROL,
                &estimator
            );
            
            double new_omega[4];
            control_quad_commands(
                quad.linear_position_W,
                quad.linear_velocity_W,
                estimator.R,
                estimator.angular_velocity,
                quad.inertia,
                target,
                new_omega
            );
            memcpy(quad.omega_next, new_omega, 4 * sizeof(double));
            t_control = 0.0;
        }
        
        // Render update
        if (t_render >= DT_RENDER) {
            // Update drone position and rotation in the scene
            set_mesh_position(&scene.meshes[0], 
                (Vec3){(float)quad.linear_position_W[0], 
                       (float)quad.linear_position_W[1], 
                       (float)quad.linear_position_W[2]});
            
            set_mesh_rotation(&scene.meshes[0], 
                (Vec3){
                    atan2f(quad.R_W_B[7], quad.R_W_B[8]),
                    asinf(-quad.R_W_B[6]),
                    atan2f(quad.R_W_B[3], quad.R_W_B[0])
                }
            );
            
            render_scene(&scene);
            next_frame(&scene);
            update_progress_bar((int)(t * DT_PHYSICS / DT_RENDER), (int)(SIM_TIME * 24), start_time);
            t_render = 0.0;
        }
        
        // Increment timers
        t_physics += DT_PHYSICS;
        t_control += DT_PHYSICS;
        t_render += DT_PHYSICS;
    }

    // Display final results
    double dist = sqrt(pow(quad.linear_position_W[0] - target_x, 2) + 
                     pow(quad.linear_position_W[1] - target_y, 2) + 
                     pow(quad.linear_position_W[2] - target_z, 2));
    
    printf("\nFinal position: (%.2f, %.2f, %.2f)\n", 
           quad.linear_position_W[0], quad.linear_position_W[1], quad.linear_position_W[2]);
    printf("Distance to target: %.2f meters\n", dist);

    // Save animation
    char filename[64];
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_flight.webp", localtime(&(time_t){time(NULL)}));
    save_scene(&scene, filename);
    printf("Animation saved to: %s\n", filename);

    // Cleanup
    destroy_mesh(&drone_mesh);
    destroy_mesh(&treasure);
    destroy_mesh(&ground);
    destroy_scene(&scene);
    return 0;
}