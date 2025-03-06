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

// Helper function to calculate the angle between two points in the XZ plane
double calculate_yaw_to_target(double x1, double z1, double x2, double z2) {
    // Calculate direction vector from (x1,z1) to (x2,z2)
    double dx = x2 - x1;
    double dz = z2 - z1;
    
    // Compute angle (atan2 returns angle in range [-π, π])
    return atan2(dx, dz);
}

int main() {
    srand(time(NULL));
    
    // Initialize random drone position
    double drone_x = random_range(-2.0, 2.0);
    double drone_y = random_range(0.5, 2.0);
    double drone_z = random_range(-2.0, 2.0);
    
    // Initialize random drone yaw
    double drone_yaw = random_range(-M_PI, M_PI);
     
    // Calculate a random distance (between 1 and 4 units) in front of the drone
    double distance = random_range(1.0, 4.0);
    
    // Add some random deviation to make it more natural (±30° from the center of view)
    double angle_deviation = random_range(-M_PI/6, M_PI/6);  // ±30 degrees
    double adjusted_yaw = drone_yaw + angle_deviation;
    
    // Calculate the target position based on the drone's position, adjusted yaw, and distance
    double target_x = drone_x + sin(adjusted_yaw) * distance;
    double target_z = drone_z + cos(adjusted_yaw) * distance;
    
    // Keep the target within boundaries
    target_x = fmax(-2.0, fmin(2.0, target_x));
    target_z = fmax(-2.0, fmin(2.0, target_z));
    
    // Set a random target height
    double target_y = random_range(0.5, 2.5);
    
    // Calculate the target yaw to point toward the drone
    // This makes the treasure chest "face" the drone
    double target_yaw = calculate_yaw_to_target(target_x, target_z, drone_x, drone_z);
    
    // Create combined target array
    double target[7] = {
        target_x, target_y, target_z,    // Target position
        0.0, 0.0, 0.0,                  // Zero velocity target
        target_yaw                      // Target yaw pointing to drone
    };
    
    printf("Initial drone position: (%.2f, %.2f, %.2f) with yaw: %.2f rad\n", 
           drone_x, drone_y, drone_z, drone_yaw);
    printf("Target position: (%.2f, %.2f, %.2f) with yaw: %.2f rad\n", 
           target[0], target[1], target[2], target[6]);
    printf("Target is %.2f meters away in the drone's field of view (%.2f° from center)\n", 
           distance, angle_deviation * 180.0 / M_PI);
    
    // Initialize quadcopter with random position and yaw
    Quad quad = create_quad(drone_x, drone_y, drone_z, drone_yaw);
    
    // Initialize state estimator
    StateEstimator estimator = {
        .angular_velocity = {0.0, 0.0, 0.0},
        .gyro_bias = {0.0, 0.0, 0.0}
    };
    // Copy the quad's rotation matrix to the estimator
    memcpy(estimator.R, quad.R_W_B, 9 * sizeof(double));
    
    // Initialize raytracer scene
    Scene scene = create_scene(800, 600, (int)(SIM_TIME * 1000), 24, 0.4f);
    
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
    
    // Add treasure chest at target location
    Mesh treasure = create_mesh("raytracer/treasure.obj", "raytracer/treasure.webp");
    add_mesh_to_scene(&scene, treasure);
    
    // Set treasure position and rotation
    set_mesh_position(&scene.meshes[1], 
        (Vec3){(float)target[0], (float)target[1], (float)target[2]});
    set_mesh_rotation(&scene.meshes[1], 
        (Vec3){0.0f, (float)target[6], 0.0f});
    
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
            // Update state estimator
            update_estimator(
                quad.gyro_measurement,
                quad.accel_measurement,
                DT_CONTROL,
                &estimator
            );
            
            // For the drone, calculate yaw to look at a point slightly behind the target
            // This prevents the drone from constantly rotating when it's at the target
            
            // Calculate vector from drone to target
            double drone_to_target_x = target[0] - quad.linear_position_W[0];
            double drone_to_target_z = target[2] - quad.linear_position_W[2];
            
            // Calculate distance to target in xz plane
            double xz_distance = sqrt(drone_to_target_x * drone_to_target_x + drone_to_target_z * drone_to_target_z);
            
            // If drone is far enough away, point directly at target
            if (xz_distance > 0.3) {
                target_yaw = calculate_yaw_to_target(
                    quad.linear_position_W[0],
                    quad.linear_position_W[2],
                    target[0],
                    target[2]
                );
            }
            
            // Use this yaw as the target yaw for the controller
            double control_target[7];
            memcpy(control_target, target, 7 * sizeof(double));
            control_target[6] = target_yaw;
            
            double new_omega[4];
            control_quad_commands(
                quad.linear_position_W,
                quad.linear_velocity_W,
                estimator.R,
                estimator.angular_velocity,
                quad.inertia,
                control_target,
                new_omega
            );
            memcpy(quad.omega_next, new_omega, 4 * sizeof(double));
            t_control = 0.0;
        }
        
        // Render update
        if (t_render >= DT_RENDER) {
            // Update drone position and rotation
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

    printf("\nFinal position: (%.2f, %.2f, %.2f) with yaw %.2f or ±%.2f\n", 
           quad.linear_position_W[0], quad.linear_position_W[1], quad.linear_position_W[2],
           asinf(-quad.R_W_B[6]), M_PI - fabs(asinf(-quad.R_W_B[6])));
           
    // Calculate distance to target
    double dist = sqrt(pow(quad.linear_position_W[0] - target[0], 2) + 
                     pow(quad.linear_position_W[1] - target[1], 2) + 
                     pow(quad.linear_position_W[2] - target[2], 2));
    printf("Distance to target: %.2f meters\n", dist);

    // Save animation
    char filename[64];
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_flight.webp", localtime(&(time_t){time(NULL)}));
    save_scene(&scene, filename);
    printf("Animation saved to: %s\n", filename);

    // Cleanup
    destroy_mesh(&drone);
    destroy_mesh(&treasure);
    destroy_mesh(&ground);
    destroy_scene(&scene);
    return 0;
}