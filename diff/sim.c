#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include "quad.h"
#include "scene.h"

// ============================================================================
// SIMULATION PARAMETERS
// ============================================================================

#define DT_PHYSICS  (1.0 / 1000.0)   // Physics timestep: 1000 Hz
#define DT_CONTROL  (1.0 / 60.0)     // Control timestep: 60 Hz
#define DT_RENDER   (1.0 / 24.0)     // Render timestep: 24 Hz
#define SIM_TIME    10.0             // Total simulation time [s]

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

double random_range(double min, double max) {
    return min + (double)rand() / RAND_MAX * (max - min);
}

// ============================================================================
// MAIN SIMULATION LOOP
// ============================================================================

int main() {
    srand(time(NULL));
    
    // -------------------------------------------------------------------------
    // Initialize drone state (random position and yaw)
    // -------------------------------------------------------------------------
    double drone_x   = random_range(-2.0, 2.0);
    double drone_y   = random_range(0.5, 2.0);
    double drone_z   = random_range(-2.0, 2.0);
    double drone_yaw = random_range(-M_PI, M_PI);
    
    // -------------------------------------------------------------------------
    // Initialize target (random position and yaw)
    // -------------------------------------------------------------------------
    double target_x   = random_range(-2.0, 2.0);
    double target_y   = random_range(0.5, 2.5);
    double target_z   = random_range(-2.0, 2.0);
    double target_yaw = random_range(-M_PI, M_PI);
    
    // Target state: [position(3), velocity(3), yaw(1)]
    double target[7] = {
        target_x, target_y, target_z,   // Desired position
        0.0, 0.0, 0.0,                  // Desired velocity (hover)
        target_yaw                      // Desired yaw
    };
    
    printf("Drone starts at (%.2f, %.2f, %.2f) with yaw %.2f\n", 
           drone_x, drone_y, drone_z, drone_yaw);
    printf("Target at (%.2f, %.2f, %.2f) with yaw %.2f\n", 
           target_x, target_y, target_z, target_yaw);
    
    // -------------------------------------------------------------------------
    // Create quadcopter
    // -------------------------------------------------------------------------
    Quad quad = create_quad(drone_x, drone_y, drone_z, drone_yaw);
    
    // -------------------------------------------------------------------------
    // Initialize raytracer scene
    // -------------------------------------------------------------------------
    Scene scene = create_scene(400, 300, (int)(SIM_TIME * 1000), 24, 0.4f);
    
    set_scene_camera(&scene,
        (Vec3){-3.0f, 3.0f, -3.0f},   // Camera position
        (Vec3){0.0f, 0.0f, 0.0f},     // Look-at point
        (Vec3){0.0f, 1.0f, 0.0f},     // Up vector
        60.0f                         // Field of view
    );
    
    set_scene_light(&scene,
        (Vec3){1.0f, 1.0f, -1.0f},    // Light direction
        (Vec3){1.4f, 1.4f, 1.4f}      // Light intensity
    );
    
    // Load meshes
    Mesh drone_mesh = create_mesh("../raytracer/assets/drone.obj", 
                                   "../raytracer/assets/drone.webp");
    add_mesh_to_scene(&scene, drone_mesh);
    
    Mesh treasure = create_mesh("../raytracer/assets/treasure.obj", 
                                 "../raytracer/assets/treasure.webp");
    add_mesh_to_scene(&scene, treasure);
    set_mesh_position(&scene.meshes[1], 
        (Vec3){(float)target_x, (float)target_y, (float)target_z});
    
    Mesh ground = create_mesh("../raytracer/assets/ground.obj", 
                               "../raytracer/assets/ground.webp");
    add_mesh_to_scene(&scene, ground);

    // -------------------------------------------------------------------------
    // Simulation loop timers
    // -------------------------------------------------------------------------
    double t_physics = 0.0;
    double t_control = 0.0;
    double t_render  = 0.0;
    clock_t start_time = clock();

    // -------------------------------------------------------------------------
    // Main simulation loop
    // -------------------------------------------------------------------------
    for (int step = 0; step < (int)(SIM_TIME / DT_PHYSICS); step++) {
        
        // ---------------------------------------------------------------------
        // Physics update (1000 Hz)
        // ---------------------------------------------------------------------
        if (t_physics >= DT_PHYSICS) {
            double new_position[3];
            double new_velocity[3];
            double new_angular_velocity[3];
            double new_quaternion[4];
            double new_omega[4];
            
            update_quad_states(
                quad.omega,
                quad.linear_position_W,
                quad.linear_velocity_W,
                quad.angular_velocity_B,
                quad.q_W_B,
                quad.inertia,
                quad.omega_next,
                DT_PHYSICS,
                new_position,
                new_velocity,
                new_angular_velocity,
                new_quaternion,
                new_omega
            );
            
            memcpy(quad.linear_position_W, new_position, 3 * sizeof(double));
            memcpy(quad.linear_velocity_W, new_velocity, 3 * sizeof(double));
            memcpy(quad.angular_velocity_B, new_angular_velocity, 3 * sizeof(double));
            memcpy(quad.q_W_B, new_quaternion, 4 * sizeof(double));
            memcpy(quad.omega, new_omega, 4 * sizeof(double));
            
            t_physics = 0.0;
        }
        
        // ---------------------------------------------------------------------
        // Control update (60 Hz)
        // ---------------------------------------------------------------------
        if (t_control >= DT_CONTROL) {
            double omega_cmd[4];
            
            control_quad_commands(
                quad.linear_position_W,
                quad.linear_velocity_W,
                quad.q_W_B,
                quad.angular_velocity_B,
                quad.inertia,
                target,
                omega_cmd
            );
            
            memcpy(quad.omega_next, omega_cmd, 4 * sizeof(double));
            t_control = 0.0;
        }
        
        // ---------------------------------------------------------------------
        // Render update (24 Hz)
        // ---------------------------------------------------------------------
        if (t_render >= DT_RENDER) {
            // Update drone position
            set_mesh_position(&scene.meshes[0], (Vec3){
                (float)quad.linear_position_W[0], 
                (float)quad.linear_position_W[1], 
                (float)quad.linear_position_W[2]
            });
            
            // Convert quaternion to rotation matrix for Euler angle extraction
            double R[9];
            quatToRotMat(quad.q_W_B, R);
            
            // Extract Euler angles (XYZ convention, Y-up)
            // Roll  (X): atan2(R[2,1], R[2,2])
            // Pitch (Y): asin(-R[2,0])
            // Yaw   (Z): atan2(R[1,0], R[0,0])
            set_mesh_rotation(&scene.meshes[0], (Vec3){
                atan2f(R[7], R[8]),    // Roll
                asinf(-R[6]),          // Pitch (actually yaw in Y-up)
                atan2f(R[3], R[0])     // Yaw (actually roll in Y-up)
            });
            
            render_scene(&scene);
            next_frame(&scene);
            update_progress_bar(
                (int)(step * DT_PHYSICS / DT_RENDER), 
                (int)(SIM_TIME * 24), 
                start_time
            );
            
            t_render = 0.0;
        }
        
        // Increment timers
        t_physics += DT_PHYSICS;
        t_control += DT_PHYSICS;
        t_render  += DT_PHYSICS;
    }

    // -------------------------------------------------------------------------
    // Display final results
    // -------------------------------------------------------------------------
    double R_final[9];
    quatToRotMat(quad.q_W_B, R_final);
    double final_yaw = asinf(-R_final[6]);
    
    printf("\nFinal position: (%.2f, %.2f, %.2f)\n", 
           quad.linear_position_W[0], 
           quad.linear_position_W[1], 
           quad.linear_position_W[2]);
    printf("Final yaw: %.2f rad or ±%.2f rad (target: %.2f rad)\n", final_yaw, M_PI - fabs(final_yaw), target_yaw);
    
    double pos_error = sqrt(
        pow(quad.linear_position_W[0] - target_x, 2) +
        pow(quad.linear_position_W[1] - target_y, 2) +
        pow(quad.linear_position_W[2] - target_z, 2)
    );
    printf("Position error: %.3f m\n", pos_error);

    // -------------------------------------------------------------------------
    // Save animation
    // -------------------------------------------------------------------------
    char filename[64];
    time_t current_time = time(NULL);
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_flight.webp", 
             localtime(&current_time));
    save_scene(&scene, filename);
    printf("Animation saved to: %s\n", filename);

    // -------------------------------------------------------------------------
    // Cleanup
    // -------------------------------------------------------------------------
    destroy_mesh(&drone_mesh);
    destroy_mesh(&treasure);
    destroy_mesh(&ground);
    destroy_scene(&scene);
    
    return 0;
}