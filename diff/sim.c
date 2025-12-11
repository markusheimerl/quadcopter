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
// ENZYME AUTODIFF DECLARATIONS
// ============================================================================

extern double __enzyme_autodiff(void*, double);

// Fixed initial conditions for optimization
static double IC_DRONE_X, IC_DRONE_Y, IC_DRONE_Z, IC_DRONE_YAW;
static double IC_TARGET_X, IC_TARGET_Y, IC_TARGET_Z, IC_TARGET_YAW;

// Global gains for wrapper functions
static double G_KV, G_KR, G_KW;
static double G_KP, G_KR2, G_KW2;
static double G_KP2, G_KV2, G_KW3;
static double G_KP3, G_KV3, G_KR3;

// ============================================================================
// LOSS FUNCTION (physics-only simulation)
// ============================================================================

double simulate_for_loss(double K_P, double K_V, double K_R, double K_W) {
    // Initialize quad with fixed initial conditions
    Quad quad = create_quad(IC_DRONE_X, IC_DRONE_Y, IC_DRONE_Z, IC_DRONE_YAW);
    
    // Fixed target
    double target[7] = {
        IC_TARGET_X, IC_TARGET_Y, IC_TARGET_Z,
        0.0, 0.0, 0.0,
        IC_TARGET_YAW
    };
    
    double t_physics = 0.0;
    double t_control = 0.0;
    
    // Run simulation
    for (int step = 0; step < (int)(SIM_TIME / DT_PHYSICS); step++) {
        // Physics update
        if (t_physics >= DT_PHYSICS) {
            double new_position[3], new_velocity[3], new_angular_velocity[3];
            double new_quaternion[4], new_omega[4];
            
            update_quad_states(
                quad.omega, quad.linear_position_W, quad.linear_velocity_W,
                quad.angular_velocity_B, quad.q_W_B, quad.inertia,
                quad.omega_next, DT_PHYSICS,
                new_position, new_velocity, new_angular_velocity,
                new_quaternion, new_omega
            );
            
            memcpy(quad.linear_position_W, new_position, 3 * sizeof(double));
            memcpy(quad.linear_velocity_W, new_velocity, 3 * sizeof(double));
            memcpy(quad.angular_velocity_B, new_angular_velocity, 3 * sizeof(double));
            memcpy(quad.q_W_B, new_quaternion, 4 * sizeof(double));
            memcpy(quad.omega, new_omega, 4 * sizeof(double));
            
            t_physics = 0.0;
        }
        
        // Control update
        if (t_control >= DT_CONTROL) {
            double omega_cmd[4];
            
            control_quad_commands(
                quad.linear_position_W, quad.linear_velocity_W,
                quad.q_W_B, quad.angular_velocity_B, quad.inertia,
                target, K_P, K_V, K_R, K_W, omega_cmd
            );
            
            memcpy(quad.omega_next, omega_cmd, 4 * sizeof(double));
            t_control = 0.0;
        }
        
        t_physics += DT_PHYSICS;
        t_control += DT_PHYSICS;
    }
    
    // Return position error
    double error = sqrt(
        pow(quad.linear_position_W[0] - target[0], 2) +
        pow(quad.linear_position_W[1] - target[1], 2) +
        pow(quad.linear_position_W[2] - target[2], 2)
    );
    
    return error;
}

// ============================================================================
// WRAPPER FUNCTIONS FOR ENZYME (one parameter each)
// ============================================================================

double loss_wrt_KP(double K_P) {
    return simulate_for_loss(K_P, G_KV, G_KR, G_KW);
}

double loss_wrt_KV(double K_V) {
    return simulate_for_loss(G_KP, K_V, G_KR2, G_KW2);
}

double loss_wrt_KR(double K_R) {
    return simulate_for_loss(G_KP2, G_KV2, K_R, G_KW3);
}

double loss_wrt_KW(double K_W) {
    return simulate_for_loss(G_KP3, G_KV3, G_KR3, K_W);
}

// ============================================================================
// OPTIMIZATION LOOP
// ============================================================================

void optimize_gains(double* gains) {
    printf("\n=== OPTIMIZING CONTROLLER GAINS ===\n");
    
    double learning_rate = 0.001;
    int num_iterations = 5000;
    
    for (int iter = 0; iter < num_iterations; iter++) {
        // Compute loss
        double loss = simulate_for_loss(gains[0], gains[1], gains[2], gains[3]);
        
        // Compute gradients using forward-mode AD
        G_KV = gains[1]; G_KR = gains[2]; G_KW = gains[3];
        double d_KP = __enzyme_autodiff((void*)loss_wrt_KP, gains[0]);
        
        G_KP = gains[0]; G_KR2 = gains[2]; G_KW2 = gains[3];
        double d_KV = __enzyme_autodiff((void*)loss_wrt_KV, gains[1]);
        
        G_KP2 = gains[0]; G_KV2 = gains[1]; G_KW3 = gains[3];
        double d_KR = __enzyme_autodiff((void*)loss_wrt_KR, gains[2]);
        
        G_KP3 = gains[0]; G_KV3 = gains[1]; G_KR3 = gains[2];
        double d_KW = __enzyme_autodiff((void*)loss_wrt_KW, gains[3]);
        
        // Gradient descent update
        gains[0] -= learning_rate * d_KP;
        gains[1] -= learning_rate * d_KV;
        gains[2] -= learning_rate * d_KR;
        gains[3] -= learning_rate * d_KW;
        
        // Clamp gains to reasonable range
        for (int i = 0; i < 4; i++) {
            if (gains[i] < 0.01) gains[i] = 0.01;
            if (gains[i] > 5.0) gains[i] = 5.0;
        }
        
        // Print periodically
        if (iter % 1000 == 0 || iter == num_iterations - 1) {
            printf("Iter %2d: Loss=%.6f  K_P=%.3f K_V=%.3f K_R=%.3f K_W=%.3f\n",
                   iter, loss, gains[0], gains[1], gains[2], gains[3]);
        }
    }
    
    printf("\nOptimized gains: K_P=%.3f K_V=%.3f K_R=%.3f K_W=%.3f\n\n",
           gains[0], gains[1], gains[2], gains[3]);
}

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
    // Initialize initial conditions (fixed for optimization)
    // -------------------------------------------------------------------------
    IC_DRONE_X   = random_range(-2.0, 2.0);
    IC_DRONE_Y   = random_range(0.5, 2.0);
    IC_DRONE_Z   = random_range(-2.0, 2.0);
    IC_DRONE_YAW = random_range(-M_PI, M_PI);
    
    IC_TARGET_X   = random_range(-2.0, 2.0);
    IC_TARGET_Y   = random_range(0.5, 2.5);
    IC_TARGET_Z   = random_range(-2.0, 2.0);
    IC_TARGET_YAW = random_range(-M_PI, M_PI);
    
    printf("Drone starts at (%.2f, %.2f, %.2f) with yaw %.2f\n", 
           IC_DRONE_X, IC_DRONE_Y, IC_DRONE_Z, IC_DRONE_YAW);
    printf("Target at (%.2f, %.2f, %.2f) with yaw %.2f\n", 
           IC_TARGET_X, IC_TARGET_Y, IC_TARGET_Z, IC_TARGET_YAW);
    
    // -------------------------------------------------------------------------
    // Optimize controller gains
    // -------------------------------------------------------------------------
    double gains[4] = {0.2, 0.6, 0.6, 0.6};  // Initial gains
    optimize_gains(gains);
    
    // -------------------------------------------------------------------------
    // Run final simulation with visualization
    // -------------------------------------------------------------------------
    printf("=== RUNNING VISUALIZATION WITH OPTIMIZED GAINS ===\n");
    
    Quad quad = create_quad(IC_DRONE_X, IC_DRONE_Y, IC_DRONE_Z, IC_DRONE_YAW);
    
    double target[7] = {
        IC_TARGET_X, IC_TARGET_Y, IC_TARGET_Z,
        0.0, 0.0, 0.0,
        IC_TARGET_YAW
    };
    
    // Initialize raytracer scene
    Scene scene = create_scene(400, 300, (int)(SIM_TIME * 1000), 24, 0.4f);
    
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
    
    // Load meshes
    Mesh drone_mesh = create_mesh("../raytracer/assets/drone.obj", 
                                   "../raytracer/assets/drone.webp");
    add_mesh_to_scene(&scene, drone_mesh);
    
    Mesh treasure = create_mesh("../raytracer/assets/treasure.obj", 
                                 "../raytracer/assets/treasure.webp");
    add_mesh_to_scene(&scene, treasure);
    set_mesh_position(&scene.meshes[1], 
        (Vec3){(float)IC_TARGET_X, (float)IC_TARGET_Y, (float)IC_TARGET_Z});
    
    Mesh ground = create_mesh("../raytracer/assets/ground.obj", 
                               "../raytracer/assets/ground.webp");
    add_mesh_to_scene(&scene, ground);

    // Simulation loop timers
    double t_physics = 0.0;
    double t_control = 0.0;
    double t_render  = 0.0;
    clock_t start_time = clock();

    // Main simulation loop
    for (int step = 0; step < (int)(SIM_TIME / DT_PHYSICS); step++) {
        
        // Physics update
        if (t_physics >= DT_PHYSICS) {
            double new_position[3], new_velocity[3], new_angular_velocity[3];
            double new_quaternion[4], new_omega[4];
            
            update_quad_states(
                quad.omega, quad.linear_position_W, quad.linear_velocity_W,
                quad.angular_velocity_B, quad.q_W_B, quad.inertia,
                quad.omega_next, DT_PHYSICS,
                new_position, new_velocity, new_angular_velocity,
                new_quaternion, new_omega
            );
            
            memcpy(quad.linear_position_W, new_position, 3 * sizeof(double));
            memcpy(quad.linear_velocity_W, new_velocity, 3 * sizeof(double));
            memcpy(quad.angular_velocity_B, new_angular_velocity, 3 * sizeof(double));
            memcpy(quad.q_W_B, new_quaternion, 4 * sizeof(double));
            memcpy(quad.omega, new_omega, 4 * sizeof(double));
            
            t_physics = 0.0;
        }
        
        // Control update
        if (t_control >= DT_CONTROL) {
            double omega_cmd[4];
            
            control_quad_commands(
                quad.linear_position_W, quad.linear_velocity_W,
                quad.q_W_B, quad.angular_velocity_B, quad.inertia,
                target, gains[0], gains[1], gains[2], gains[3], omega_cmd
            );
            
            memcpy(quad.omega_next, omega_cmd, 4 * sizeof(double));
            t_control = 0.0;
        }
        
        // Render update
        if (t_render >= DT_RENDER) {
            set_mesh_position(&scene.meshes[0], (Vec3){
                (float)quad.linear_position_W[0], 
                (float)quad.linear_position_W[1], 
                (float)quad.linear_position_W[2]
            });
            
            double R[9];
            quatToRotMat(quad.q_W_B, R);
            
            set_mesh_rotation(&scene.meshes[0], (Vec3){
                atan2f(R[7], R[8]),
                asinf(-R[6]),
                atan2f(R[3], R[0])
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
        
        t_physics += DT_PHYSICS;
        t_control += DT_PHYSICS;
        t_render  += DT_PHYSICS;
    }

    // Display final results
    double R_final[9];
    quatToRotMat(quad.q_W_B, R_final);
    double final_yaw = asinf(-R_final[6]);
    
    printf("\nFinal position: (%.2f, %.2f, %.2f)\n", 
           quad.linear_position_W[0], 
           quad.linear_position_W[1], 
           quad.linear_position_W[2]);
    printf("Final yaw: %.2f rad or ±%.2f rad (target: %.2f rad)\n", 
           final_yaw, M_PI - fabs(final_yaw), IC_TARGET_YAW);
    
    double pos_error = sqrt(
        pow(quad.linear_position_W[0] - IC_TARGET_X, 2) +
        pow(quad.linear_position_W[1] - IC_TARGET_Y, 2) +
        pow(quad.linear_position_W[2] - IC_TARGET_Z, 2)
    );
    printf("Position error: %.3f m\n", pos_error);

    // Save animation
    char filename[64];
    time_t current_time = time(NULL);
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_flight.webp", 
             localtime(&current_time));
    save_scene(&scene, filename);
    printf("Animation saved to: %s\n", filename);

    // Cleanup
    destroy_mesh(&drone_mesh);
    destroy_mesh(&treasure);
    destroy_mesh(&ground);
    destroy_scene(&scene);
    
    return 0;
}