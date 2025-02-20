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

void update_estimator(const double *gyro, const double *accel, double dt, double *R_est) {
    // 1. Normalize accelerometer reading
    double acc_norm = sqrt(dotVec3f(accel, accel));
    double a_norm[3] = {
        accel[0] / acc_norm,
        accel[1] / acc_norm,
        accel[2] / acc_norm
    };

    // 2. Calculate error between measured and expected gravity direction
    double error[3];
    crossVec3f(a_norm, (double[]){0, -1, 0}, error);
    
    // 3. Apply correction to gyro measurement
    double omega_corr[3];
    for (int i = 0; i < 3; i++) {
        omega_corr[i] = gyro[i] + 0.1 * error[i];  // 0.1 is correction gain
    }
    
    // 4. Update rotation matrix
    double omega_hat[9];
    so3hat(omega_corr, omega_hat);
    double R_dot[9];
    multMat3f(R_est, omega_hat, R_dot);
    
    // 5. Integrate and orthonormalize
    for (int i = 0; i < 9; i++) {
        R_est[i] += dt * R_dot[i];
    }
    orthonormalize_rotation_matrix(R_est);
}

int main() {
    srand(time(NULL));
    
    // Initialize random target position and yaw
    double target[7] = {
        (double)rand() / RAND_MAX * 4.0 - 2.0,    // x: [-2,2]
        (double)rand() / RAND_MAX * 2.0 + 0.5,    // y: [0.5,2.5]
        (double)rand() / RAND_MAX * 4.0 - 2.0,    // z: [-2,2]
        0.0, 0.0, 0.0,                            // Zero velocity target
        ((double)rand() / RAND_MAX * 2.0 - 1.0) * M_PI    // yaw: [-π,π]
    };
    
    printf("Target position: (%.2f, %.2f, %.2f) with yaw: %.2f rad\n", 
           target[0], target[1], target[2], target[6]);
    
    // Initialize quadcopter
    Quad* quad = create_quad(0.0, 0.0, 0.0);
    
    // Initialize our estimated rotation matrix to identity.
    double R_est[9] = { 1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0 };
    
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
            update_estimator(quad->gyro_measurement, quad->accel_measurement, DT_CONTROL, R_est);
            double new_omega[4];
            control_quad_commands(
                quad->linear_position_W,
                quad->linear_velocity_W,
                R_est,
                quad->angular_velocity_B,
                quad->inertia,
                target,
                new_omega
            );
            memcpy(quad->omega_next, new_omega, 4 * sizeof(double));
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
                    atan2f(quad->R_W_B[7], quad->R_W_B[8]),
                    asinf(-quad->R_W_B[6]),
                    atan2f(quad->R_W_B[3], quad->R_W_B[0])
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

    printf("Final position: (%.2f, %.2f, %.2f) with yaw %.2f or ±%.2f (depending on interpretation)\n", 
           quad->linear_position_W[0], quad->linear_position_W[1], quad->linear_position_W[2],
           asinf(-quad->R_W_B[6]), M_PI - fabs(asinf(-quad->R_W_B[6])));
    
    //────────────────────────────────────────────
    // Print the estimated vs true rotation matrices.
    // Our matrices are stored in row-major order.
    //────────────────────────────────────────────
    printf("\nEstimated rotation matrix (R_est):\n");
    for (int i = 0; i < 3; i++) {
        printf("%.4f %.4f %.4f\n", R_est[i*3+0], R_est[i*3+1], R_est[i*3+2]);
    }
    
    printf("\nTrue rotation matrix (quad->R_W_B):\n");
    for (int i = 0; i < 3; i++) {
        printf("%.4f %.4f %.4f\n", quad->R_W_B[i*3+0], quad->R_W_B[i*3+1], quad->R_W_B[i*3+2]);
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