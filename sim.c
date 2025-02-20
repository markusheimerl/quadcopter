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

//────────────────────────────────────────────────────────────
// A simple attitude estimator:
// It integrates the gyroscope and uses the accelerometer to correct tilt drift.
// The estimated rotation matrix R_est (from body to world) is updated by:
//   Ṙ_est = R_est · hat(ω_meas + K_acc · error)
// where error = acc_meas_normalized × (0, -1, 0)
// (When the quad is in steady hover, the accelerometer reading in body frame
// should be nearly (0, -g, 0), and the normalized value is (0,-1,0).)
//────────────────────────────────────────────────────────────
void update_estimator(const double *gyro_measurement, const double *accel_measurement, 
                     double dt, double *R_est) {
    // 1. Get the gyroscope measurement (in body frame)
    double omega[3];
    memcpy(omega, gyro_measurement, 3 * sizeof(double));
    
    // 2. Get accelerometer measurement and normalize it.
    //    (We assume that for low accelerations the accelerometer measures gravity.)
    double a[3];
    memcpy(a, accel_measurement, 3 * sizeof(double));
    double norm = sqrt(dotVec3f(a, a));
    if (norm > 1e-6) {
        a[0] /= norm; a[1] /= norm; a[2] /= norm;
    }
    
    // 3. Compute an error signal from the accelerometer.
    //    The expected acceleration (in body frame) when "down" is measured is (0, -1, 0).
    double gravity_ref[3] = {0, -1, 0};
    double error[3];
    crossVec3f(a, gravity_ref, error);
    // Correction gain – this is a tuning parameter.
    double K_acc = 0.1;
    double correction[3];
    multScalVec3f(K_acc, error, correction);
    
    // 4. Combine to get a corrected rotation rate (in body frame)
    double omega_corr[3];
    for (int i = 0; i < 3; i++) {
        omega_corr[i] = omega[i] + correction[i];
    }
    
    // 5. Form the skew–symmetric matrix (hat operator) from omega_corr.
    double omega_hat[9];
    so3hat(omega_corr, omega_hat);
    
    // 6. Update the estimated rotation matrix.
    //    Since dR/dt = R_est * omega_hat, using Euler integration:
    double R_dot[9];
    multMat3f(R_est, omega_hat, R_dot);
    
    double dR[9];
    multScalMat3f(dt, R_dot, dR);
    
    // Use a temporary matrix to hold the new estimate:
    double new_R[9];
    for (int i = 0; i < 9; i++) {
        new_R[i] = R_est[i] + dR[i];
    }
    memcpy(R_est, new_R, 9 * sizeof(double));
    
    // 7. Re-orthonormalize to keep R_est a proper rotation matrix.
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