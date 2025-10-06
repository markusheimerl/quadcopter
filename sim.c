#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include <math.h>
#include "quad.h"

#ifndef NO_RENDER
#include "scene.h"
#endif

#define DT_PHYSICS  (1.0 / 1000.0)
#define DT_CONTROL  (1.0 / 60.0)
#define DT_RENDER   (1.0 / 24.0)
#define DT_LOG      (1.0 / 60.0)

// Helper function to get random value in range [min, max]
double random_range(double min, double max) {
    return min + (double)rand() / RAND_MAX * (max - min);
}

// Check if quad is close to target
bool is_near_target(const double* pos, const double* target_pos, double threshold) {
    double dx = pos[0] - target_pos[0];
    double dy = pos[1] - target_pos[1];
    double dz = pos[2] - target_pos[2];
    return sqrt(dx*dx + dy*dy + dz*dz) < threshold;
}

// Generate new random target
void generate_random_target(double* target) {
    target[0] = random_range(-2.0, 2.0);  // x
    target[1] = random_range(0.5, 2.5);   // y
    target[2] = random_range(-2.0, 2.0);  // z
    target[3] = 0.0;  // vx
    target[4] = 0.0;  // vy
    target[5] = 0.0;  // vz
    target[6] = random_range(-M_PI, M_PI);  // yaw
}

// Transform target from world frame to body frame using estimated state
void world_target_to_body_frame(
    const double* target_world,      // [x, y, z, vx, vy, vz, yaw]
    const double* estimated_pos,     // Estimated drone position
    const double* estimated_R,       // Estimated rotation matrix
    double* target_body              // Output: [x, y, z, vx, vy, vz, yaw_diff]
) {
    // Transform position error to body frame
    double pos_error_world[3] = {
        target_world[0] - estimated_pos[0],
        target_world[1] - estimated_pos[1],
        target_world[2] - estimated_pos[2]
    };
    
    // R is world-to-body, we want body-to-world, so transpose
    double R_T[9];
    transpMat3f(estimated_R, R_T);
    
    // Transform to body frame: R_T^T * error = R * error
    multMatVec3f(estimated_R, pos_error_world, target_body);
    
    // Transform velocity to body frame
    multMatVec3f(estimated_R, &target_world[3], &target_body[3]);
    
    // Compute yaw difference (simplified - just use target yaw for now)
    target_body[6] = target_world[6];
}

void run_data_generation(double sim_time, const char* output_file) {
    FILE* csv = fopen(output_file, "w");
    if (!csv) {
        fprintf(stderr, "Failed to open CSV file\n");
        exit(1);
    }
    
    // Write CSV header - now with relative targets
    fprintf(csv, "time,");
    fprintf(csv, "accel_x,accel_y,accel_z,");
    fprintf(csv, "gyro_x,gyro_y,gyro_z,");
    fprintf(csv, "omega_0,omega_1,omega_2,omega_3,");
    fprintf(csv, "omega_cmd_0,omega_cmd_1,omega_cmd_2,omega_cmd_3,");
    fprintf(csv, "target_rel_x,target_rel_y,target_rel_z,");
    fprintf(csv, "target_rel_vx,target_rel_vy,target_rel_vz,target_yaw\n");
    
    // Initialize quadcopter
    double drone_x = random_range(-2.0, 2.0);
    double drone_y = random_range(0.5, 2.0);
    double drone_z = random_range(-2.0, 2.0);
    double drone_yaw = random_range(-M_PI, M_PI);
    
    Quad quad = create_quad(drone_x, drone_y, drone_z, drone_yaw);
    
    // Initialize state estimator
    StateEstimator estimator = {
        .angular_velocity = {0.0, 0.0, 0.0},
        .gyro_bias = {0.0, 0.0, 0.0}
    };
    memcpy(estimator.R, quad.R_W_B, 9 * sizeof(double));
    
    // Also estimate position for relative targets
    double estimated_pos[3] = {drone_x, drone_y, drone_z};
    double estimated_vel[3] = {0.0, 0.0, 0.0};
    
    // Generate initial target
    double target_world[7];
    generate_random_target(target_world);
    
    printf("Starting data generation for %.1f seconds...\n", sim_time);
    printf("Output file: %s\n", output_file);
    
    double t_physics = 0.0, t_control = 0.0, t_log = 0.0, current_time = 0.0;
    int target_count = 0, log_count = 0;
    
    while (current_time < sim_time) {
        // Physics update
        if (t_physics >= DT_PHYSICS) {
            double new_linear_position_W[3], new_linear_velocity_W[3];
            double new_angular_velocity_B[3], new_R_W_B[9];
            double accel_measurement[3], gyro_measurement[3];
            double new_accel_bias[3], new_gyro_bias[3], new_omega[4];
            
            update_quad_states(
                quad.omega, quad.linear_position_W, quad.linear_velocity_W,
                quad.angular_velocity_B, quad.R_W_B, quad.inertia,
                quad.accel_bias, quad.gyro_bias, quad.accel_scale, quad.gyro_scale,
                quad.omega_next, DT_PHYSICS,
                (double)rand()/RAND_MAX, (double)rand()/RAND_MAX,
                (double)rand()/RAND_MAX, (double)rand()/RAND_MAX,
                new_linear_position_W, new_linear_velocity_W, new_angular_velocity_B,
                new_R_W_B, accel_measurement, gyro_measurement,
                new_accel_bias, new_gyro_bias, new_omega
            );
            
            memcpy(quad.linear_position_W, new_linear_position_W, 3 * sizeof(double));
            memcpy(quad.linear_velocity_W, new_linear_velocity_W, 3 * sizeof(double));
            memcpy(quad.angular_velocity_B, new_angular_velocity_B, 3 * sizeof(double));
            memcpy(quad.R_W_B, new_R_W_B, 9 * sizeof(double));
            memcpy(quad.accel_measurement, accel_measurement, 3 * sizeof(double));
            memcpy(quad.gyro_measurement, gyro_measurement, 3 * sizeof(double));
            memcpy(quad.accel_bias, new_accel_bias, 3 * sizeof(double));
            memcpy(quad.gyro_bias, new_gyro_bias, 3 * sizeof(double));
            memcpy(quad.omega, new_omega, 4 * sizeof(double));
            
            t_physics = 0.0;
        }
        
        // Control update
        if (t_control >= DT_CONTROL) {
            // Update estimator (which estimates orientation)
            update_estimator(
                quad.gyro_measurement, quad.accel_measurement,
                DT_CONTROL, &estimator
            );
            
            // Simple position estimator (integrate velocity)
            // In reality you'd use a proper filter like EKF
            for (int i = 0; i < 3; i++) {
                estimated_vel[i] += quad.accel_measurement[i] * DT_CONTROL;
                estimated_pos[i] += estimated_vel[i] * DT_CONTROL;
            }
            
            // Run controller with absolute targets (for now)
            double new_omega_cmd[4];
            control_quad_commands(
                quad.linear_position_W, quad.linear_velocity_W,
                estimator.R, estimator.angular_velocity,
                quad.inertia, target_world, new_omega_cmd
            );
            memcpy(quad.omega_next, new_omega_cmd, 4 * sizeof(double));
            
            // Check if target reached
            if (is_near_target(quad.linear_position_W, target_world, 0.3)) {
                generate_random_target(target_world);
                target_count++;
            }
            
            t_control = 0.0;
        }
        
        // Log data
        if (t_log >= DT_LOG) {
            // Transform target to body frame for logging
            double target_body[7];
            world_target_to_body_frame(target_world, estimated_pos, 
                                      estimator.R, target_body);
            
            fprintf(csv, "%.6f,", current_time);
            fprintf(csv, "%.6f,%.6f,%.6f,", 
                    quad.accel_measurement[0], quad.accel_measurement[1], 
                    quad.accel_measurement[2]);
            fprintf(csv, "%.6f,%.6f,%.6f,", 
                    quad.gyro_measurement[0], quad.gyro_measurement[1], 
                    quad.gyro_measurement[2]);
            fprintf(csv, "%.6f,%.6f,%.6f,%.6f,", 
                    quad.omega[0], quad.omega[1], quad.omega[2], quad.omega[3]);
            fprintf(csv, "%.6f,%.6f,%.6f,%.6f,", 
                    quad.omega_next[0], quad.omega_next[1], 
                    quad.omega_next[2], quad.omega_next[3]);
            fprintf(csv, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", 
                    target_body[0], target_body[1], target_body[2],
                    target_body[3], target_body[4], target_body[5], target_body[6]);
            log_count++;
            
            if (log_count % 600 == 0) {
                float progress = current_time / sim_time * 100.0f;
                printf("\rProgress: %.1f%% | Targets: %d | Samples: %d    ", 
                       progress, target_count, log_count);
                fflush(stdout);
            }
            
            t_log = 0.0;
        }
        
        t_physics += DT_PHYSICS;
        t_control += DT_PHYSICS;
        t_log += DT_PHYSICS;
        current_time += DT_PHYSICS;
    }
    
    fclose(csv);
    printf("\n\nData generation complete!\n");
    printf("Total samples: %d\n", log_count);
    printf("Targets reached: %d\n", target_count);
    printf("Data saved to: %s\n", output_file);
}

#ifndef NO_RENDER
void run_visualization(double sim_time) {
    // Initialize quadcopter
    double drone_x = random_range(-2.0, 2.0);
    double drone_y = random_range(0.5, 2.0);
    double drone_z = random_range(-2.0, 2.0);
    double drone_yaw = random_range(-M_PI, M_PI);
    
    double target_x = random_range(-2.0, 2.0);
    double target_y = random_range(0.5, 2.5);
    double target_z = random_range(-2.0, 2.0);
    double target_yaw = random_range(-M_PI, M_PI);
    
    double target[7] = {
        target_x, target_y, target_z,
        0.0, 0.0, 0.0,
        target_yaw
    };
    
    printf("Drone starts at (%.2f, %.2f, %.2f), target at (%.2f, %.2f, %.2f)\n", 
           drone_x, drone_y, drone_z, target_x, target_y, target_z);
    
    Quad quad = create_quad(drone_x, drone_y, drone_z, drone_yaw);
    
    StateEstimator estimator = {
        .angular_velocity = {0.0, 0.0, 0.0},
        .gyro_bias = {0.0, 0.0, 0.0}
    };
    memcpy(estimator.R, quad.R_W_B, 9 * sizeof(double));
    
    // Initialize scene
    Scene scene = create_scene(400, 300, (int)(sim_time * 1000), 24, 0.4f);
    
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
    
    Mesh drone_mesh = create_mesh("raytracer/assets/drone.obj", 
                                  "raytracer/assets/drone.webp");
    add_mesh_to_scene(&scene, drone_mesh);
    
    Mesh treasure = create_mesh("raytracer/assets/treasure.obj", 
                               "raytracer/assets/treasure.webp");
    add_mesh_to_scene(&scene, treasure);
    set_mesh_position(&scene.meshes[1], 
                     (Vec3){(float)target_x, (float)target_y, (float)target_z});
    
    Mesh ground = create_mesh("raytracer/assets/ground.obj", 
                             "raytracer/assets/ground.webp");
    add_mesh_to_scene(&scene, ground);

    double t_physics = 0.0, t_control = 0.0, t_render = 0.0;
    clock_t start_time = clock();

    for (int t = 0; t < (int)(sim_time / DT_PHYSICS); t++) {
        if (t_physics >= DT_PHYSICS) {
            double new_linear_position_W[3], new_linear_velocity_W[3];
            double new_angular_velocity_B[3], new_R_W_B[9];
            double accel_measurement[3], gyro_measurement[3];
            double new_accel_bias[3], new_gyro_bias[3], new_omega[4];
            
            update_quad_states(
                quad.omega, quad.linear_position_W, quad.linear_velocity_W,
                quad.angular_velocity_B, quad.R_W_B, quad.inertia,
                quad.accel_bias, quad.gyro_bias, quad.accel_scale, quad.gyro_scale,
                quad.omega_next, DT_PHYSICS,
                (double)rand()/RAND_MAX, (double)rand()/RAND_MAX,
                (double)rand()/RAND_MAX, (double)rand()/RAND_MAX,
                new_linear_position_W, new_linear_velocity_W, new_angular_velocity_B,
                new_R_W_B, accel_measurement, gyro_measurement,
                new_accel_bias, new_gyro_bias, new_omega
            );
            
            memcpy(quad.linear_position_W, new_linear_position_W, 3 * sizeof(double));
            memcpy(quad.linear_velocity_W, new_linear_velocity_W, 3 * sizeof(double));
            memcpy(quad.angular_velocity_B, new_angular_velocity_B, 3 * sizeof(double));
            memcpy(quad.R_W_B, new_R_W_B, 9 * sizeof(double));
            memcpy(quad.accel_measurement, accel_measurement, 3 * sizeof(double));
            memcpy(quad.gyro_measurement, gyro_measurement, 3 * sizeof(double));
            memcpy(quad.accel_bias, new_accel_bias, 3 * sizeof(double));
            memcpy(quad.gyro_bias, new_gyro_bias, 3 * sizeof(double));
            memcpy(quad.omega, new_omega, 4 * sizeof(double));
            
            t_physics = 0.0;
        }
        
        if (t_control >= DT_CONTROL) {
            update_estimator(quad.gyro_measurement, quad.accel_measurement,
                           DT_CONTROL, &estimator);
            
            double new_omega[4];
            control_quad_commands(
                quad.linear_position_W, quad.linear_velocity_W,
                estimator.R, estimator.angular_velocity,
                quad.inertia, target, new_omega
            );
            memcpy(quad.omega_next, new_omega, 4 * sizeof(double));
            t_control = 0.0;
        }
        
        if (t_render >= DT_RENDER) {
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
            update_progress_bar((int)(t * DT_PHYSICS / DT_RENDER), 
                              (int)(sim_time * 24), start_time);
            t_render = 0.0;
        }
        
        t_physics += DT_PHYSICS;
        t_control += DT_PHYSICS;
        t_render += DT_PHYSICS;
    }

    printf("\nFinal position: (%.2f, %.2f, %.2f)\n", 
           quad.linear_position_W[0], quad.linear_position_W[1], 
           quad.linear_position_W[2]);

    char filename[64];
    time_t current_time = time(NULL);
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_flight.webp", 
            localtime(&current_time));
    save_scene(&scene, filename);
    printf("Animation saved to: %s\n", filename);

    destroy_mesh(&drone_mesh);
    destroy_mesh(&treasure);
    destroy_mesh(&ground);
    destroy_scene(&scene);
}
#endif

int main(int argc, char** argv) {
    srand(time(NULL));
    
    if (argc > 1 && strcmp(argv[1], "--data") == 0) {
        double sim_time = 1000.0;  // Default: 1000 seconds
        if (argc > 2) sim_time = atof(argv[2]);
        
        char filename[64];
        time_t current_time = time(NULL);
        strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_data.csv", 
                localtime(&current_time));
        
        run_data_generation(sim_time, filename);
    } else {
#ifndef NO_RENDER
        run_visualization(10.0);  // 10 second visualization
#else
        printf("Rendering not compiled. Use --data flag for data generation.\n");
        printf("Or recompile without -DNO_RENDER flag.\n");
        return 1;
#endif
    }
    
    return 0;
}