#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#ifdef RENDER
#include "gif.h"
#include "rasterizer.h"
#endif
#include "quad.h"

#define DT_PHYSICS  (1.0 / 1000.0)
#define DT_CONTROL  (1.0 / 60.0)
#define DT_RENDER   (1.0 / 30.0)

int main(int argc, char *argv[]) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char filename[100];
    
    #ifdef RENDER
    sprintf(filename, "%d-%d-%d_%d-%d-%d_simulation.gif", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    Mesh* meshes[] = {create_mesh("rasterizer/drone.obj", "rasterizer/drone.bmp"), create_mesh("rasterizer/ground.obj", "rasterizer/ground.bmp")};
    uint8_t *frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    ge_GIF *gif = ge_new_gif(filename, WIDTH, HEIGHT, 4, -1, 0);
    transform_mesh(meshes[1], (double[3]){0.0, -0.5, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});
    double t_render = 0.0;
    double t_status = 0.0;
    int max_steps = 2;
    #else
    int max_steps = 100;
    #endif

    if (argc > 1) max_steps = strtol(argv[1], NULL, 10);
    
    #ifdef LOG
    sprintf(filename, "%d-%d-%d_%d-%d-%d_control_data.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE *csv_file = fopen(filename, "w");
    fprintf(csv_file, "vel_d_B[0],vel_d_B[1],vel_d_B[2],yaw_d,ang_vel[0],ang_vel[1],ang_vel[2],acc[0],acc[1],acc[2],omega[0],omega[1],omega[2],omega[3]\n");
    #endif

    srand(time(NULL));
    double t_physics = 0.0, t_control = 0.0;

    for (int meta_step = 0; meta_step < max_steps; meta_step++) {
        for (int i = 0; i < 3; i++) linear_velocity_d_B[i] = (double)rand() / RAND_MAX * 1.0 - 0.5;
        linear_velocity_d_B[1] *= 0.2;
        yaw_d = (double)rand() / RAND_MAX * 2 * M_PI;
        #ifdef RENDER
        printf("\n=== New Target %d ===\nDesired velocity (body): [%.3f, %.3f, %.3f], yaw: %.3f\n", 
               meta_step, linear_velocity_d_B[0], linear_velocity_d_B[1], linear_velocity_d_B[2], yaw_d);
        #endif

        double min_time = t_physics + 0.5;
        bool velocity_achieved = false;

        while (!velocity_achieved || t_physics < min_time) {
            if (fabs(linear_position_W[0]) > 1000.0 || fabs(linear_position_W[1]) > 1000.0 || 
                fabs(linear_position_W[2]) > 1000.0 || fabs(linear_velocity_W[0]) > 100.0 || 
                fabs(linear_velocity_W[1]) > 100.0 || fabs(linear_velocity_W[2]) > 100.0 || 
                fabs(angular_velocity_B[0]) > 100.0 || fabs(angular_velocity_B[1]) > 100.0 || 
                fabs(angular_velocity_B[2]) > 100.0) {
                printf("Simulation diverged.\n");
                #ifdef LOG
                fclose(csv_file);
                remove(filename);
                #endif
                return 1;
            }

            update_drone_physics(DT_PHYSICS);
            t_physics += DT_PHYSICS;
            
            if (t_control <= t_physics) {
                update_drone_control();
                #ifdef LOG
                fprintf(csv_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", linear_velocity_d_B[0], linear_velocity_d_B[1], linear_velocity_d_B[2], yaw_d, angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2], linear_acceleration_B[0], linear_acceleration_B[1], linear_acceleration_B[2], omega_next[0], omega_next[1], omega_next[2], omega_next[3]);
                #endif
                update_rotor_speeds();
                t_control += DT_CONTROL;

                velocity_achieved = true;
                for (int i = 0; i < 3; i++) {
                    if (fabs(angular_velocity_B[i]) > 0.005 || fabs(linear_velocity_B[i] - linear_velocity_d_B[i]) > 0.1) {
                        velocity_achieved = false;
                        break;
                    }
                }

                #ifdef RENDER
                if (t_physics >= t_status) {
                    // Calculate current yaw from rotation matrix
                    double current_yaw = atan2(R_W_B[2], R_W_B[8]);
                    printf("\rPos: [%5.2f, %5.2f, %5.2f] Vel (body): [%5.2f, %5.2f, %5.2f] Angular: [%5.2f, %5.2f, %5.2f] Yaw: %5.2f/%5.2f", 
                           linear_position_W[0], linear_position_W[1], linear_position_W[2],
                           linear_velocity_B[0], linear_velocity_B[1], linear_velocity_B[2],
                           angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2],
                           current_yaw, yaw_d);
                    fflush(stdout);
                    t_status = t_physics + 0.1;
                }
                #endif
            }

            #ifdef RENDER
            if (t_render <= t_physics) {
                transform_mesh(meshes[0], linear_position_W, 0.5, R_W_B);
                memset(frame_buffer, 0, WIDTH * HEIGHT * 3);
                vertex_shader(meshes, 2, (double[3]){-2.0, 2.0, -2.0}, (double[3]){0.0, 0.0, 0.0});
                rasterize(frame_buffer, meshes, 2);
                ge_add_frame(gif, frame_buffer, 6);
                t_render += DT_RENDER;
            }
            #endif
        }
        #ifdef RENDER
        printf("\nTarget achieved!\n");
        #endif
    }

    #ifdef LOG
    fclose(csv_file);
    #endif
    #ifdef RENDER
    free(frame_buffer); free_meshes(meshes, 2); ge_close_gif(gif);
    #endif
    
    return 0;
}