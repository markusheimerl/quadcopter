#ifdef RENDER
#include "gif.h"
#include "rasterizer.h"
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include "quad.h"

static bool is_stable(const double v[3]) {
    for (int i = 0; i < 3; i++) {if (fabs(v[i]) > 0.005) return false;}
    return true;
}

static bool is_at_target_position(void) {
    double position_error[3] = {linear_position_W[0] - linear_position_d_W[0], linear_position_W[1] - linear_position_d_W[1], linear_position_W[2] - linear_position_d_W[2]};
    for (int i = 0; i < 3; i++) {if (fabs(position_error[i]) > 0.1) return false;}
    return true;
}

static bool check_divergence(void) {
    if (fabs(linear_position_W[0]) > 1000.0 || fabs(linear_position_W[1]) > 1000.0 || fabs(linear_position_W[2]) > 1000.0 ||fabs(linear_velocity_W[0]) > 100.0 || fabs(linear_velocity_W[1]) > 100.0 || fabs(linear_velocity_W[2]) > 100.0 ||fabs(angular_velocity_B[0]) > 100.0 || fabs(angular_velocity_B[1]) > 100.0 || fabs(angular_velocity_B[2]) > 100.0) return true;
    for (int i = 0; i < 4; i++) {if (omega_next[i] < 0 || omega_next[i] > 1000) return true;}
    return false;
}

int main() {
    #ifdef RENDER
    Mesh* meshes[] = {
        create_mesh("rasterizer/drone.obj", "rasterizer/drone.bmp"),
        create_mesh("rasterizer/ground.obj", "rasterizer/ground.bmp")
    };
    uint8_t *frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    ge_GIF *gif = ge_new_gif("drone_simulation.gif", WIDTH, HEIGHT, 4, -1, 0);
    transform_mesh(meshes[1], (double[3]){0.0, -0.5, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});
    #endif

    #ifdef LOG
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char filename[100];
    sprintf(filename, "%d-%d-%d_%d-%d-%d_control_data.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE *csv_file = fopen(filename, "w");
    fprintf(csv_file, "meta_step,step,pos_d[0],pos_d[1],pos_d[2],yaw_d,ang_vel[0],ang_vel[1],ang_vel[2],acc[0],acc[1],acc[2],omega[0],omega[1],omega[2],omega[3]\n");
    srand(time(NULL));
    #endif

    for (int meta_step = 0; meta_step < 4; meta_step++) {
        #ifdef LOG
        linear_position_d_W[0] = (double)rand() / RAND_MAX * 10 - 5;
        linear_position_d_W[1] = (double)rand() / RAND_MAX * 10;
        linear_position_d_W[2] = (double)rand() / RAND_MAX * 10 - 5;
        yaw_d = (double)rand() / RAND_MAX * 2 * M_PI;
        printf("New target %d: [%.3f, %.3f, %.3f], yaw: %.3f\n", meta_step, linear_position_d_W[0], linear_position_d_W[1], linear_position_d_W[2], yaw_d);
        #endif

        while (!is_stable(angular_velocity_B) || !is_at_target_position()) {
            if (check_divergence()) {
                #ifdef LOG
                fclose(csv_file);
                remove(filename);
                printf("Simulation diverged at meta_step %d\n", meta_step);
                #endif
                return 1;
            }

            update_drone_physics();
            update_drone_control();
            
            #ifdef LOG
            fprintf(csv_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", linear_position_d_W[0], linear_position_d_W[1], linear_position_d_W[2], yaw_d, angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2], linear_acceleration_B[0], linear_acceleration_B[1], linear_acceleration_B[2], omega_next[0], omega_next[1], omega_next[2], omega_next[3]);
            #endif

            update_rotor_speeds();

            #ifdef RENDER
            transform_mesh(meshes[0], linear_position_W, 0.5, R_W_B);
            memset(frame_buffer, 0, WIDTH * HEIGHT * 3);
            vertex_shader(meshes, 2, (double[3]){-2.0, 2.0, -2.0}, (double[3]){0.0, 0.0, 0.0});
            rasterize(frame_buffer, meshes, 2);
            ge_add_frame(gif, frame_buffer, 6);
            #endif

            #ifdef LOG
            printf("Meta step %d\n", meta_step);
            #endif
            printf("Position: [%.3f, %.3f, %.3f]\n", linear_position_W[0], linear_position_W[1], linear_position_W[2]);
            printf("Desired position: [%.3f, %.3f, %.3f]\n", linear_position_d_W[0], linear_position_d_W[1], linear_position_d_W[2]);
            printf("Angular Velocity: [%.3f, %.3f, %.3f]\n", angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2]);
            printf("---\n");
        }

        #ifndef LOG
        break;
        #endif
    }

    #ifdef LOG
    fclose(csv_file);
    #endif

    #ifdef RENDER
    free(frame_buffer);
    free_meshes(meshes, 2);
    ge_close_gif(gif);
    #endif
    
    return 0;
}