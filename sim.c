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

static bool is_stable(void) {
    for (int i = 0; i < 3; i++)
        if (fabs(angular_velocity_B[i]) > 0.005) return false;
    return true;
}

static bool is_at_target_position(void) {
    for (int i = 0; i < 3; i++)
        if (fabs(linear_position_W[i] - linear_position_d_W[i]) > 0.1) return false;
    return true;
}

static bool check_divergence(void) {
    for (int i = 0; i < 3; i++) {
        if (fabs(linear_position_W[i]) > 1000.0 || 
            fabs(linear_velocity_W[i]) > 100.0 || 
            fabs(angular_velocity_B[i]) > 100.0) return true;
    }
    for (int i = 0; i < 4; i++)
        if (omega_next[i] < 0 || omega_next[i] > 1000) return true;
    return false;
}

int main(int argc, char *argv[]) {
    #ifdef RENDER
    Mesh* meshes[] = {
        create_mesh("rasterizer/drone.obj", "rasterizer/drone.bmp"),
        create_mesh("rasterizer/ground.obj", "rasterizer/ground.bmp")
    };
    uint8_t *frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    ge_GIF *gif = ge_new_gif("drone_simulation.gif", WIDTH, HEIGHT, 4, -1, 0);
    transform_mesh(meshes[1], (double[3]){0.0, -0.5, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});
    double t_render = 0.0;
    #endif

    #ifdef LOG
    int max_steps = 5000;
    if (argc > 1) max_steps = strtol(argv[1], NULL, 10);
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char filename[100];
    sprintf(filename, "%d-%d-%d_%d-%d-%d_control_data.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE *csv_file = fopen(filename, "w");
    fprintf(csv_file, "pos_d[0],pos_d[1],pos_d[2],yaw_d,ang_vel[0],ang_vel[1],ang_vel[2],acc[0],acc[1],acc[2],omega[0],omega[1],omega[2],omega[3]\n");
    srand(time(NULL));
    #endif

    double t_physics = 0.0, t_control = 0.0, t_simulation = 0.0;

    for (int meta_step = 0; meta_step < max_steps; meta_step++) {
        #ifdef LOG
        for (int i = 0; i < 3; i++) linear_position_d_W[i] = (double)rand() / RAND_MAX * 10 - (i != 1 ? 5 : 0);
        yaw_d = (double)rand() / RAND_MAX * 2 * M_PI;
        if(meta_step % 100 == 0) printf("New target %d: [%.3f, %.3f, %.3f], yaw: %.3f\n", meta_step, linear_position_d_W[0], linear_position_d_W[1], linear_position_d_W[2], yaw_d);
        #endif

        while (!is_stable() || !is_at_target_position()) {
            if (check_divergence()) {
                #ifdef LOG
                fclose(csv_file);
                remove(filename);
                #endif
                printf("Simulation diverged.\n");
                return 1;
            }

            while (t_physics <= t_simulation) {
                update_drone_physics(DT_PHYSICS);
                t_physics += DT_PHYSICS;
            }
            
            if (t_control <= t_simulation) {
                update_drone_control();
                #ifdef LOG
                fprintf(csv_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", linear_position_d_W[0], linear_position_d_W[1], linear_position_d_W[2], yaw_d, angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2], linear_acceleration_B[0], linear_acceleration_B[1], linear_acceleration_B[2], omega_next[0], omega_next[1], omega_next[2], omega_next[3]);
                #endif
                update_rotor_speeds();
                t_control += DT_CONTROL;
            }

            #ifdef RENDER
            if (t_render <= t_simulation) {
                transform_mesh(meshes[0], linear_position_W, 0.5, R_W_B);
                memset(frame_buffer, 0, WIDTH * HEIGHT * 3);
                vertex_shader(meshes, 2, (double[3]){-2.0, 2.0, -2.0}, (double[3]){0.0, 0.0, 0.0});
                rasterize(frame_buffer, meshes, 2);
                ge_add_frame(gif, frame_buffer, 6);
                t_render += DT_RENDER;
            }
            #endif

            #ifndef LOG
            printf("Position: [%.3f, %.3f, %.3f]\nDesired position: [%.3f, %.3f, %.3f]\nAngular Velocity: [%.3f, %.3f, %.3f]\n---\n", linear_position_W[0], linear_position_W[1], linear_position_W[2], linear_position_d_W[0], linear_position_d_W[1], linear_position_d_W[2], angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2]);
            #endif

            t_simulation += DT_PHYSICS;
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