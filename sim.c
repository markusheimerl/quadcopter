#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#define VEC3_MAG2(v) ((v)[0]*(v)[0] + (v)[1]*(v)[1] + (v)[2]*(v)[2])
#define WAIT_TIME 1.0
#define GAMMA 0.99  // Discount factor for rewards

int main(int argc, char *argv[]) {
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char filename[100];
    
    #ifdef RENDER
    sprintf(filename, "%d-%d-%d_%d-%d-%d_simulation.gif", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    Mesh* meshes[] = {create_mesh("rasterizer/drone.obj", "rasterizer/drone.bmp"), create_mesh("rasterizer/ground.obj", "rasterizer/ground.bmp")};
    uint8_t *frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    ge_GIF *gif = ge_new_gif(filename, WIDTH, HEIGHT, 4, -1, 0);
    transform_mesh(meshes[1], (double[3]){0.0, -0.2, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});
    double t_render = 0.0, t_status = 0.0;
    int max_steps = 4;
    #else
    int max_steps = 1000;
    #endif

    if (argc > 1) max_steps = strtol(argv[1], NULL, 10);
    
    #ifdef LOG
    sprintf(filename, "%d-%d-%d_%d-%d-%d_state_data.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    FILE *csv_file = fopen(filename, "w");
    fprintf(csv_file, "pos[0],pos[1],pos[2],vel[0],vel[1],vel[2],ang_vel[0],ang_vel[1],ang_vel[2],R[0],R[1],R[2],R[3],R[4],R[5],R[6],R[7],R[8],omega[0],omega[1],omega[2],omega[3],reward,discounted_return\n");
    #endif

    srand(time(NULL));
    for(int i = 0; i < 3; i++) {
        accel_bias[i] = (2.0 * ((double)rand() / RAND_MAX) - 1.0) * ACCEL_BIAS;
        gyro_bias[i] = (2.0 * ((double)rand() / RAND_MAX) - 1.0) * GYRO_BIAS;
    }

    double t_physics = 0.0, t_control = 0.0, wait_start = 0.0;
    bool is_waiting = true, at_ground = true;

    for (int meta_step = 0; meta_step < max_steps; meta_step++) {
        if (is_waiting && at_ground) {
            for(int i = 0; i < 3; i++) linear_position_d_W[i] = linear_position_W[i];
            wait_start = t_physics;
            #ifdef RENDER
            printf("\n=== Waiting at ground ===\n");
            #endif
        } else {
            linear_position_d_W[0] = linear_position_d_W[2] = yaw_d = 0.0;
            linear_position_d_W[1] = at_ground ? 1.0 : 0.0;
            #ifdef RENDER
            printf("\n=== Moving to [0.000, %.3f, 0.000] ===\n", linear_position_d_W[1]);
            #endif
        }

        bool position_achieved = false, stability_achieved = false;
        double min_time = t_physics + 0.5;

        while (!position_achieved || !stability_achieved || t_physics < min_time) {
            if (VEC3_MAG2(linear_position_W) > 100.0*100.0 || VEC3_MAG2(linear_velocity_W) > 10.0*10.0 || VEC3_MAG2(angular_velocity_B) > 10.0*10.0) {
                printf("\nSimulation diverged.\n");
                #ifdef LOG
                fclose(csv_file); remove(filename);
                #endif
                return 1;
            }

            update_drone_physics(DT_PHYSICS);
            t_physics += DT_PHYSICS;
            
            if (t_control <= t_physics) {
                update_drone_control();

                #ifdef LOG
                double pos_error = sqrt(pow(linear_position_W[0], 2) + pow(linear_position_W[1] - 1.0, 2) + pow(linear_position_W[2], 2));
                double reward = exp(-(pos_error * 2.0 + sqrt(VEC3_MAG2(angular_velocity_B)) * 0.5));
                fprintf(csv_file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", linear_position_W[0], linear_position_W[1], linear_position_W[2], linear_velocity_W[0], linear_velocity_W[1], linear_velocity_W[2], angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2], R_W_B[0], R_W_B[1], R_W_B[2], R_W_B[3], R_W_B[4], R_W_B[5], R_W_B[6], R_W_B[7], R_W_B[8], omega[0], omega[1], omega[2], omega[3], reward, 0.0);
                #endif
                
                update_rotor_speeds();
                t_control += DT_CONTROL;

                position_achieved = stability_achieved = true;
                for (int i = 0; i < 3; i++) {
                    if (fabs(linear_position_W[i] - linear_position_d_W[i]) > 0.05) position_achieved = false;
                    if (fabs(angular_velocity_B[i]) > 0.05) stability_achieved = false;
                }
                if (is_waiting && (t_physics - wait_start >= WAIT_TIME)) position_achieved = stability_achieved = true;

                #ifdef RENDER
                if (t_physics >= t_status) {
                    printf("\rP: [% 6.3f, % 6.3f, % 6.3f] yaw: % 6.3f A_V_B: [% 6.3f, % 6.3f, % 6.3f] R: [% 6.3f, % 6.3f, % 6.3f, % 6.3f]", linear_position_W[0], linear_position_W[1], linear_position_W[2], atan2(R_W_B[2], R_W_B[8]) < 0 ? atan2(R_W_B[2], R_W_B[8]) + 2*M_PI : atan2(R_W_B[2], R_W_B[8]), angular_velocity_B[0], angular_velocity_B[1], angular_velocity_B[2], omega[0], omega[1], omega[2], omega[3]); 
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
        
        if (is_waiting) is_waiting = false;
        else { at_ground = !at_ground; is_waiting = at_ground; }
    }

    #ifdef LOG
    fclose(csv_file);
    FILE *input = fopen(filename, "r");
    char temp_filename[100];
    sprintf(temp_filename, "%s.tmp", filename);
    FILE *output = fopen(temp_filename, "w");
    
    char line[1024], header[1024];
    fgets(header, sizeof(header), input);
    fprintf(output, "%s", header);

    int line_count = 0;
    double *rewards = NULL;
    while (fgets(line, sizeof(line), input)) {
        rewards = realloc(rewards, (line_count + 1) * sizeof(double));
        char *token = strtok(line, ",");
        for (int i = 0; i < 22; i++) token = strtok(NULL, ",");
        rewards[line_count++] = atof(token);
    }

    rewind(input);
    fgets(line, sizeof(line), input);

    int current_line = 0;
    while (fgets(line, sizeof(line), input)) {
        double discounted_return = 0.0;
        double discount = 1.0;
        for (int i = current_line; i < line_count; i++) {
            discounted_return += discount * rewards[i];
            discount *= GAMMA;
        }
        
        line[strlen(line)-1] = '\0';
        char *last_comma = strrchr(line, ',');
        *last_comma = '\0';
        fprintf(output, "%s,%f\n", line, discounted_return);
        current_line++;
    }

    free(rewards); fclose(input); fclose(output); remove(filename); rename(temp_filename, filename);
    #endif

    #ifdef RENDER
    free(frame_buffer); free_meshes(meshes, 2); ge_close_gif(gif);
    #endif
    return 0;
}