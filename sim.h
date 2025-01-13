#ifndef SIM_H
#define SIM_H

#include "rasterizer.h"
#include "gif.h"
#include "quad.h"

typedef struct {
    Mesh** meshes;
    uint8_t *frame_buffer;
    ge_GIF *gif;
    FILE *csv_file;
} Sim;

void init_sensors(){
    for(int i = 0; i < 3; i++) {
        accel_bias[i] = (2.0*((double)rand()/RAND_MAX) - 1.0) * ACCEL_BIAS;
        gyro_bias[i] = (2.0*((double)rand()/RAND_MAX) - 1.0) * GYRO_BIAS;
    }
}

Sim* init_sim_render(){
    Sim* sim = malloc(sizeof(Sim));
    sim->meshes = malloc(2 * sizeof(Mesh*));
    sim->meshes[0] = create_mesh("rasterizer/drone.obj", "rasterizer/drone.bmp");
    sim->meshes[1] = create_mesh("rasterizer/ground.obj", "rasterizer/ground.bmp");
    sim->frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char filename[100];
    sprintf(filename, "%d-%d-%d_%d-%d-%d_flight.gif", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    sim->gif = ge_new_gif(filename, WIDTH, HEIGHT, 4, -1, 0);
    transform_mesh(sim->meshes[1], (double[3]){0.0, -0.2, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});
    init_sensors();
    return sim;
}

void render_sim(Sim* sim){
    transform_mesh(sim->meshes[0], linear_position_W, 0.5, R_W_B);
    memset(sim->frame_buffer, 0, WIDTH * HEIGHT * 3);
    vertex_shader(sim->meshes, 2, (double[3]){-2.0, 2.0, -2.0}, (double[3]){0.0, 0.0, 0.0});
    rasterize(sim->frame_buffer, sim->meshes, 2);
    ge_add_frame(sim->gif, sim->frame_buffer, 6);
}

void free_sim_render(Sim* sim){
    free_meshes(sim->meshes, 2);
    free(sim->frame_buffer);
    ge_close_gif(sim->gif);
    free(sim);
}

Sim* init_sim_log(){
    Sim* sim = malloc(sizeof(Sim));
    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char filename[100];
    sprintf(filename, "%d-%d-%d_%d-%d-%d_trajectory.csv", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    sim->csv_file = fopen(filename, "w");
    fprintf(sim->csv_file, "pos[0],pos[1],pos[2],vel[0],vel[1],vel[2],ang_vel[0],ang_vel[1],ang_vel[2],acc_s[0],acc_s[1],acc_s[2],gyro_s[0],gyro_s[1],gyro_s[2],mean[0],mean[1],mean[2],mean[3],var[0],var[1],var[2],var[3],omega[0],omega[1],omega[2],omega[3],reward,discounted_return\n");
    init_sensors();
    return sim;
}

#endif // SIM_H