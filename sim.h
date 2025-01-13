#ifndef SIM_H
#define SIM_H

#include "rasterizer.h"
#include "gif.h"
#include "quad.h"

typedef struct {
    Mesh** meshes;
    Quad* quad;
    uint8_t *frame_buffer;
    ge_GIF *gif;
} Sim;

Sim* init_sim(){
    Sim* sim = malloc(sizeof(Sim));
    sim->meshes = malloc(2 * sizeof(Mesh*));
    sim->meshes[0] = create_mesh("sim/rasterizer/drone.obj", "sim/rasterizer/drone.bmp");
    sim->meshes[1] = create_mesh("sim/rasterizer/ground.obj", "sim/rasterizer/ground.bmp");
    sim->quad = init_quad(0.0, 0.0, 0.0);
    sim->frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));

    time_t t = time(NULL);
    struct tm tm = *localtime(&t);
    char filename[100];
    sprintf(filename, "%d-%d-%d_%d-%d-%d_flight.gif", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    sim->gif = ge_new_gif(filename, WIDTH, HEIGHT, 4, -1, 0);

    transform_mesh(sim->meshes[1], (double[3]){0.0, -0.2, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});
    return sim;
}

void render_sim(Sim* sim){
    transform_mesh(sim->meshes[0], sim->quad->linear_position_W, 0.5, sim->quad->R_W_B);
    memset(sim->frame_buffer, 0, WIDTH * HEIGHT * 3);
    vertex_shader(sim->meshes, 2, (double[3]){-2.0, 2.0, -2.0}, (double[3]){0.0, 0.0, 0.0});
    rasterize(sim->frame_buffer, sim->meshes, 2);
    ge_add_frame(sim->gif, sim->frame_buffer, 6);
}

void free_sim(Sim* sim){
    free_meshes(sim->meshes, 2);
    free(sim->frame_buffer);
    ge_close_gif(sim->gif);
    free(sim->gif);
    free(sim->quad);
    free(sim);
}

#endif // SIM_H