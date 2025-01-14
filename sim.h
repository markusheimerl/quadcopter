#ifndef SIM_H
#define SIM_H

#include <stdbool.h>
#include <time.h>
#include "rasterizer.h"
#include "gif.h"
#include "quad.h"

typedef struct {
    Mesh** meshes;
    Quad* quad;
    uint8_t *frame_buffer;
    ge_GIF *gif;
    bool render;
} Sim;

Sim* init_sim(bool render){
    Sim* sim = malloc(sizeof(Sim));
    if(render){
        sim->meshes = malloc(2 * sizeof(Mesh*));
        sim->meshes[0] = create_mesh("rasterizer/drone.obj", "rasterizer/drone.bmp");
        sim->meshes[1] = create_mesh("rasterizer/ground.obj", "rasterizer/ground.bmp");
        sim->frame_buffer = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
        char filename[100];
        strftime(filename, 100, "%Y-%m-%d_%H-%M-%S_flight.gif", localtime(&(time_t){time(NULL)}));
        sim->gif = ge_new_gif(filename, WIDTH, HEIGHT, 4, -1, 0);
        transform_mesh(sim->meshes[1], (double[3]){0.0, -0.2, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});
    }
    sim->render = render;
    sim->quad = init_quad(0.0, 0.0, 0.0);
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
    if(sim->render){
        free_meshes(sim->meshes, 2);
        free(sim->frame_buffer);
        ge_close_gif(sim->gif);
    }
    free(sim->quad);
    free(sim);
}

#endif // SIM_H