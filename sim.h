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
    uint8_t **frame_buffers;
    int size_frame_buffers;
    ge_GIF *gif;
    bool render;
} Sim;

Sim* init_sim(const char* prefix, bool render){
    Sim* sim = malloc(sizeof(Sim));
    if(render){
        char path[256];
        sim->meshes = malloc(2 * sizeof(Mesh*));
        
        snprintf(path, sizeof(path), "%srasterizer/drone.obj", prefix);
        char tex_path[256];
        snprintf(tex_path, sizeof(tex_path), "%srasterizer/drone.bmp", prefix);
        sim->meshes[0] = create_mesh(path, tex_path);
            
        snprintf(path, sizeof(path), "%srasterizer/ground.obj", prefix);
        snprintf(tex_path, sizeof(tex_path), "%srasterizer/ground.bmp", prefix);
        sim->meshes[1] = create_mesh(path, tex_path);
            
        sim->frame_buffers = NULL;
        sim->size_frame_buffers = 0;
        transform_mesh(sim->meshes[1], (double[3]){0.0, -0.2, 0.0}, 1.0, (double[9]){1,0,0, 0,1,0, 0,0,1});
    }
    sim->render = render;
    sim->quad = init_quad(0.0, 0.0, 0.0);
    return sim;
}

void render_sim(Sim* sim){
    transform_mesh(sim->meshes[0], sim->quad->linear_position_W, 0.5, sim->quad->R_W_B);
    vertex_shader(sim->meshes, 2, (double[3]){-2.0, 2.0, -2.0}, (double[3]){0.0, 0.0, 0.0});
    sim->frame_buffers = realloc(sim->frame_buffers, (sim->size_frame_buffers + 1) * sizeof(uint8_t*));
    sim->frame_buffers[sim->size_frame_buffers] = calloc(WIDTH * HEIGHT * 3, sizeof(uint8_t));
    rasterize(sim->frame_buffers[sim->size_frame_buffers], sim->meshes, 2);
    sim->size_frame_buffers++;
}

void save_sim(Sim* sim){
    char filename[256];
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_flight.gif", localtime(&(time_t){time(NULL)}));
    write_gif(filename, WIDTH, HEIGHT, sim->frame_buffers, sim->size_frame_buffers);
}

void free_sim(Sim* sim){
    if(sim->render){
        free_meshes(sim->meshes, 2);
        for(int i = 0; i < sim->size_frame_buffers; i++){
            free(sim->frame_buffers[i]);
        }
        free(sim->frame_buffers);
    }
    free(sim->quad);
    free(sim);
}

#endif // SIM_H