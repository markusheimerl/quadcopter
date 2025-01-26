#ifndef SIM_H
#define SIM_H

#include <stdbool.h>
#include <time.h>
#include "raytracer.h"
#include "quad.h"

#define MAX_TIME 30.0

typedef struct {
    Mesh** meshes;
    Quad* quad;
    unsigned char **frame_buffers;
    int size_frame_buffers;
    bool render;
} Sim;

Sim* init_sim(const char* prefix, bool render){
    Sim* sim = malloc(sizeof(Sim));
    if(render){
        char path[256];
        sim->meshes = malloc(2 * sizeof(Mesh*));
        
        // Load drone mesh
        snprintf(path, sizeof(path), "%sraytracer/drone.obj", prefix);
        char tex_path[256];
        snprintf(tex_path, sizeof(tex_path), "%sraytracer/drone.webp", prefix);
        sim->meshes[0] = malloc(sizeof(Mesh));
        *sim->meshes[0] = init_mesh(path, tex_path);
            
        // Load ground mesh
        snprintf(path, sizeof(path), "%sraytracer/ground.obj", prefix);
        snprintf(tex_path, sizeof(tex_path), "%sraytracer/ground.webp", prefix);
        sim->meshes[1] = malloc(sizeof(Mesh));
        *sim->meshes[1] = init_mesh(path, tex_path);
            
        sim->frame_buffers = NULL;
        sim->size_frame_buffers = 0;

        // Initial ground transform
        float ground_pos[3] = {0.0f, 0.0f, 0.0f};
        float ground_scale[3] = {1.0f, 1.0f, 1.0f};
        float ground_transform[16];
        float temp[16];
        mat4_scale(temp, ground_scale);
        mat4_translate(ground_transform, ground_pos);
        mat4_multiply(ground_transform, ground_transform, temp);
        transform_mesh(sim->meshes[1], ground_transform);
    }
    sim->render = render;
    sim->quad = init_quad(0.0, 0.0, 0.0);
    return sim;
}

void render_sim(Sim* sim){
    // Convert quad transform to raytracer format
    float pos[3] = {
        (float)sim->quad->linear_position_W[0],
        (float)sim->quad->linear_position_W[1],
        (float)sim->quad->linear_position_W[2]
    };
    float scale[3] = {1.0f, 1.0f, 1.0f};
    float transform[16], temp[16];
    
    mat4_scale(temp, scale);
    mat4_translate(transform, pos);
    mat4_multiply(transform, transform, temp);
    
    // Apply rotation from quad
    float rotation[16] = {
        sim->quad->R_W_B[0], sim->quad->R_W_B[1], sim->quad->R_W_B[2], 0,
        sim->quad->R_W_B[3], sim->quad->R_W_B[4], sim->quad->R_W_B[5], 0,
        sim->quad->R_W_B[6], sim->quad->R_W_B[7], sim->quad->R_W_B[8], 0,
        0, 0, 0, 1
    };
    mat4_multiply(transform, transform, rotation);
    
    transform_mesh(sim->meshes[0], transform);

    // Allocate and render new frame
    const int width = 800;
    const int height = 600;
    sim->frame_buffers = realloc(sim->frame_buffers, (sim->size_frame_buffers + 1) * sizeof(unsigned char*));
    sim->frame_buffers[sim->size_frame_buffers] = malloc(width * height * 3);
    
    const Mesh* mesh_array[] = {sim->meshes[0], sim->meshes[1]};
    render(mesh_array, 2, sim->frame_buffers[sim->size_frame_buffers], width, height);
    
    sim->size_frame_buffers++;
}

void save_sim(Sim* sim, double duration){
    char filename[256];
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_flight.webp", 
             localtime(&(time_t){time(NULL)}));
    save_animated_webp(filename, sim->frame_buffers, sim->size_frame_buffers, 
                      800, 600, (int)(duration * 1000));
}

void free_sim(Sim* sim){
    if(sim->render){
        for(int i = 0; i < 2; i++){
            free(sim->meshes[i]->triangles);
            free(sim->meshes[i]->transformed);
            free(sim->meshes[i]->texture_data);
            if(sim->meshes[i]->bvh) free_bvh(sim->meshes[i]->bvh);
            free(sim->meshes[i]);
        }
        free(sim->meshes);
        
        for(int i = 0; i < sim->size_frame_buffers; i++){
            free(sim->frame_buffers[i]);
        }
        free(sim->frame_buffers);
    }
    free(sim->quad);
    free(sim);
}

#endif // SIM_H