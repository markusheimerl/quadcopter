#ifndef SCENE_H
#define SCENE_H

#include "mesh.h"

typedef struct {
    Mesh* meshes;
    size_t mesh_count;
    Camera camera;
    unsigned char* pixels;
    int width;
    int height;
} Scene;

Scene create_scene(int width, int height) {
    Scene scene = {
        .meshes = NULL,
        .mesh_count = 0,
        .width = width,
        .height = height,
        .pixels = malloc(width * height * 3)
    };
    return scene;
}

void add_mesh_to_scene(Scene* scene, Mesh mesh) {
    scene->meshes = realloc(scene->meshes, (scene->mesh_count + 1) * sizeof(Mesh));
    scene->meshes[scene->mesh_count] = mesh;
    scene->mesh_count++;
}

void set_scene_camera(Scene* scene, Vec3 position, Vec3 look_at, Vec3 up, float fov) {
    scene->camera = create_camera(position, look_at, up, fov);
}

void render_scene(Scene* scene) {
    float aspect = (float)scene->width / scene->height;

    for (int y = 0; y < scene->height; y++) {
        for (int x = 0; x < scene->width; x++) {
            Ray ray = get_camera_ray(&scene->camera, 
                                   (x + 0.5f) / scene->width, 
                                   (y + 0.5f) / scene->height, 
                                   aspect);
            
            float closest_t = INFINITY;
            bool hit = false;
            Vec2 hit_uv = {0, 0};
            const Mesh* hit_mesh = NULL;

            // Check intersection with all meshes
            for (size_t m = 0; m < scene->mesh_count; m++) {
                const Mesh* current_mesh = &scene->meshes[m];
                for (size_t i = 0; i < current_mesh->triangle_count; i++) {
                    float t, u, v;
                    if (ray_triangle_intersect(ray, 
                                             current_mesh->triangles[i].v0,
                                             current_mesh->triangles[i].v1,
                                             current_mesh->triangles[i].v2,
                                             &t, &u, &v) && t < closest_t) {
                        closest_t = t;
                        hit = true;
                        hit_mesh = current_mesh;
                        float w = 1.0f - u - v;
                        hit_uv.u = w * current_mesh->triangles[i].t0.u + 
                                  u * current_mesh->triangles[i].t1.u + 
                                  v * current_mesh->triangles[i].t2.u;
                        hit_uv.v = w * current_mesh->triangles[i].t0.v + 
                                  u * current_mesh->triangles[i].t1.v + 
                                  v * current_mesh->triangles[i].t2.v;
                    }
                }
            }

            int idx = (y * scene->width + x) * 3;
            if (hit && hit_mesh) {
                Vec3 color = sample_mesh_texture(hit_mesh, hit_uv.u, hit_uv.v);
                scene->pixels[idx] = (unsigned char)(color.x * 255.0f);
                scene->pixels[idx + 1] = (unsigned char)(color.y * 255.0f);
                scene->pixels[idx + 2] = (unsigned char)(color.z * 255.0f);
            } else {
                scene->pixels[idx] = scene->pixels[idx + 1] = scene->pixels[idx + 2] = 50;
            }
        }
    }
}

void save_scene(Scene* scene, const char* filename) {
    uint8_t* rgba = malloc(scene->width * scene->height * 4);
    for (int i = 0; i < scene->width * scene->height; i++) {
        rgba[i * 4] = scene->pixels[i * 3];
        rgba[i * 4 + 1] = scene->pixels[i * 3 + 1];
        rgba[i * 4 + 2] = scene->pixels[i * 3 + 2];
        rgba[i * 4 + 3] = 255;
    }

    uint8_t* output;
    size_t output_size = WebPEncodeRGBA(rgba, scene->width, scene->height, 
                                       scene->width * 4, 75, &output);

    if (output_size > 0) {
        FILE* fp = fopen(filename, "wb");
        if (fp) {
            fwrite(output, output_size, 1, fp);
            fclose(fp);
        }
        WebPFree(output);
    }

    free(rgba);
}

void destroy_scene(Scene* scene) {
    for (size_t i = 0; i < scene->mesh_count; i++) {
        destroy_mesh(&scene->meshes[i]);
    }
    free(scene->meshes);
    free(scene->pixels);
    scene->meshes = NULL;
    scene->pixels = NULL;
    scene->mesh_count = 0;
}

#endif