#ifndef MESH_H
#define MESH_H

#include "raytracer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webp/decode.h>
#include <webp/encode.h>

typedef struct {
    Vec3 v0, v1, v2;      // Vertices
    Vec2 t0, t1, t2;      // Texture coordinates
} Triangle;

typedef struct {
    Triangle* triangles;
    size_t triangle_count;
} Mesh;

typedef struct {
    unsigned char* data;
    int width, height;
} Texture;

Mesh load_obj(const char* filename) {
    Vec3* vertices = malloc(1000000 * sizeof(Vec3));
    Vec2* texcoords = malloc(1000000 * sizeof(Vec2));
    int vertex_count = 0, texcoord_count = 0, triangle_count = 0;
    Mesh mesh = {malloc(1000000 * sizeof(Triangle)), 0};

    FILE* file = fopen(filename, "r");
    if (!file) { 
        fprintf(stderr, "Failed to open %s\n", filename); 
        exit(1); 
    }

    char line[256];
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == 'v' && line[1] == ' ') {
            sscanf(line + 2, "%f %f %f", 
                &vertices[vertex_count].x,
                &vertices[vertex_count].y,
                &vertices[vertex_count].z);
            vertex_count++;
        } else if (line[0] == 'v' && line[1] == 't') {
            sscanf(line + 3, "%f %f",
                &texcoords[texcoord_count].u,
                &texcoords[texcoord_count].v);
            texcoord_count++;
        } else if (line[0] == 'f') {
            int v1, v2, v3, t1, t2, t3;
            sscanf(line + 2, "%d/%d/%*d %d/%d/%*d %d/%d/%*d",
                &v1, &t1, &v2, &t2, &v3, &t3);
            mesh.triangles[triangle_count].v0 = vertices[v1-1];
            mesh.triangles[triangle_count].v1 = vertices[v2-1];
            mesh.triangles[triangle_count].v2 = vertices[v3-1];
            mesh.triangles[triangle_count].t0 = texcoords[t1-1];
            mesh.triangles[triangle_count].t1 = texcoords[t2-1];
            mesh.triangles[triangle_count].t2 = texcoords[t3-1];
            triangle_count++;
        }
    }
    mesh.triangle_count = triangle_count;
    printf("Loaded %d vertices, %d texcoords, %d triangles\n", 
           vertex_count, texcoord_count, triangle_count);

    free(vertices);
    free(texcoords);
    fclose(file);
    return mesh;
}

Texture load_texture(const char* filename) {
    Texture tex = {NULL, 0, 0};
    FILE* fp = fopen(filename, "rb");
    if (!fp) return tex;

    // Get file size
    fseek(fp, 0, SEEK_END);
    size_t file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    // Read file data
    uint8_t* file_data = malloc(file_size);
    if (fread(file_data, 1, file_size, fp) != file_size) {
        free(file_data);
        fclose(fp);
        return tex;
    }
    fclose(fp);

    // Get WebP image features
    WebPBitstreamFeatures features;
    if (WebPGetFeatures(file_data, file_size, &features) != VP8_STATUS_OK) {
        free(file_data);
        return tex;
    }

    // Decode WebP
    tex.width = features.width;
    tex.height = features.height;
    tex.data = WebPDecodeRGBA(file_data, file_size, &tex.width, &tex.height);

    free(file_data);
    return tex;
}

void save_webp(const char* filename, unsigned char* pixels, int width, int height) {
    // Convert RGB to RGBA (WebP encoder expects RGBA)
    uint8_t* rgba = malloc(width * height * 4);
    for (int i = 0; i < width * height; i++) {
        rgba[i * 4] = pixels[i * 3];     // R
        rgba[i * 4 + 1] = pixels[i * 3 + 1]; // G
        rgba[i * 4 + 2] = pixels[i * 3 + 2]; // B
        rgba[i * 4 + 3] = 255;           // A
    }

    // Encode WebP
    uint8_t* output;
    size_t output_size = WebPEncodeRGBA(rgba, width, height, width * 4, 75, &output);

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

Vec3 sample_texture(const Texture* tex, float u, float v) {
    u = u - floorf(u);
    v = v - floorf(v);
    int x = (int)(u * (tex->width - 1));
    int y = (int)(v * (tex->height - 1));
    int idx = (y * tex->width + x) * 4;
    return (Vec3){
        tex->data[idx] / 255.0f,
        tex->data[idx + 1] / 255.0f,
        tex->data[idx + 2] / 255.0f
    };
}

void render(const Mesh* mesh, const Texture* texture, unsigned char* pixels, int width, int height) {
    // Camera parameters
    Vec3 camera_pos = {3.0f, 2.0f, -3.0f};
    Vec3 look_at = {0.0f, 0.0f, 0.0f};
    Vec3 up = {0.0f, 1.0f, 0.0f};
    
    // Calculate camera coordinate system
    Vec3 forward = vec3_normalize(vec3_sub(look_at, camera_pos));
    Vec3 right = vec3_normalize(vec3_cross(forward, up));
    Vec3 camera_up = vec3_cross(right, forward);
    
    float fov = 60.0f;
    float scale = tanf((fov * 0.5f) * M_PI / 180.0f);
    float aspect = (float)width / height;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float ray_x = (2.0f * ((x + 0.5f) / width) - 1.0f) * aspect * scale;
            float ray_y = (1.0f - 2.0f * ((y + 0.5f) / height)) * scale;

            // Calculate ray direction using camera basis vectors
            Vec3 ray_dir = vec3_normalize(vec3_add(
                vec3_add(
                    vec3_mul(right, ray_x),
                    vec3_mul(camera_up, ray_y)
                ),
                forward
            ));

            Ray ray = {camera_pos, ray_dir};
            float closest_t = INFINITY;
            bool hit = false;
            Vec2 hit_uv = {0, 0};

            for (size_t i = 0; i < mesh->triangle_count; i++) {
                float t, u, v;
                if (ray_triangle_intersect(ray, 
                                        mesh->triangles[i].v0,
                                        mesh->triangles[i].v1,
                                        mesh->triangles[i].v2,
                                        &t, &u, &v) && t < closest_t) {
                    closest_t = t;
                    hit = true;
                    float w = 1.0f - u - v;
                    hit_uv.u = w * mesh->triangles[i].t0.u + 
                              u * mesh->triangles[i].t1.u + 
                              v * mesh->triangles[i].t2.u;
                    hit_uv.v = w * mesh->triangles[i].t0.v + 
                              u * mesh->triangles[i].t1.v + 
                              v * mesh->triangles[i].t2.v;
                }
            }

            int idx = (y * width + x) * 3;
            if (hit) {
                Vec3 color = sample_texture(texture, hit_uv.u, hit_uv.v);
                pixels[idx] = (unsigned char)(color.x * 255.0f);
                pixels[idx + 1] = (unsigned char)(color.y * 255.0f);
                pixels[idx + 2] = (unsigned char)(color.z * 255.0f);
            } else {
                pixels[idx] = pixels[idx + 1] = pixels[idx + 2] = 50;
            }
        }
    }
}

#endif