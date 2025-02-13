#ifndef MESH_H
#define MESH_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webp/decode.h>
#include <webp/encode.h>
#include "bvh.h"

// Modify the Mesh struct
typedef struct {
    Triangle* triangles;
    size_t triangle_count;
    unsigned char* texture_data;
    int texture_width;
    int texture_height;
    BVH bvh;
    Transform transform;
} Mesh;

Mesh create_mesh(const char* obj_filename, const char* texture_filename) {
    Mesh mesh = {
        .triangles = NULL,
        .triangle_count = 0,
        .texture_data = NULL,
        .texture_width = 0,
        .texture_height = 0,
        .bvh = {
            .root = NULL,
            .triangles = NULL,
            .triangle_count = 0
        },
        .transform = {
            .position = {0, 0, 0},
            .rotation = {0, 0, 0}
        }
    };
    // Load geometry
    Vec3* vertices = (Vec3*)malloc(1000000 * sizeof(Vec3));
    Vec2* texcoords = (Vec2*)malloc(1000000 * sizeof(Vec2));
    Vec3* normals = (Vec3*)malloc(1000000 * sizeof(Vec3));
    int vertex_count = 0, texcoord_count = 0, normal_count = 0, triangle_count = 0;
    mesh.triangles = (Triangle*)malloc(1000000 * sizeof(Triangle));

    FILE* file = fopen(obj_filename, "r");
    if (!file) { 
        fprintf(stderr, "Failed to open %s\n", obj_filename); 
        free(vertices);
        free(texcoords);
        free(normals);
        return mesh;
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
        } else if (line[0] == 'v' && line[1] == 'n') {
            sscanf(line + 3, "%f %f %f",
                &normals[normal_count].x,
                &normals[normal_count].y,
                &normals[normal_count].z);
            normal_count++;
        } else if (line[0] == 'f') {
            int v1, v2, v3, t1, t2, t3, n1, n2, n3;
            sscanf(line + 2, "%d/%d/%d %d/%d/%d %d/%d/%d",
                &v1, &t1, &n1,
                &v2, &t2, &n2,
                &v3, &t3, &n3);
            mesh.triangles[triangle_count].v0 = vertices[v1-1];
            mesh.triangles[triangle_count].v1 = vertices[v2-1];
            mesh.triangles[triangle_count].v2 = vertices[v3-1];
            mesh.triangles[triangle_count].t0 = texcoords[t1-1];
            mesh.triangles[triangle_count].t1 = texcoords[t2-1];
            mesh.triangles[triangle_count].t2 = texcoords[t3-1];
            mesh.triangles[triangle_count].n0 = normals[n1-1];
            mesh.triangles[triangle_count].n1 = normals[n2-1];
            mesh.triangles[triangle_count].n2 = normals[n3-1];
            triangle_count++;
        }
    }
    mesh.triangle_count = triangle_count;
    fclose(file);

    // Load texture
    FILE* tex_file = fopen(texture_filename, "rb");
    if (!tex_file) {
        fprintf(stderr, "Failed to open texture %s\n", texture_filename);
        free(vertices);
        free(texcoords);
        free(normals);
        return mesh;
    }

    fseek(tex_file, 0, SEEK_END);
    size_t file_size = ftell(tex_file);
    fseek(tex_file, 0, SEEK_SET);

    uint8_t* file_data = (uint8_t*)malloc(file_size);
    if (fread(file_data, 1, file_size, tex_file) != file_size) {
        free(file_data);
        fclose(tex_file);
        free(vertices);
        free(texcoords);
        free(normals);
        return mesh;
    }
    fclose(tex_file);

    mesh.texture_data = WebPDecodeRGBA(file_data, file_size, 
                                      &mesh.texture_width, 
                                      &mesh.texture_height);
    free(file_data);

    mesh.bvh = create_bvh(mesh.triangles, triangle_count);

    printf("Loaded %d vertices, %d texcoords, %d normals, %d triangles\n", 
           vertex_count, texcoord_count, normal_count, triangle_count);

    free(vertices);
    free(texcoords);
    free(normals);
    return mesh;
}

void set_mesh_position(Mesh* mesh, Vec3 position) {
    mesh->transform.position = position;
}

void set_mesh_rotation(Mesh* mesh, Vec3 rotation) {
    mesh->transform.rotation = rotation;
}

void destroy_mesh(Mesh* mesh) {
    if (mesh->triangles) free(mesh->triangles);
    if (mesh->texture_data) WebPFree(mesh->texture_data);
    destroy_bvh(&mesh->bvh);
    mesh->triangles = NULL;
    mesh->texture_data = NULL;
    mesh->triangle_count = 0;
}

Vec3 sample_mesh_texture(const Mesh* mesh, float u, float v) {
    u = u - floorf(u);
    v = v - floorf(v);
    int x = (int)(u * (mesh->texture_width - 1));
    int y = (int)(v * (mesh->texture_height - 1));
    int idx = (y * mesh->texture_width + x) * 4;
    return (Vec3){
        mesh->texture_data[idx] / 255.0f,
        mesh->texture_data[idx + 1] / 255.0f,
        mesh->texture_data[idx + 2] / 255.0f
    };
}

#endif