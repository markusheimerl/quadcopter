#ifndef MESH_H
#define MESH_H

#include "../geometry/triangle.h"
#include "../accel/bvh.h"
#include "../math/ray.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    Triangle* triangles;
    size_t triangle_count;
    unsigned char* texture_data;
    int texture_width;
    int texture_height;
    BVH bvh;
    Transform transform;
} Mesh;

// Mesh operations
Mesh create_mesh(const char* obj_filename, const char* texture_filename);
void set_mesh_position(Mesh* mesh, Vec3 position);
void set_mesh_rotation(Mesh* mesh, Vec3 rotation);
void destroy_mesh(Mesh* mesh);
Vec3 sample_mesh_texture(const Mesh* mesh, float u, float v);

#endif