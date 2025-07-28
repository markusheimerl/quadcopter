#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "../math/vec3.h"

typedef struct {
    Vec3 v0, v1, v2;      // Vertices
    Vec2 t0, t1, t2;      // Texture coordinates
    Vec3 n0, n1, n2;      // Vertex normals
} Triangle;

#endif