#ifndef AABB_H
#define AABB_H

#include "math/vec3.h"
#include "math/ray.h"
#include "triangle.h"
#include <stdbool.h>

typedef struct {
    Vec3 min;
    Vec3 max;
} AABB;

// AABB operations
AABB create_empty_aabb(void);
AABB expand_aabb(AABB box, Vec3 point);
AABB get_triangle_bounds(Triangle tri);
bool ray_aabb_intersect(Ray ray, AABB box);

#endif