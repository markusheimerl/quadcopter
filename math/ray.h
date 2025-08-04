#ifndef RAY_H
#define RAY_H

#include "vec3.h"
#include "mat4.h"
#include <stdbool.h>

typedef struct { 
    Vec3 origin, direction; 
} Ray;

typedef struct {
    Vec3 position;     // Translation vector
    Vec3 rotation;     // Rotation in radians (around x, y, z axes)
} Transform;

// Ray operations
Ray transform_ray(Ray ray, Transform transform);
Vec3 transform_normal(Vec3 normal, Transform transform);
bool ray_triangle_intersect(Ray ray, Vec3 v0, Vec3 v1, Vec3 v2, 
                          float* t, float* u_out, float* v_out);

#endif