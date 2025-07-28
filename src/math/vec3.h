#ifndef VEC3_H
#define VEC3_H

#include <math.h>

typedef struct { float x, y, z; } Vec3;
typedef struct { float u, v; } Vec2;

// Vec3 operations
Vec3 vec3_add(Vec3 a, Vec3 b);
Vec3 vec3_sub(Vec3 a, Vec3 b);
Vec3 vec3_mul(Vec3 a, float t);
Vec3 vec3_div(Vec3 a, float t);
float vec3_dot(Vec3 a, Vec3 b);
Vec3 vec3_cross(Vec3 a, Vec3 b);
float vec3_length(Vec3 v);
Vec3 vec3_normalize(Vec3 v);
Vec3 vec3_mul_vec3(Vec3 a, Vec3 b);

#endif