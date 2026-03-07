#ifndef MAT4_H
#define MAT4_H

#include "vec3.h"

typedef struct {
    float m[4][4];
} Mat4;

// Matrix operations
Mat4 mat4_identity(void);
Mat4 mat4_multiply(Mat4 a, Mat4 b);
Mat4 mat4_translation(Vec3 t);
Mat4 mat4_rotation_x(float angle);
Mat4 mat4_rotation_y(float angle);
Mat4 mat4_rotation_z(float angle);
Vec3 mat4_transform_point(Mat4 m, Vec3 p);
Vec3 mat4_transform_vector(Mat4 m, Vec3 v);
Mat4 mat4_inverse(Mat4 m);
Mat4 mat4_transpose(Mat4 m);

#endif