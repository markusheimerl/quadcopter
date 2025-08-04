#include "ray.h"

Ray transform_ray(Ray ray, Transform transform) {
    // Create transformation matrix
    Mat4 rot_x = mat4_rotation_x(transform.rotation.x);
    Mat4 rot_y = mat4_rotation_y(transform.rotation.y);
    Mat4 rot_z = mat4_rotation_z(transform.rotation.z);
    Mat4 trans = mat4_translation(transform.position);
    
    // Combine transformations
    Mat4 transform_matrix = mat4_multiply(trans, 
                           mat4_multiply(rot_z,
                           mat4_multiply(rot_y, rot_x)));
    
    // Get inverse transform
    Mat4 inv_transform = mat4_inverse(transform_matrix);
    
    // Transform ray origin and direction
    Vec3 new_origin = mat4_transform_point(inv_transform, ray.origin);
    Vec3 new_direction = mat4_transform_vector(inv_transform, ray.direction);
    
    return (Ray){new_origin, vec3_normalize(new_direction)};
}

Vec3 transform_normal(Vec3 normal, Transform transform) {
    // Create rotation matrices
    Mat4 rot_x = mat4_rotation_x(transform.rotation.x);
    Mat4 rot_y = mat4_rotation_y(transform.rotation.y);
    Mat4 rot_z = mat4_rotation_z(transform.rotation.z);
    
    // Combine rotations
    Mat4 rotation = mat4_multiply(rot_z,
                    mat4_multiply(rot_y, rot_x));
    
    // Transform normal using only the rotation part
    // Note: We use the transpose of the inverse for normal transformation
    Mat4 normal_transform = mat4_transpose(mat4_inverse(rotation));
    
    return vec3_normalize(mat4_transform_vector(normal_transform, normal));
}

bool ray_triangle_intersect(Ray ray, Vec3 v0, Vec3 v1, Vec3 v2, 
                          float* t, float* u_out, float* v_out) {
    const float EPSILON = 0.0000001f;
    Vec3 edge1 = vec3_sub(v1, v0);
    Vec3 edge2 = vec3_sub(v2, v0);
    Vec3 h = vec3_cross(ray.direction, edge2);
    float a = vec3_dot(edge1, h);

    if (a > -EPSILON && a < EPSILON) return false;

    float f = 1.0f / a;
    Vec3 s = vec3_sub(ray.origin, v0);
    float u = f * vec3_dot(s, h);

    if (u < 0.0f || u > 1.0f) return false;

    Vec3 q = vec3_cross(s, edge1);
    float v = f * vec3_dot(ray.direction, q);

    if (v < 0.0f || u + v > 1.0f) return false;

    *t = f * vec3_dot(edge2, q);
    *u_out = u;
    *v_out = v;
    return *t > EPSILON;
}