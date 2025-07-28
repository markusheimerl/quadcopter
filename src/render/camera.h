#ifndef CAMERA_H
#define CAMERA_H

#include "../math/vec3.h"
#include "../math/ray.h"

typedef struct {
    Vec3 position; 
    Vec3 look_at; 
    Vec3 up; 
    float fov; 
} Camera;

// Camera operations
Camera create_camera(Vec3 position, Vec3 look_at, Vec3 up, float fov);
Ray get_camera_ray(const Camera* camera, float x, float y, float aspect);

#endif