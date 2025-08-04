#include "camera.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

Camera create_camera(Vec3 position, Vec3 look_at, Vec3 up, float fov) {
    return (Camera){position, look_at, up, fov};
}

Ray get_camera_ray(const Camera* camera, float x, float y, float aspect) {
    Vec3 forward = vec3_normalize(vec3_sub(camera->look_at, camera->position));
    Vec3 right = vec3_normalize(vec3_cross(forward, camera->up));
    Vec3 camera_up = vec3_cross(right, forward);
    
    float scale = tanf((camera->fov * 0.5f) * M_PI / 180.0f);
    float ray_x = (2.0f * x - 1.0f) * aspect * scale;
    float ray_y = (1.0f - 2.0f * y) * scale;

    Vec3 ray_dir = vec3_normalize(vec3_add(
        vec3_add(
            vec3_mul(right, ray_x),
            vec3_mul(camera_up, ray_y)
        ),
        forward
    ));

    return (Ray){camera->position, ray_dir};
}