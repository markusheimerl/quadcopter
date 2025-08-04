#include "aabb.h"

AABB create_empty_aabb(void) {
    return (AABB){
        (Vec3){1e30f, 1e30f, 1e30f},
        (Vec3){-1e30f, -1e30f, -1e30f}
    };
}

AABB expand_aabb(AABB box, Vec3 point) {
    return (AABB){
        (Vec3){
            fminf(box.min.x, point.x),
            fminf(box.min.y, point.y),
            fminf(box.min.z, point.z)
        },
        (Vec3){
            fmaxf(box.max.x, point.x),
            fmaxf(box.max.y, point.y),
            fmaxf(box.max.z, point.z)
        }
    };
}

AABB get_triangle_bounds(Triangle tri) {
    AABB bounds = create_empty_aabb();
    bounds = expand_aabb(bounds, tri.v0);
    bounds = expand_aabb(bounds, tri.v1);
    bounds = expand_aabb(bounds, tri.v2);
    return bounds;
}

bool ray_aabb_intersect(Ray ray, AABB box) {
    Vec3 inv_dir = (Vec3){
        1.0f / ray.direction.x,
        1.0f / ray.direction.y,
        1.0f / ray.direction.z
    };

    float tx1 = (box.min.x - ray.origin.x) * inv_dir.x;
    float tx2 = (box.max.x - ray.origin.x) * inv_dir.x;
    float tmin = fminf(tx1, tx2);
    float tmax = fmaxf(tx1, tx2);

    float ty1 = (box.min.y - ray.origin.y) * inv_dir.y;
    float ty2 = (box.max.y - ray.origin.y) * inv_dir.y;
    tmin = fmaxf(tmin, fminf(ty1, ty2));
    tmax = fminf(tmax, fmaxf(ty1, ty2));

    float tz1 = (box.min.z - ray.origin.z) * inv_dir.z;
    float tz2 = (box.max.z - ray.origin.z) * inv_dir.z;
    tmin = fmaxf(tmin, fminf(tz1, tz2));
    tmax = fminf(tmax, fmaxf(tz1, tz2));

    return tmax >= tmin && tmax > 0;
}