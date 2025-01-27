#ifndef BVH_H
#define BVH_H

#include "ray.h"

typedef struct {
    Vec3 min;
    Vec3 max;
} AABB;

typedef struct BVHNode {
    AABB bounds;
    struct BVHNode* left;
    struct BVHNode* right;
    int start_idx;
    int triangle_count;
} BVHNode;

typedef struct {
    BVHNode* root;
    Triangle* triangles;
    size_t triangle_count;
} BVH;

// AABB functions
AABB create_empty_aabb() {
    return (AABB){
        (Vec3){INFINITY, INFINITY, INFINITY},
        (Vec3){-INFINITY, -INFINITY, -INFINITY}
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

BVHNode* create_bvh_node(Triangle* triangles, int start, int count) {
    BVHNode* node = malloc(sizeof(BVHNode));
    node->start_idx = start;
    node->triangle_count = count;
    node->left = node->right = NULL;

    // Calculate bounds
    node->bounds = create_empty_aabb();
    for (int i = 0; i < count; i++) {
        node->bounds = expand_aabb(node->bounds, triangles[start + i].v0);
        node->bounds = expand_aabb(node->bounds, triangles[start + i].v1);
        node->bounds = expand_aabb(node->bounds, triangles[start + i].v2);
    }

    // Split if more than 4 triangles
    if (count > 4) {
        // Find longest axis
        Vec3 extent = vec3_sub(node->bounds.max, node->bounds.min);
        int axis = 0;
        if (extent.y > extent.x) axis = 1;
        if (extent.z > extent.x && extent.z > extent.y) axis = 2;

        // Sort triangles based on centroid
        float split = 0.0f;
        for (int i = 0; i < count; i++) {
            Triangle* tri = &triangles[start + i];
            Vec3 centroid = vec3_mul(vec3_add(vec3_add(tri->v0, tri->v1), tri->v2), 1.0f/3.0f);
            split += axis == 0 ? centroid.x : (axis == 1 ? centroid.y : centroid.z);
        }
        split /= count;

        // Partition triangles
        int mid = start;
        for (int i = 0; i < count; i++) {
            Triangle* tri = &triangles[start + i];
            Vec3 centroid = vec3_mul(vec3_add(vec3_add(tri->v0, tri->v1), tri->v2), 1.0f/3.0f);
            float value = axis == 0 ? centroid.x : (axis == 1 ? centroid.y : centroid.z);
            
            if (value < split) {
                Triangle temp = triangles[start + i];
                triangles[start + i] = triangles[mid];
                triangles[mid] = temp;
                mid++;
            }
        }

        // Create children
        int left_count = mid - start;
        if (left_count > 0 && left_count < count) {
            node->left = create_bvh_node(triangles, start, left_count);
            node->right = create_bvh_node(triangles, mid, count - left_count);
        }
    }

    return node;
}

BVH create_bvh(Triangle* triangles, size_t count) {
    BVH bvh;
    bvh.triangles = triangles;
    bvh.triangle_count = count;
    bvh.root = create_bvh_node(triangles, 0, count);
    return bvh;
}

void destroy_bvh_node(BVHNode* node) {
    if (node->left) destroy_bvh_node(node->left);
    if (node->right) destroy_bvh_node(node->right);
    free(node);
}

void destroy_bvh(BVH* bvh) {
    if (bvh->root) destroy_bvh_node(bvh->root);
    bvh->root = NULL;
}

#endif