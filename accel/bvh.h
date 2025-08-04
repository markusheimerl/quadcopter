#ifndef BVH_H
#define BVH_H

#include "geometry/aabb.h"
#include "geometry/triangle.h"
#include <stdlib.h>

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

// BVH operations
BVHNode* create_bvh_node(Triangle* triangles, int start, int count);
BVH create_bvh(Triangle* triangles, size_t count);
void destroy_bvh_node(BVHNode* node);
void destroy_bvh(BVH* bvh);
bool intersect_bvh(BVHNode* node, Ray ray, const Triangle* triangles,
                   float* t_out, float* u_out, float* v_out, int* tri_idx);

#endif