#ifndef SCENE_H
#define SCENE_H

#include "mesh.h"
#include "bvh.h"
#include <webp/encode.h>
#include <webp/mux.h>

typedef struct {
    Vec3 direction;  // Direction the light is coming from
    Vec3 color;      // Color and intensity of the light
} DirectionalLight;

typedef struct {
    Mesh* meshes;
    size_t mesh_count;
    Camera camera;
    DirectionalLight light;
    unsigned char** frames;  // Array of frame buffers
    int frame_count;        // Total number of frames
    int current_frame;      // Current frame being rendered
    int width;
    int height;
} Scene;

Scene create_scene(int width, int height, int frame_count) {
    Scene scene = {
        .meshes = NULL,
        .mesh_count = 0,
        .width = width,
        .height = height,
        .frame_count = frame_count,
        .current_frame = 0,
        .frames = malloc(frame_count * sizeof(unsigned char*))
    };
    
    // Allocate memory for each frame
    for (int i = 0; i < frame_count; i++) {
        scene.frames[i] = malloc(width * height * 3);
    }
    
    return scene;
}

void add_mesh_to_scene(Scene* scene, Mesh mesh) {
    scene->meshes = realloc(scene->meshes, (scene->mesh_count + 1) * sizeof(Mesh));
    scene->meshes[scene->mesh_count] = mesh;
    scene->mesh_count++;
}

void set_scene_camera(Scene* scene, Vec3 position, Vec3 look_at, Vec3 up, float fov) {
    scene->camera = create_camera(position, look_at, up, fov);
}

void set_scene_light(Scene* scene, Vec3 direction, Vec3 color) {
    scene->light.direction = vec3_normalize(direction);
    scene->light.color = color;
}

void next_frame(Scene* scene) {
    scene->current_frame++;
    if (scene->current_frame >= scene->frame_count) {
        scene->current_frame = scene->frame_count - 1;
    }
}

bool intersect_bvh(BVHNode* node, Ray ray, const Triangle* triangles,
                   float* t_out, float* u_out, float* v_out, int* tri_idx) {
    if (!ray_aabb_intersect(ray, node->bounds)) return false;

    bool hit = false;
    float closest_t = *t_out;

    if (node->left == NULL && node->right == NULL) {
        // Leaf node - test all triangles
        for (int i = 0; i < node->triangle_count; i++) {
            float t, u, v;
            if (ray_triangle_intersect(ray,
                triangles[node->start_idx + i].v0,
                triangles[node->start_idx + i].v1,
                triangles[node->start_idx + i].v2,
                &t, &u, &v) && t < closest_t) {
                closest_t = t;
                *t_out = t;
                *u_out = u;
                *v_out = v;
                *tri_idx = node->start_idx + i;
                hit = true;
            }
        }
    } else {
        // Internal node - recurse
        float t1 = INFINITY, t2 = INFINITY;
        float u1, v1, u2, v2;
        int idx1, idx2;

        bool hit1 = node->left ? intersect_bvh(node->left, ray, triangles, &t1, &u1, &v1, &idx1) : false;
        bool hit2 = node->right ? intersect_bvh(node->right, ray, triangles, &t2, &u2, &v2, &idx2) : false;

        if (hit1 && (!hit2 || t1 < t2)) {
            *t_out = t1;
            *u_out = u1;
            *v_out = v1;
            *tri_idx = idx1;
            hit = true;
        } else if (hit2) {
            *t_out = t2;
            *u_out = u2;
            *v_out = v2;
            *tri_idx = idx2;
            hit = true;
        }
    }

    return hit;
}

void render_scene(Scene* scene) {
    float aspect = (float)scene->width / scene->height;
    unsigned char* current_frame = scene->frames[scene->current_frame];

    for (int y = 0; y < scene->height; y++) {
        for (int x = 0; x < scene->width; x++) {
            Ray ray = get_camera_ray(&scene->camera, 
                                   (x + 0.5f) / scene->width, 
                                   (y + 0.5f) / scene->height, 
                                   aspect);
            
            float closest_t = INFINITY;
            bool hit = false;
            Vec2 hit_uv = {0, 0};
            Vec3 hit_normal = {0, 0, 0};
            const Mesh* hit_mesh = NULL;

            // Check intersection with all meshes using BVH
            for (size_t m = 0; m < scene->mesh_count; m++) {
                const Mesh* current_mesh = &scene->meshes[m];
                float t = closest_t;
                float u, v;
                int tri_idx;
                
                // Transform ray to mesh local space
                Ray transformed_ray = transform_ray(ray, current_mesh->transform);
                
                if (intersect_bvh(current_mesh->bvh.root, transformed_ray, current_mesh->triangles,
                                 &t, &u, &v, &tri_idx) && t < closest_t) {
                    closest_t = t;
                    hit = true;
                    hit_mesh = current_mesh;
                    float w = 1.0f - u - v;

                    // Interpolate texture coordinates
                    hit_uv.u = w * current_mesh->triangles[tri_idx].t0.u + 
                              u * current_mesh->triangles[tri_idx].t1.u + 
                              v * current_mesh->triangles[tri_idx].t2.u;
                    hit_uv.v = w * current_mesh->triangles[tri_idx].t0.v + 
                              u * current_mesh->triangles[tri_idx].t1.v + 
                              v * current_mesh->triangles[tri_idx].t2.v;

                    // Interpolate normal
                    hit_normal = vec3_normalize(vec3_add(
                        vec3_add(
                            vec3_mul(current_mesh->triangles[tri_idx].n0, w),
                            vec3_mul(current_mesh->triangles[tri_idx].n1, u)
                        ),
                        vec3_mul(current_mesh->triangles[tri_idx].n2, v)
                    ));
                }
            }

            int idx = (y * scene->width + x) * 3;
            if (hit && hit_mesh) {
                Vec3 color = sample_mesh_texture(hit_mesh, hit_uv.u, hit_uv.v);
                
                // Calculate diffuse lighting
                float diffuse = 0.2f;  // Ambient light level
                
                // Calculate hit point in world space using original ray
                Vec3 hit_point = vec3_add(ray.origin, vec3_mul(ray.direction, closest_t));
                Vec3 shadow_origin = vec3_add(hit_point, vec3_mul(hit_normal, 0.001f));
                Ray shadow_ray = {shadow_origin, scene->light.direction};
                
                // Check if point is in shadow
                bool in_shadow = false;
                for (size_t m = 0; m < scene->mesh_count && !in_shadow; m++) {
                    const Mesh* current_mesh = &scene->meshes[m];
                    float shadow_t = INFINITY;
                    float shadow_u, shadow_v;
                    int shadow_tri_idx;
                    
                    // Transform shadow ray to mesh local space
                    Ray transformed_shadow_ray = transform_ray(shadow_ray, current_mesh->transform);
                    
                    if (intersect_bvh(current_mesh->bvh.root, transformed_shadow_ray, 
                                    current_mesh->triangles,
                                    &shadow_t, &shadow_u, &shadow_v, 
                                    &shadow_tri_idx)) {
                        in_shadow = true;
                    }
                }
                
                // Add direct lighting if not in shadow
                if (!in_shadow) {
                    diffuse = fmaxf(diffuse, 
                        vec3_dot(hit_normal, scene->light.direction));
                }
                
                // Apply lighting
                color = vec3_mul_vec3(color, scene->light.color);
                color = vec3_mul(color, diffuse);
                
                // Convert to RGB bytes
                current_frame[idx] = (unsigned char)(fminf(color.x * 255.0f, 255.0f));
                current_frame[idx + 1] = (unsigned char)(fminf(color.y * 255.0f, 255.0f));
                current_frame[idx + 2] = (unsigned char)(fminf(color.z * 255.0f, 255.0f));
            } else {
                current_frame[idx] = current_frame[idx + 1] = current_frame[idx + 2] = 50;
            }
        }
    }
}

void save_scene(Scene* scene, const char* filename) {
    // Prepare WebP animation configuration
    WebPAnimEncoderOptions anim_config;
    WebPAnimEncoderOptionsInit(&anim_config);
    WebPAnimEncoder* enc = WebPAnimEncoderNew(scene->width, scene->height, &anim_config);
    
    // Configure each frame
    WebPConfig config;
    WebPConfigInit(&config);
    config.lossless = 1;
    config.method = 4;
    config.image_hint = WEBP_HINT_GRAPH;
    
    // Prepare picture
    WebPPicture pic;
    WebPPictureInit(&pic);
    pic.width = scene->width;
    pic.height = scene->height;
    pic.use_argb = 1;
    WebPPictureAlloc(&pic);

    // Add each frame to the animation
    for (int frame = 0; frame < scene->frame_count; frame++) {
        // Convert frame to ARGB format
        for (int i = 0; i < scene->width * scene->height; i++) {
            pic.argb[i] = (0xFF << 24) |                           // Alpha
                         (scene->frames[frame][i * 3] << 16) |     // Red
                         (scene->frames[frame][i * 3 + 1] << 8) |  // Green
                         scene->frames[frame][i * 3 + 2];          // Blue
        }
        
        // Add frame to animation (50ms delay between frames)
        WebPAnimEncoderAdd(enc, &pic, frame * 50, &config);
    }
    
    // Finalize animation
    WebPAnimEncoderAdd(enc, NULL, scene->frame_count * 50, NULL);
    WebPData webp_data;
    WebPDataInit(&webp_data);
    WebPAnimEncoderAssemble(enc, &webp_data);
    
    // Save to file
    FILE* fp = fopen(filename, "wb");
    if (fp) {
        fwrite(webp_data.bytes, webp_data.size, 1, fp);
        fclose(fp);
    }
    
    // Cleanup
    WebPDataClear(&webp_data);
    WebPAnimEncoderDelete(enc);
    WebPPictureFree(&pic);
}

void destroy_scene(Scene* scene) {
    for (size_t i = 0; i < scene->mesh_count; i++) {
        destroy_mesh(&scene->meshes[i]);
    }
    free(scene->meshes);
    
    // Free all frame buffers
    for (int i = 0; i < scene->frame_count; i++) {
        free(scene->frames[i]);
    }
    free(scene->frames);
    
    scene->meshes = NULL;
    scene->frames = NULL;
    scene->mesh_count = 0;
}

#endif