#ifndef SCENE_H
#define SCENE_H

#include "mesh.h"
#include "bvh.h"
#include <webp/encode.h>
#include <time.h>
#include <webp/mux.h>

typedef struct {
    Vec3 direction;
    Vec3 color;
} DirectionalLight;

typedef struct {
    Mesh* meshes;
    size_t mesh_count;
    Camera camera;
    DirectionalLight light;
    unsigned char** frames;
    int frame_count;
    int current_frame;
    int width;
    int height;
    float scale_factor;
    int duration_ms;
    int fps;
} Scene;

Scene create_scene(int width, int height, int duration_ms, int fps, float scale_factor) {
    int frame_count = (duration_ms * fps) / 1000;

    Scene scene = {
        .meshes = NULL,
        .mesh_count = 0,
        .width = (int)(width * scale_factor),
        .height = (int)(height * scale_factor),
        .scale_factor = scale_factor,
        .frame_count = frame_count,
        .current_frame = 0,
        .duration_ms = duration_ms,
        .fps = fps,
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

                    // Transform the interpolated normal according to the mesh's transformation
                    hit_normal = transform_normal(hit_normal, hit_mesh->transform);
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

// Cubic interpolation helper function
float cubic_hermite(float A, float B, float C, float D, float t) {
    float a = -A/2.0f + (3.0f*B)/2.0f - (3.0f*C)/2.0f + D/2.0f;
    float b = A - (5.0f*B)/2.0f + 2.0f*C - D/2.0f;
    float c = -A/2.0f + C/2.0f;
    float d = B;

    return a*t*t*t + b*t*t + c*t + d;
}

// Get pixel value safely with bounds checking
uint32_t get_pixel_rgb(unsigned char* frame, int x, int y, int width, int height) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= width) x = width - 1;
    if (y >= height) y = height - 1;
    
    int idx = (y * width + x) * 3;
    return (frame[idx] << 16) | (frame[idx + 1] << 8) | frame[idx + 2];
}

// Bicubic interpolation for a single pixel
uint32_t bicubic_interpolate(unsigned char* frame, float x, float y, int width, int height) {
    int x1 = (int)x;
    int y1 = (int)y;
    float fx = x - x1;
    float fy = y - y1;

    // Get 4x4 patch of pixels
    uint32_t p[4][4];
    for (int dy = -1; dy <= 2; dy++) {
        for (int dx = -1; dx <= 2; dx++) {
            p[dy+1][dx+1] = get_pixel_rgb(frame, x1 + dx, y1 + dy, width, height);
        }
    }

    // Interpolate each color channel
    float r[4], g[4], b[4];
    
    // Interpolate horizontally first
    for (int i = 0; i < 4; i++) {
        r[i] = cubic_hermite(
            (p[i][0] >> 16) & 0xFF,
            (p[i][1] >> 16) & 0xFF,
            (p[i][2] >> 16) & 0xFF,
            (p[i][3] >> 16) & 0xFF,
            fx
        );
        g[i] = cubic_hermite(
            (p[i][0] >> 8) & 0xFF,
            (p[i][1] >> 8) & 0xFF,
            (p[i][2] >> 8) & 0xFF,
            (p[i][3] >> 8) & 0xFF,
            fx
        );
        b[i] = cubic_hermite(
            p[i][0] & 0xFF,
            p[i][1] & 0xFF,
            p[i][2] & 0xFF,
            p[i][3] & 0xFF,
            fx
        );
    }

    // Interpolate vertically
    int ri = (int)(cubic_hermite(r[0], r[1], r[2], r[3], fy) + 0.5f);
    int gi = (int)(cubic_hermite(g[0], g[1], g[2], g[3], fy) + 0.5f);
    int bi = (int)(cubic_hermite(b[0], b[1], b[2], b[3], fy) + 0.5f);

    // Clamp results
    ri = ri < 0 ? 0 : (ri > 255 ? 255 : ri);
    gi = gi < 0 ? 0 : (gi > 255 ? 255 : gi);
    bi = bi < 0 ? 0 : (bi > 255 ? 255 : bi);

    return (0xFF << 24) | (ri << 16) | (gi << 8) | bi;
}

void save_scene(Scene* scene, const char* filename) {
    // Calculate scaled dimensions
    int scaled_width = (int)(scene->width / scene->scale_factor + 0.5f);
    int scaled_height = (int)(scene->height / scene->scale_factor + 0.5f);

    // Prepare WebP animation configuration
    WebPAnimEncoderOptions anim_config;
    WebPAnimEncoderOptionsInit(&anim_config);
    WebPAnimEncoder* enc = WebPAnimEncoderNew(scaled_width, scaled_height, &anim_config);
    
    // Configure each frame
    WebPConfig config;
    WebPConfigInit(&config);
    config.image_hint = WEBP_HINT_GRAPH;
    
    // Prepare picture with scaled dimensions
    WebPPicture pic;
    WebPPictureInit(&pic);
    pic.width = scaled_width;
    pic.height = scaled_height;
    pic.use_argb = 1;
    WebPPictureAlloc(&pic);

    // Add each frame to the animation
    for (int frame = 0; frame < scene->frame_count; frame++) {
        // Scale up the frame using bicubic interpolation
        for (int y = 0; y < scaled_height; y++) {
            for (int x = 0; x < scaled_width; x++) {
                float src_x = x * (scene->width - 1.0f) / (scaled_width - 1.0f);
                float src_y = y * (scene->height - 1.0f) / (scaled_height - 1.0f);
                
                pic.argb[y * scaled_width + x] = bicubic_interpolate(
                    scene->frames[frame],
                    src_x,
                    src_y,
                    scene->width,
                    scene->height
                );
            }
        }
        
        int timestamp = frame * (scene->duration_ms / scene->frame_count);
        WebPAnimEncoderAdd(enc, &pic, timestamp, &config);
    }
    
    // Finalize animation
    WebPAnimEncoderAdd(enc, NULL, scene->duration_ms, NULL);
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

void update_progress_bar(int frame, int total_frames, clock_t start_time) {
    printf("\r[");
    int barWidth = 30;
    int pos = barWidth * (frame + 1) / total_frames;
    
    for (int i = 0; i < barWidth; i++) {
        if (i < pos) printf("=");
        else if (i == pos) printf(">");
        else printf(" ");
    }

    float progress = (frame + 1.0f) / total_frames * 100.0f;
    float elapsed = (clock() - start_time) / (float)CLOCKS_PER_SEC;
    float estimated_total = elapsed * total_frames / (frame + 1);
    float remaining = estimated_total - elapsed;

    printf("] %.1f%% | Frame %d/%d | %.1fs elapsed | %.1fs remaining", 
        progress, frame + 1, total_frames, elapsed, remaining);
    fflush(stdout);

    if (frame == total_frames - 1) printf("\n");
}

#endif