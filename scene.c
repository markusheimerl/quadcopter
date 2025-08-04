#include "scene.h"
#include "accel/bvh.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <pthread.h>
#include <unistd.h>

// Structure to pass data to worker threads
typedef struct {
    Scene* scene;
    unsigned char* current_frame;
    int start_row;
    int end_row;
    float aspect;
} RenderThreadData;

// Worker function for rendering a range of rows
void* render_rows(void* arg) {
    RenderThreadData* data = (RenderThreadData*)arg;
    Scene* scene = data->scene;
    unsigned char* current_frame = data->current_frame;
    float aspect = data->aspect;
    
    for (int y = data->start_row; y < data->end_row; y++) {
        for (int x = 0; x < scene->width; x++) {
            Ray ray = get_camera_ray(&scene->camera, 
                                   (x + 0.5f) / scene->width, 
                                   (y + 0.5f) / scene->height, 
                                   aspect);
            
            float closest_t = 1e30f;
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
                    float shadow_t = 1e30f;
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
    
    return NULL;
}

Scene create_scene(int width, int height, int duration_ms, int fps, float scale_factor) {
    int frame_count = (duration_ms * fps) / 1000;

    Scene scene;
    scene.meshes = NULL;
    scene.mesh_count = 0;
    scene.width = (int)(width * scale_factor);
    scene.height = (int)(height * scale_factor);
    scene.scale_factor = scale_factor;
    scene.frame_count = frame_count;
    scene.current_frame = 0;
    scene.duration_ms = duration_ms;
    scene.fps = fps;
    scene.frames = (unsigned char**)malloc(frame_count * sizeof(unsigned char*));
    
    // Allocate memory for each frame
    for (int i = 0; i < frame_count; i++) {
        scene.frames[i] = (unsigned char*)malloc(width * height * 3);
    }
    
    return scene;
}

void add_mesh_to_scene(Scene* scene, Mesh mesh) {
    scene->meshes = (Mesh*)realloc(scene->meshes, (scene->mesh_count + 1) * sizeof(Mesh));
    scene->meshes[scene->mesh_count] = mesh;
    scene->mesh_count++;
}

void set_scene_camera(Scene* scene, Vec3 position, Vec3 look_at, Vec3 up, float fov) {
    scene->camera = create_camera(position, look_at, up, fov);
}

void set_scene_light(Scene* scene, Vec3 direction, Vec3 color) {
    scene->light = create_directional_light(direction, color);
}

void next_frame(Scene* scene) {
    scene->current_frame++;
    if (scene->current_frame >= scene->frame_count) {
        scene->current_frame = scene->frame_count - 1;
    }
}

void render_scene(Scene* scene) {
    float aspect = (float)scene->width / scene->height;
    unsigned char* current_frame = scene->frames[scene->current_frame];
    
    // Determine number of threads - allow environment variable override
    int num_threads = (int)sysconf(_SC_NPROCESSORS_ONLN);
    char* thread_env = getenv("RAYTRACER_THREADS");
    if (thread_env) {
        num_threads = atoi(thread_env);
    }
    if (num_threads < 1) num_threads = 1;
    if (num_threads > scene->height) num_threads = scene->height;
    
    // Single-threaded fallback for comparison or if only 1 thread
    if (num_threads == 1) {
        for (int y = 0; y < scene->height; y++) {
            for (int x = 0; x < scene->width; x++) {
                Ray ray = get_camera_ray(&scene->camera, 
                                       (x + 0.5f) / scene->width, 
                                       (y + 0.5f) / scene->height, 
                                       aspect);
                
                float closest_t = 1e30f;
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
                        float shadow_t = 1e30f;
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
        return;
    }
    
    // Multi-threaded version
    pthread_t threads[num_threads];
    RenderThreadData thread_data[num_threads];
    
    // Calculate rows per thread
    int rows_per_thread = scene->height / num_threads;
    int remaining_rows = scene->height % num_threads;
    
    // Create and start threads
    for (int i = 0; i < num_threads; i++) {
        thread_data[i].scene = scene;
        thread_data[i].current_frame = current_frame;
        thread_data[i].aspect = aspect;
        
        // Calculate start row for this thread
        int start_row = i * rows_per_thread + (i < remaining_rows ? i : remaining_rows);
        int rows_for_this_thread = rows_per_thread + (i < remaining_rows ? 1 : 0);
        
        thread_data[i].start_row = start_row;
        thread_data[i].end_row = start_row + rows_for_this_thread;
        
        pthread_create(&threads[i], NULL, render_rows, &thread_data[i]);
    }
    
    // Wait for all threads to complete
    for (int i = 0; i < num_threads; i++) {
        pthread_join(threads[i], NULL);
    }
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