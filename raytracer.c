#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <webp/decode.h>
#include <webp/encode.h>

// Basic types
typedef struct { float x, y, z; } Vec3;
typedef struct { float u, v; } Vec2;
typedef struct { Vec3 origin, direction; } Ray;
typedef struct {
    Vec3 v0, v1, v2;      // Vertices
    Vec2 t0, t1, t2;      // Texture coordinates
} Triangle;
typedef struct {
    Triangle* triangles;
    size_t triangle_count;
} Mesh;
typedef struct {
    unsigned char* data;
    int width, height;
} Texture;

// Vector operations
Vec3 vec3_add(Vec3 a, Vec3 b) { return (Vec3){a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 vec3_sub(Vec3 a, Vec3 b) { return (Vec3){a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 vec3_mul(Vec3 a, float t) { return (Vec3){a.x * t, a.y * t, a.z * t}; }
Vec3 vec3_div(Vec3 a, float t) { return vec3_mul(a, 1.0f/t); }
float vec3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 vec3_cross(Vec3 a, Vec3 b) {
    return (Vec3){
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}
float vec3_length(Vec3 v) { return sqrtf(vec3_dot(v, v)); }
Vec3 vec3_normalize(Vec3 v) { return vec3_div(v, vec3_length(v)); }

// Ray-triangle intersection using Möller–Trumbore algorithm
bool ray_triangle_intersect(Ray ray, Triangle triangle, float* t, float* u_out, float* v_out) {
    const float EPSILON = 0.0000001f;
    Vec3 edge1 = vec3_sub(triangle.v1, triangle.v0);
    Vec3 edge2 = vec3_sub(triangle.v2, triangle.v0);
    Vec3 h = vec3_cross(ray.direction, edge2);
    float a = vec3_dot(edge1, h);

    if (a > -EPSILON && a < EPSILON) return false;

    float f = 1.0f / a;
    Vec3 s = vec3_sub(ray.origin, triangle.v0);
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

// Load texture from webp
Texture load_texture(const char* filename) {
    Texture tex = {NULL, 0, 0};
    FILE* fp = fopen(filename, "rb");
    if (!fp) return tex;

    // Get file size
    fseek(fp, 0, SEEK_END);
    size_t file_size = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    // Read file data
    uint8_t* file_data = malloc(file_size);
    if (fread(file_data, 1, file_size, fp) != file_size) {
        free(file_data);
        fclose(fp);
        return tex;
    }
    fclose(fp);

    // Get WebP image features
    WebPBitstreamFeatures features;
    if (WebPGetFeatures(file_data, file_size, &features) != VP8_STATUS_OK) {
        free(file_data);
        return tex;
    }

    // Decode WebP
    tex.width = features.width;
    tex.height = features.height;
    tex.data = WebPDecodeRGBA(file_data, file_size, &tex.width, &tex.height);

    free(file_data);
    return tex;
}

// Sample texture at UV coordinates
Vec3 sample_texture(const Texture* tex, float u, float v) {
    u = u - floorf(u);
    v = v - floorf(v);
    int x = (int)(u * (tex->width - 1));
    int y = (int)(v * (tex->height - 1));
    int idx = (y * tex->width + x) * 4;
    return (Vec3){
        tex->data[idx] / 255.0f,
        tex->data[idx + 1] / 255.0f,
        tex->data[idx + 2] / 255.0f
    };
}

// Load OBJ file
Mesh load_obj(const char* filename) {
    Vec3* vertices = malloc(1000000 * sizeof(Vec3));
    Vec2* texcoords = malloc(1000000 * sizeof(Vec2));
    int vertex_count = 0, texcoord_count = 0, triangle_count = 0;
    Mesh mesh = {malloc(1000000 * sizeof(Triangle)), 0};

    FILE* file = fopen(filename, "r");
    if (!file) { fprintf(stderr, "Failed to open %s\n", filename); exit(1); }

    char line[256];
    while (fgets(line, sizeof(line), file)) {
        if (line[0] == 'v' && line[1] == ' ') {
            sscanf(line + 2, "%f %f %f", 
                &vertices[vertex_count].x,
                &vertices[vertex_count].y,
                &vertices[vertex_count].z);
            vertex_count++;
        } else if (line[0] == 'v' && line[1] == 't') {
            sscanf(line + 3, "%f %f",
                &texcoords[texcoord_count].u,
                &texcoords[texcoord_count].v);
            texcoord_count++;
        } else if (line[0] == 'f') {
            int v1, v2, v3, t1, t2, t3;
            sscanf(line + 2, "%d/%d/%*d %d/%d/%*d %d/%d/%*d",
                &v1, &t1, &v2, &t2, &v3, &t3);
            mesh.triangles[triangle_count].v0 = vertices[v1-1];
            mesh.triangles[triangle_count].v1 = vertices[v2-1];
            mesh.triangles[triangle_count].v2 = vertices[v3-1];
            mesh.triangles[triangle_count].t0 = texcoords[t1-1];
            mesh.triangles[triangle_count].t1 = texcoords[t2-1];
            mesh.triangles[triangle_count].t2 = texcoords[t3-1];
            triangle_count++;
        }
    }
    mesh.triangle_count = triangle_count;
    printf("Loaded %d vertices, %d texcoords, %d triangles\n", 
           vertex_count, texcoord_count, triangle_count);

    free(vertices);
    free(texcoords);
    fclose(file);
    return mesh;
}

void save_webp(const char* filename, unsigned char* pixels, int width, int height) {
    // Convert RGB to RGBA (WebP encoder expects RGBA)
    uint8_t* rgba = malloc(width * height * 4);
    for (int i = 0; i < width * height; i++) {
        rgba[i * 4] = pixels[i * 3];     // R
        rgba[i * 4 + 1] = pixels[i * 3 + 1]; // G
        rgba[i * 4 + 2] = pixels[i * 3 + 2]; // B
        rgba[i * 4 + 3] = 255;           // A
    }

    // Encode WebP
    uint8_t* output;
    size_t output_size = WebPEncodeRGBA(rgba, width, height, width * 4, 75, &output);

    if (output_size > 0) {
        FILE* fp = fopen(filename, "wb");
        if (fp) {
            fwrite(output, output_size, 1, fp);
            fclose(fp);
        }
        WebPFree(output);
    }

    free(rgba);
}

// Render scene
void render(const Mesh* mesh, const Texture* texture, unsigned char* pixels, int width, int height) {
    // Camera parameters
    Vec3 camera_pos = {3.0f, 2.0f, -3.0f};
    Vec3 look_at = {0.0f, 0.0f, 0.0f};
    Vec3 up = {0.0f, 1.0f, 0.0f};
    
    // Calculate camera coordinate system
    Vec3 forward = vec3_normalize(vec3_sub(look_at, camera_pos));
    Vec3 right = vec3_normalize(vec3_cross(forward, up));
    Vec3 camera_up = vec3_cross(right, forward);
    
    float fov = 60.0f;
    float scale = tanf((fov * 0.5f) * M_PI / 180.0f);
    float aspect = (float)width / height;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float ray_x = (2.0f * ((x + 0.5f) / width) - 1.0f) * aspect * scale;
            float ray_y = (1.0f - 2.0f * ((y + 0.5f) / height)) * scale;

            // Calculate ray direction using camera basis vectors
            Vec3 ray_dir = vec3_normalize(vec3_add(
                vec3_add(
                    vec3_mul(right, ray_x),
                    vec3_mul(camera_up, ray_y)
                ),
                forward
            ));

            Ray ray = {camera_pos, ray_dir};
            float closest_t = INFINITY;
            bool hit = false;
            Vec2 hit_uv = {0, 0};

            for (size_t i = 0; i < mesh->triangle_count; i++) {
                float t, u, v;
                if (ray_triangle_intersect(ray, mesh->triangles[i], &t, &u, &v) && t < closest_t) {
                    closest_t = t;
                    hit = true;
                    float w = 1.0f - u - v;
                    hit_uv.u = w * mesh->triangles[i].t0.u + 
                              u * mesh->triangles[i].t1.u + 
                              v * mesh->triangles[i].t2.u;
                    hit_uv.v = w * mesh->triangles[i].t0.v + 
                              u * mesh->triangles[i].t1.v + 
                              v * mesh->triangles[i].t2.v;
                }
            }

            int idx = (y * width + x) * 3;
            if (hit) {
                Vec3 color = sample_texture(texture, hit_uv.u, hit_uv.v);
                pixels[idx] = (unsigned char)(color.x * 255.0f);
                pixels[idx + 1] = (unsigned char)(color.y * 255.0f);
                pixels[idx + 2] = (unsigned char)(color.z * 255.0f);
            } else {
                pixels[idx] = pixels[idx + 1] = pixels[idx + 2] = 50;
            }
        }
    }
}

int main() {
    const int width = 800;
    const int height = 600;
    
    Mesh mesh = load_obj("drone.obj");
    Texture texture = load_texture("drone.webp");
    if (!texture.data) {
        fprintf(stderr, "Failed to load texture\n");
        free(mesh.triangles);
        return 1;
    }

    unsigned char* pixels = malloc(width * height * 3);
    render(&mesh, &texture, pixels, width, height);
    save_webp("output.webp", pixels, width, height);

    free(mesh.triangles);
    WebPFree(texture.data);
    free(pixels);
    return 0;
}