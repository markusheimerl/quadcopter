#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include <png.h>

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

// Load texture from PNG
Texture load_texture(const char* filename) {
    Texture tex = {NULL, 0, 0};
    FILE* fp = fopen(filename, "rb");
    if (!fp) return tex;

    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info = png_create_info_struct(png);
    png_init_io(png, fp);
    png_read_info(png, info);

    tex.width = png_get_image_width(png, info);
    tex.height = png_get_image_height(png, info);
    png_byte color_type = png_get_color_type(png, info);
    png_byte bit_depth = png_get_bit_depth(png, info);

    if (bit_depth == 16) png_set_strip_16(png);
    if (color_type == PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png);
    if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) png_set_expand_gray_1_2_4_to_8(png);
    if (png_get_valid(png, info, PNG_INFO_tRNS)) png_set_tRNS_to_alpha(png);
    if (color_type == PNG_COLOR_TYPE_RGB ||
        color_type == PNG_COLOR_TYPE_GRAY ||
        color_type == PNG_COLOR_TYPE_PALETTE) png_set_filler(png, 0xFF, PNG_FILLER_AFTER);
    if (color_type == PNG_COLOR_TYPE_GRAY ||
        color_type == PNG_COLOR_TYPE_GRAY_ALPHA) png_set_gray_to_rgb(png);

    png_read_update_info(png, info);
    tex.data = malloc(tex.height * tex.width * 4);
    png_bytep* row_pointers = malloc(sizeof(png_bytep) * tex.height);
    for(int y = 0; y < tex.height; y++) row_pointers[y] = tex.data + y * tex.width * 4;

    png_read_image(png, row_pointers);
    free(row_pointers);
    png_destroy_read_struct(&png, &info, NULL);
    fclose(fp);
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

// Save PNG file
void save_png(const char* filename, unsigned char* pixels, int width, int height) {
    FILE* fp = fopen(filename, "wb");
    if (!fp) return;

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    png_infop info = png_create_info_struct(png);
    png_init_io(png, fp);
    png_set_IHDR(png, info, width, height, 8, PNG_COLOR_TYPE_RGB,
                 PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    for (int y = 0; y < height; y++) {
        png_write_row(png, pixels + y * width * 3);
    }

    png_write_end(png, NULL);
    png_destroy_write_struct(&png, &info);
    fclose(fp);
}

// Render scene
void render(const Mesh* mesh, const Texture* texture, unsigned char* pixels, int width, int height) {
    Vec3 camera_pos = {0.0f, 0.0f, -5.0f};
    float fov = 60.0f;
    float scale = tanf((fov * 0.5f) * M_PI / 180.0f);
    float aspect = (float)width / height;

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float ray_x = (2.0f * ((x + 0.5f) / width) - 1.0f) * aspect * scale;
            float ray_y = (1.0f - 2.0f * ((y + 0.5f) / height)) * scale;

            Ray ray = {camera_pos, vec3_normalize((Vec3){ray_x, ray_y, 1.0f})};
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
    Texture texture = load_texture("drone.png");
    if (!texture.data) {
        fprintf(stderr, "Failed to load texture\n");
        free(mesh.triangles);
        return 1;
    }

    unsigned char* pixels = malloc(width * height * 3);
    render(&mesh, &texture, pixels, width, height);
    save_png("output.png", pixels, width, height);

    free(mesh.triangles);
    free(texture.data);
    free(pixels);
    return 0;
}