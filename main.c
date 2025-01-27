#include "mesh.h"
#include "raytracer.h"

int main() {
    const int width = 800;
    const int height = 600;
    
    // Create and configure camera
    Camera camera = create_camera(
        (Vec3){3.0f, 2.0f, -3.0f},  // position
        (Vec3){0.0f, 0.0f, 0.0f},   // look_at
        (Vec3){0.0f, 1.0f, 0.0f},   // up
        60.0f                        // fov
    );
    
    Mesh mesh = load_obj("drone.obj");
    Texture texture = load_texture("drone.webp");
    if (!texture.data) {
        fprintf(stderr, "Failed to load texture\n");
        free(mesh.triangles);
        return 1;
    }

    unsigned char* pixels = malloc(width * height * 3);
    render(&mesh, &texture, &camera, pixels, width, height);
    save_webp("output.webp", pixels, width, height);

    free(mesh.triangles);
    WebPFree(texture.data);
    free(pixels);
    return 0;
}