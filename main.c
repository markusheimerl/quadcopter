#include "mesh.h"

int main() {
    const int width = 800;
    const int height = 600;
    
    Camera camera = create_camera(
        (Vec3){3.0f, 2.0f, -3.0f},
        (Vec3){0.0f, 0.0f, 0.0f},
        (Vec3){0.0f, 1.0f, 0.0f},
        60.0f
    );
    
    Mesh mesh = create_mesh("drone.obj", "drone.webp");
    if (!mesh.triangles || !mesh.texture_data) {
        fprintf(stderr, "Failed to load mesh or texture\n");
        destroy_mesh(&mesh);
        return 1;
    }

    unsigned char* pixels = malloc(width * height * 3);
    render(&mesh, &camera, pixels, width, height);
    save_webp("output.webp", pixels, width, height);

    destroy_mesh(&mesh);
    free(pixels);
    return 0;
}