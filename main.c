#include "mesh.h"
#include "raytracer.h"

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