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
    
    // Load both meshes
    Mesh drone_mesh = create_mesh("drone.obj", "drone.webp");
    Mesh treasure_mesh = create_mesh("treasure.obj", "treasure.webp");
    
    if (!drone_mesh.triangles || !drone_mesh.texture_data ||
        !treasure_mesh.triangles || !treasure_mesh.texture_data) {
        fprintf(stderr, "Failed to load meshes or textures\n");
        destroy_mesh(&drone_mesh);
        destroy_mesh(&treasure_mesh);
        return 1;
    }

    unsigned char* pixels = malloc(width * height * 3);
    
    // Create an array of meshes
    Mesh meshes[] = {drone_mesh, treasure_mesh};
    size_t mesh_count = sizeof(meshes) / sizeof(meshes[0]);
    
    // Render all meshes
    render_scene(meshes, mesh_count, &camera, pixels, width, height);
    save_webp("output.webp", pixels, width, height);

    // Cleanup
    destroy_mesh(&drone_mesh);
    destroy_mesh(&treasure_mesh);
    free(pixels);
    return 0;
}