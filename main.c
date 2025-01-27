#include "scene.h"

int main() {
    // Create scene
    Scene scene = create_scene(800, 600);
    
    // Set up camera
    set_scene_camera(&scene,
        (Vec3){-3.0f, 3.0f, -3.0f},
        (Vec3){0.0f, 0.0f, 0.0f},
        (Vec3){0.0f, 1.0f, 0.0f},
        60.0f
    );
    
    // Set up light
    set_scene_light(&scene,
        (Vec3){1.0f, 1.0f, -1.0f},     // Direction
        (Vec3){1.0f, 1.0f, 1.0f}       // White light
    );
    
    // Add meshes to scene
    add_mesh_to_scene(&scene, create_mesh("drone.obj", "drone.webp"));
    add_mesh_to_scene(&scene, create_mesh("treasure.obj", "treasure.webp"));
    add_mesh_to_scene(&scene, create_mesh("ground.obj", "ground.webp"));
    
    // Render and save
    render_scene(&scene);
    save_scene(&scene, "output.webp");

    // Cleanup
    destroy_scene(&scene);
    return 0;
}