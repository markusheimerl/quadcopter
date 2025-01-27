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
    Mesh drone = create_mesh("drone.obj", "drone.webp");
    set_mesh_position(&drone, (Vec3){0.0f, 1.0f, 0.0f});  // Raise drone up by 1 unit
    add_mesh_to_scene(&scene, drone);
    
    Mesh treasure = create_mesh("treasure.obj", "treasure.webp");
    set_mesh_position(&treasure, (Vec3){1.0f, 0.0f, 1.0f});  // Move treasure to the right and forward
    add_mesh_to_scene(&scene, treasure);
    
    Mesh ground = create_mesh("ground.obj", "ground.webp");
    set_mesh_position(&ground, (Vec3){0.0f, 0.0f, 0.0f});  // Keep ground at origin
    add_mesh_to_scene(&scene, ground);

    // Render and save
    render_scene(&scene);
    save_scene(&scene, "output.webp");

    // Cleanup
    destroy_scene(&scene);
    return 0;
}