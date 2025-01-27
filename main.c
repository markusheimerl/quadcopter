#include "scene.h"

int main() {
    // Create scene with 4 seconds duration at 24 fps and a scaling factor of 0.9
    Scene scene = create_scene(800, 600, 4000, 24, 0.9f);
    
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
        (Vec3){1.4f, 1.4f, 1.4f}       // White light
    );
    
    // Add meshes to scene
    Mesh drone = create_mesh("drone.obj", "drone.webp");
    add_mesh_to_scene(&scene, drone);
    
    Mesh treasure = create_mesh("treasure.obj", "treasure.webp");
    add_mesh_to_scene(&scene, treasure);
    
    Mesh ground = create_mesh("ground.obj", "ground.webp");
    set_mesh_position(&ground, (Vec3){0.0f, 0.0f, 0.0f});
    add_mesh_to_scene(&scene, ground);

    // Initialize timer for progress bar
    clock_t start_time = clock();

    // Render each frame
    for (int frame = 0; frame < scene.frame_count; frame++) {
        float t = frame * (2.0f * M_PI / 120.0f);
        
        // Animate drone
        set_mesh_position(&scene.meshes[0], 
            (Vec3){2.0f * cosf(t), 1.0f + 0.2f * sinf(2*t), 2.0f * sinf(t)});
        set_mesh_rotation(&scene.meshes[0], 
            (Vec3){0.1f * sinf(t), t, 0.1f * cosf(t)});
        
        // Animate treasure
        set_mesh_position(&scene.meshes[1], 
            (Vec3){1.0f, 0.5f + 0.1f * sinf(t), 1.0f});
        set_mesh_rotation(&scene.meshes[1], 
            (Vec3){0, t * 0.5f, 0});
            
        // Render frame
        render_scene(&scene);
        next_frame(&scene);
        
        // Update progress bar
        update_progress_bar(frame, scene.frame_count, start_time);
    }

    // Save all frames as animated WebP
    char filename[64];
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S_rendering.webp", 
             localtime(&(time_t){time(NULL)}));
    save_scene(&scene, filename);

    // Cleanup
    destroy_scene(&scene);
    return 0;
}