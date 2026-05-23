#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#include "quad.h"
#include "scene.h"


#define DT_PHYS    (1.0 /  1000.0)
#define DT_CTRL    (1.0 /   100.0)
#define DT_RENDER  (1.0 /    24.0)
#define SIM_TIME   10.0
#define FPS        24


// Uniform random in [lo, hi].
static double rnd(double lo, double hi)
{
    return lo + (hi - lo) * (double)rand() / (double)RAND_MAX;
}

// Tait-Bryan (extrinsic XYZ: Rz * Ry * Rx) from a body->world rotation matrix.
static void euler_xyz(const double R[9], float *rx, float *ry, float *rz)
{
    *rx = (float)atan2(R[7], R[8]);   // roll  about X
    *ry = (float)asin(-R[6]);         // pitch about Y
    *rz = (float)atan2(R[3], R[0]);   // yaw   about Z
}


int main(void)
{
    srand((unsigned)time(NULL));

    // Random start state and goal.
    double sx   = rnd(-2.0, 2.0);
    double sy   = rnd( 0.5, 2.0);
    double sz   = rnd(-2.0, 2.0);
    double syaw = rnd(-M_PI, M_PI);

    double target[7] = {
        rnd(-2.0, 2.0),   // x
        rnd( 0.5, 2.5),   // y
        rnd(-2.0, 2.0),   // z
        0.0, 0.0, 0.0,    // desired velocity
        rnd(-M_PI, M_PI)  // yaw
    };

    printf("start  (%.2f, %.2f, %.2f) yaw %.2f\n", sx, sy, sz, syaw);
    printf("target (%.2f, %.2f, %.2f) yaw %.2f\n",
           target[0], target[1], target[2], target[6]);

    // Initialize quadrotor.
    Quad quad;
    quad_init(&quad, sx, sy, sz, syaw);

    // Initialize scene.
    Scene scene = create_scene(400, 300, (int)(SIM_TIME * 1000), FPS, 0.4f);

    set_scene_camera(&scene,
        (Vec3){ -3.0f,  3.0f, -3.0f },
        (Vec3){  0.0f,  0.0f,  0.0f },
        (Vec3){  0.0f,  1.0f,  0.0f },
        60.0f);

    set_scene_light(&scene,
        (Vec3){  1.0f,  1.0f, -1.0f },
        (Vec3){  1.4f,  1.4f,  1.4f });

    Mesh drone    = create_mesh("raytracer/assets/drone.obj",
                                "raytracer/assets/drone.webp");
    Mesh treasure = create_mesh("raytracer/assets/treasure.obj",
                                "raytracer/assets/treasure.webp");
    Mesh ground   = create_mesh("raytracer/assets/ground.obj",
                                "raytracer/assets/ground.webp");

    add_mesh_to_scene(&scene, drone);
    add_mesh_to_scene(&scene, treasure);
    add_mesh_to_scene(&scene, ground);

    set_mesh_position(&scene.meshes[1],
        (Vec3){ (float)target[0], (float)target[1], (float)target[2] });

    // Main loop: physics @ 1 kHz, control @ 100 Hz, render @ 24 fps.
    double omega_cmd[4];
    for (int i = 0; i < 4; i++) {
        omega_cmd[i] = quad.omega[i];
    }

    int n_steps        = (int)(SIM_TIME / DT_PHYS);
    int steps_per_ctrl = (int)(DT_CTRL   / DT_PHYS + 0.5);
    int steps_per_frm  = (int)(DT_RENDER / DT_PHYS + 0.5);
    int total_frames   = (int)(SIM_TIME * FPS);

    clock_t t_start = clock();
    for (int k = 0; k < n_steps; k++) {

        // Control update.
        if (k % steps_per_ctrl == 0) {
            quad_control(&quad, target, omega_cmd);
        }

        // Physics update.
        quad_step(&quad, omega_cmd, DT_PHYS);

        // Render update.
        if (k % steps_per_frm == 0) {
            set_mesh_position(&scene.meshes[0],
                (Vec3){ (float)quad.p[0],
                        (float)quad.p[1],
                        (float)quad.p[2] });

            float rx, ry, rz;
            euler_xyz(quad.R, &rx, &ry, &rz);
            set_mesh_rotation(&scene.meshes[0], (Vec3){ rx, ry, rz });

            render_scene(&scene);
            next_frame(&scene);
            update_progress_bar(k / steps_per_frm, total_frames, t_start);
        }
    }

    // Summary and save.
    double dx = quad.p[0] - target[0];
    double dy = quad.p[1] - target[1];
    double dz = quad.p[2] - target[2];
    double pos_err = sqrt(dx*dx + dy*dy + dz*dz);

    printf("\nfinal  (%.2f, %.2f, %.2f)   pos err %.3f m\n",
           quad.p[0], quad.p[1], quad.p[2], pos_err);

    char fname[64];
    time_t now = time(NULL);
    strftime(fname, sizeof fname, "%Y%m%d_%H%M%S_flight.webp", localtime(&now));
    save_scene(&scene, fname);
    printf("saved %s\n", fname);

    // Cleanup.
    destroy_mesh(&drone);
    destroy_mesh(&treasure);
    destroy_mesh(&ground);
    destroy_scene(&scene);
    return 0;
}
