#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <time.h>
#include "sim.h"

#define DT_PHYSICS  (1.0 / 1000.0)
#define DT_CONTROL  (1.0 / 60.0)
#define DT_RENDER   (1.0 / 30.0)
#define NUM_TARGETS 3
#define MAX_TIME 30.0

int main() {
    Sim* sim = init_sim(true);
    double t_physics = 0.0, t_control = 0.0, t_render = 0.0;
    int current_target = 0;
    double control_input[7] = {0};
    
    // Generate targets in camera's field of view
    double targets[NUM_TARGETS][3];
    double yaw_targets[NUM_TARGETS];
    srand(time(NULL));
    
    for(int i = 0; i < NUM_TARGETS; i++) {
        targets[i][0] = ((double)rand()/RAND_MAX) * 2.0 - 1.0;    // -1 to 1
        targets[i][1] = ((double)rand()/RAND_MAX) * 1.5;          // 0 to 1.5
        targets[i][2] = ((double)rand()/RAND_MAX) * 2.0 - 1.0;    // -1 to 1
        yaw_targets[i] = ((double)rand()/RAND_MAX) * 2.0 * M_PI;
        printf("Target %d: [%.2f, %.2f, %.2f], yaw: %.2f\n", 
               i, targets[i][0], targets[i][1], targets[i][2], yaw_targets[i]);
    }
    
    while(current_target < NUM_TARGETS && t_physics < MAX_TIME) {
        update_quad(sim->quad, DT_PHYSICS);
        t_physics += DT_PHYSICS;
        
        if(t_control <= t_physics) {
            memcpy(control_input, targets[current_target], 3 * sizeof(double));
            control_input[6] = yaw_targets[current_target];
            control_quad(sim->quad, control_input);
            t_control += DT_CONTROL;
            
            // Print progress every second
            if(fmod(t_physics, 1.0) < DT_CONTROL) {
                printf("\rTime: %.1fs, Target: %d/3, Pos: [%.2f, %.2f, %.2f]",
                    t_physics, current_target + 1,
                    sim->quad->linear_position_W[0],
                    sim->quad->linear_position_W[1],
                    sim->quad->linear_position_W[2]);
                fflush(stdout);
            }
            
            // Check if target reached
            double pos_error = 0;
            for(int i = 0; i < 3; i++) {
                pos_error += pow(sim->quad->linear_position_W[i] - targets[current_target][i], 2);
            }
            if(sqrt(pos_error) < 0.1) {
                current_target++;
                printf("\nTarget %d reached at %.1fs!\n", current_target, t_physics);
            }
        }
        
        if(t_render <= t_physics) {
            render_sim(sim);
            t_render += DT_RENDER;
        }
    }
    
    printf("\nSimulation completed after %.1f seconds.\n", t_physics);
    free_sim(sim);
    return 0;
}