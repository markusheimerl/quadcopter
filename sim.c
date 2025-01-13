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
#define TARGET_RADIUS 2.0
#define MIN_HEIGHT 0.1
#define MAX_YAW (2.0 * M_PI)
#define MAX_SIMULATION_TIME 10.0  // Maximum time per target in seconds
#define MAX_TOTAL_TIME 30.0      // Maximum total simulation time in seconds

bool target_reached(Quad* quad, double* target, double yaw_target) {
    double pos_error = 0;
    for(int i = 0; i < 3; i++) {
        pos_error += pow(quad->linear_position_W[i] - target[i], 2);
    }
    pos_error = sqrt(pos_error);
    
    bool velocity_stable = true;
    for(int i = 0; i < 3; i++) {
        if(fabs(quad->angular_velocity_B[i]) > 0.1) {
            velocity_stable = false;
            break;
        }
    }
    
    return (pos_error < 0.1 && velocity_stable);
}

int main() {
    Sim* sim = init_sim(true);
    
    double t_physics = 0.0;
    double t_control = 0.0;
    double t_render = 0.0;
    double target_start_time = 0.0;
    
    // Generate random targets
    double targets[NUM_TARGETS][3];
    double yaw_targets[NUM_TARGETS];
    srand(time(NULL));
    
    for(int i = 0; i < NUM_TARGETS; i++) {
        double theta = ((double)rand()/RAND_MAX) * 2.0 * M_PI;
        double r = ((double)rand()/RAND_MAX) * TARGET_RADIUS;
        targets[i][0] = r * cos(theta);
        targets[i][2] = r * sin(theta);
        targets[i][1] = MIN_HEIGHT + ((double)rand()/RAND_MAX) * TARGET_RADIUS;
        yaw_targets[i] = ((double)rand()/RAND_MAX) * MAX_YAW;
        
        printf("Target %d: [%.2f, %.2f, %.2f], yaw: %.2f\n", 
               i, targets[i][0], targets[i][1], targets[i][2], yaw_targets[i]);
    }
    
    int current_target = 0;
    double control_input[7] = {0};
    
    while(current_target < NUM_TARGETS) {
        // Safety checks
        if(t_physics >= MAX_TOTAL_TIME) {
            printf("Maximum total simulation time reached. Ending simulation.\n");
            break;
        }
        
        if(t_physics - target_start_time >= MAX_SIMULATION_TIME) {
            printf("Maximum time for target %d reached. Moving to next target.\n", current_target);
            current_target++;
            target_start_time = t_physics;
            continue;
        }
        
        update_quad(sim->quad, DT_PHYSICS);
        t_physics += DT_PHYSICS;
        
        if(t_control <= t_physics) {
            for(int i = 0; i < 3; i++) {
                control_input[i] = targets[current_target][i];
                control_input[i+3] = 0.0;
            }
            control_input[6] = yaw_targets[current_target];
            
            control_quad(sim->quad, control_input);
            t_control += DT_CONTROL;
            
            // Print current position every second
            if(fmod(t_physics, 1.0) < DT_CONTROL) {
                printf("Time: %.1fs, Position: [%.2f, %.2f, %.2f], Target: %d\n",
                       t_physics,
                       sim->quad->linear_position_W[0],
                       sim->quad->linear_position_W[1],
                       sim->quad->linear_position_W[2],
                       current_target);
            }
            
            if(target_reached(sim->quad, targets[current_target], yaw_targets[current_target])) {
                printf("Target %d reached at time %.2f!\n", current_target, t_physics);
                current_target++;
                target_start_time = t_physics;
            }
        }
        
        if(t_render <= t_physics) {
            render_sim(sim);
            t_render += DT_RENDER;
        }
    }
    
    printf("Simulation completed after %.2f seconds.\n", t_physics);
    free_sim(sim);
    return 0;
}