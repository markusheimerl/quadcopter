#include "progress.h"
#include <stdio.h>

void update_progress_bar(int frame, int total_frames, clock_t start_time) {
    printf("\r[");
    int barWidth = 30;
    int pos = barWidth * (frame + 1) / total_frames;
    
    for (int i = 0; i < barWidth; i++) {
        if (i < pos) printf("=");
        else if (i == pos) printf(">");
        else printf(" ");
    }

    float progress = (frame + 1.0f) / total_frames * 100.0f;
    float elapsed = (clock() - start_time) / (float)CLOCKS_PER_SEC;
    float estimated_total = elapsed * total_frames / (frame + 1);
    float remaining = estimated_total - elapsed;

    printf("] %.1f%% | Frame %d/%d | %.1fs elapsed | %.1fs remaining", 
        progress, frame + 1, total_frames, elapsed, remaining);
    fflush(stdout);

    if (frame == total_frames - 1) printf("\n");
}