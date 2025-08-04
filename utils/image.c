#include "image.h"

// Cubic interpolation helper function
float cubic_hermite(float A, float B, float C, float D, float t) {
    float a = -A/2.0f + (3.0f*B)/2.0f - (3.0f*C)/2.0f + D/2.0f;
    float b = A - (5.0f*B)/2.0f + 2.0f*C - D/2.0f;
    float c = -A/2.0f + C/2.0f;
    float d = B;

    return a*t*t*t + b*t*t + c*t + d;
}

// Get pixel value safely with bounds checking
uint32_t get_pixel_rgb(unsigned char* frame, int x, int y, int width, int height) {
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= width) x = width - 1;
    if (y >= height) y = height - 1;
    
    int idx = (y * width + x) * 3;
    return (frame[idx] << 16) | (frame[idx + 1] << 8) | frame[idx + 2];
}

// Bicubic interpolation for a single pixel
uint32_t bicubic_interpolate(unsigned char* frame, float x, float y, int width, int height) {
    int x1 = (int)x;
    int y1 = (int)y;
    float fx = x - x1;
    float fy = y - y1;

    // Get 4x4 patch of pixels
    uint32_t p[4][4];
    for (int dy = -1; dy <= 2; dy++) {
        for (int dx = -1; dx <= 2; dx++) {
            p[dy+1][dx+1] = get_pixel_rgb(frame, x1 + dx, y1 + dy, width, height);
        }
    }

    // Interpolate each color channel
    float r[4], g[4], b[4];
    
    // Interpolate horizontally first
    for (int i = 0; i < 4; i++) {
        r[i] = cubic_hermite(
            (p[i][0] >> 16) & 0xFF,
            (p[i][1] >> 16) & 0xFF,
            (p[i][2] >> 16) & 0xFF,
            (p[i][3] >> 16) & 0xFF,
            fx
        );
        g[i] = cubic_hermite(
            (p[i][0] >> 8) & 0xFF,
            (p[i][1] >> 8) & 0xFF,
            (p[i][2] >> 8) & 0xFF,
            (p[i][3] >> 8) & 0xFF,
            fx
        );
        b[i] = cubic_hermite(
            p[i][0] & 0xFF,
            p[i][1] & 0xFF,
            p[i][2] & 0xFF,
            p[i][3] & 0xFF,
            fx
        );
    }

    // Interpolate vertically
    int ri = (int)(cubic_hermite(r[0], r[1], r[2], r[3], fy) + 0.5f);
    int gi = (int)(cubic_hermite(g[0], g[1], g[2], g[3], fy) + 0.5f);
    int bi = (int)(cubic_hermite(b[0], b[1], b[2], b[3], fy) + 0.5f);

    // Clamp results
    ri = ri < 0 ? 0 : (ri > 255 ? 255 : ri);
    gi = gi < 0 ? 0 : (gi > 255 ? 255 : gi);
    bi = bi < 0 ? 0 : (bi > 255 ? 255 : bi);

    return (0xFF << 24) | (ri << 16) | (gi << 8) | bi;
}