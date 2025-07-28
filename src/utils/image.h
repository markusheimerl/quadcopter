#ifndef IMAGE_H
#define IMAGE_H

#include <stdint.h>

// Image processing functions
float cubic_hermite(float A, float B, float C, float D, float t);
uint32_t get_pixel_rgb(unsigned char* frame, int x, int y, int width, int height);
uint32_t bicubic_interpolate(unsigned char* frame, float x, float y, int width, int height);

#endif