#ifndef LIGHT_H
#define LIGHT_H

#include "../math/vec3.h"

typedef struct {
    Vec3 direction;
    Vec3 color;
} DirectionalLight;

// Light operations
DirectionalLight create_directional_light(Vec3 direction, Vec3 color);

#endif