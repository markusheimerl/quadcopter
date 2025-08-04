#include "light.h"

DirectionalLight create_directional_light(Vec3 direction, Vec3 color) {
    return (DirectionalLight){vec3_normalize(direction), color};
}