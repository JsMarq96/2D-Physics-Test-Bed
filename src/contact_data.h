#ifndef CONTACT_DATA_H_
#define CONTACT_DATA_H_

#include "math.h"
#include "vector.h"
#include "constants.h"

enum eColiderTypes : uint8_t {
    SPHERE_COLLIDER = 0,
    PLANE_COLLIDER,
    CUBE_COLLIDER,
    CAPSULE_COLLIDER,
    COLLIDER_COUNT
};

struct sContactData {
    sVector3 r1 = {};
    sVector3 r2 = {};

    float linear_mass = 0.0f;
    float angular_mass = 0.0f;
    float tangental_angular_mass[2] = {};

    float restitution = 0.0f;
    float bias = 0.0f;
};

struct sCollisionManifold {
    uint8_t   obj1;
    uint8_t   obj2;

    sVector3 normal;
    sVector3 tangents[2];

    uint8_t       contact_count = 0;
    sVector3      contact_point[MAX_CONTACT_COUNT];
    float         contanct_normal_impulse[MAX_CONTACT_COUNT]; // Contact constrain
    float         contanct_tang_impulse[2][MAX_CONTACT_COUNT]; // Friction constraint
    float         contact_depth[MAX_CONTACT_COUNT];
    sContactData  precompute_data[MAX_CONTACT_COUNT];
};


#endif // CONTACT_DATA_H_
