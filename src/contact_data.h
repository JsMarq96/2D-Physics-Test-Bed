#ifndef CONTACT_DATA_H_
#define CONTACT_DATA_H_

#include "math.h"
#include "vector.h"

#define MAX_COL_POINTS 10

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

    float restitution = 0.0f;
    float bias = 0.0f;

    float prev_normal_impulse = 0.0f;
    float prev_tangent_impulses[2] = {0.0f, 0.0f};

    sVector3 tangents[2] = {};
    float tangental_angular_mass[2] = {};
};

struct sCollisionManifold {
    uint8_t   obj1;
    uint8_t   obj2;

    sVector3 normal;

    sVector3 contact_points[MAX_COL_POINTS] = {};
    float    contact_depth [MAX_COL_POINTS] = {};
    sContactData contact_data[MAX_COL_POINTS] = {};
    int contanct_points_count = 0;

    inline void add_contact_point(const sVector3 &point,
                                  const float depth) {

    }
};


#endif // CONTACT_DATA_H_
