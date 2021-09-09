#ifndef PHYS_ARBITER_H_
#define PHYS_ARBITER_H_

#include "math.h"
#include "collision_types.h"
#include "types.h"

// Should be Obj num * 2
// TODO: Define a macro file
#define MAX_ARBITERS_SIZE 16

#define MAX_CONTANT_POINTS 10

struct sArbiterKey {
    int obj1 = -1;
    int obj2 = -2;

    void init(const int   i_obj1,
              const int   i_obj2) {
        if (i_obj1 < i_obj2) {
            obj1 = i_obj1;
            obj2 = i_obj2;
        } else {
            obj1 = i_obj2;
            obj2 = i_obj1;
        }
    }

    bool is_equal(const sArbiterKey &key) const {
        return obj1 == key.obj1 && obj2 == key.obj2;
    }
};

struct sPhysArbiter {
    // Arbiter data
    bool           in_use          [MAX_ARBITERS_SIZE] = {};
    sArbiterKey    keys            [MAX_ARBITERS_SIZE] = {};
    sVector3       separating_axis [MAX_ARBITERS_SIZE] = {};

    // Contact points data
    sVector3       contanct_points [MAX_ARBITERS_SIZE][MAX_CONTANT_POINTS] = {};
    float          normal_impulse  [MAX_ARBITERS_SIZE][MAX_CONTANT_POINTS] = {};
    float          normal_mass     [MAX_ARBITERS_SIZE][MAX_CONTANT_POINTS] = {};
    float          impulse_bias    [MAX_ARBITERS_SIZE][MAX_CONTANT_POINTS] = {};
    float          distance        [MAX_ARBITERS_SIZE][MAX_CONTANT_POINTS] = {};

    void init() {
        for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
            in_use[i] = false;
        }
    }

    void add_manifold(const sCollisionManifold *manifold) {}
};

#endif // PHYS_ARBITER_H_
