#ifndef CONTACT_MANAGER_H_
#define CONTACT_MANAGER_H_

//**
// Contact Manger
// For contact caching
//*/
#include "vector.h"
#include "contact_data.h"
#include <cstdint>
#include <cstring>
#include <sys/types.h>

#define MAX_CONTACT_COUNT 9
#define MAX_COLLISION_COUNT 20
#define MAX_UINT16 65535
#define EPSILON 0.000005f

inline uint16_t get_collision_id(const uint8_t id1, const uint8_t id2) {
    uint8_t min_id = 0, max_id = 0;

    if (id1 >= id2) {
        min_id = id2;
        max_id = id1;
    } else {
        min_id = id1;
        max_id = id2;
    }

    return (uint16_t) min_id | ((uint16_t) max_id << 8);
}

inline uint16_t get_contact_id() {
    return 0;
}



struct sCollision {
    uint16_t collision_id;

    sVector3 contact_normal;

    uint8_t contact_count = 0;
    sVector3 contact_point[MAX_CONTACT_COUNT];
    float    contanct_normal_impulse[MAX_CONTACT_COUNT]; // Contact constrain
    float    contanct_tang_impulse[2][MAX_CONTACT_COUNT]; // Friction constraint
    float    contact_depth[MAX_CONTACT_COUNT];
};

struct sCollisionManager {
    // For the obj ids and the collision
    uint16_t id_collision_map[MAX_UINT16] = {};
    bool id_map_in_use[MAX_UINT16] = {};

    sCollision collisions[MAX_COLLISION_COUNT];
    bool is_collision_in_use[MAX_COLLISION_COUNT];


    void renew_contacts_to_collision(const uint16_t col_id,
                                     const sVector3 *incoming_points,
                                     const float *depth_of_incomming_points,
                                     const uint8_t incoming_point_count) {

        sCollision *coll = &collisions[col_id];
        float old_contact_normal_impulse[MAX_CONTACT_COUNT];
        float old_contact_tang_impulse[2][MAX_CONTACT_COUNT];
        sVector3 old_contanct_position[MAX_CONTACT_COUNT];
        uint8_t old_contact_count = coll->contact_count;

        // Store old collision data
        memcpy(old_contact_normal_impulse, coll->contact_depth, sizeof(old_contact_normal_impulse));
        memcpy(old_contact_tang_impulse, coll->contact_depth, sizeof(old_contact_tang_impulse));
        memcpy(old_contanct_position, coll->contact_point, sizeof(old_contanct_position));

        // Copy new data to collision
        memcpy(coll->contact_depth, depth_of_incomming_points, incoming_point_count * sizeof(float));
        memcpy(coll->contact_point, incoming_points, incoming_point_count * sizeof(sVector3));
        memset(coll->contanct_normal_impulse, 0.0f, sizeof(sCollision::contanct_normal_impulse));
        memset(coll->contanct_tang_impulse, 0.0f, sizeof(sCollision::contanct_tang_impulse));
        coll->contact_count = incoming_point_count;

        // Note: O^2 complexity, but small loop size, so its doesnt really matter
        for(uint16_t j = 0; j < incoming_point_count; j++) {
            // Look for a coindicence between the old points
            for(uint16_t i = 0; i < old_contact_count; i++) {
                 // Check for extreamly close points
                if (incoming_points[j].subs(old_contanct_position[i]).magnitude() < EPSILON) {
                    // If its really close, then they are the same point,
                    // transfer the old impulse, for warmstarting
                    coll->contanct_normal_impulse[j] = old_contact_normal_impulse[i];
                    coll->contanct_tang_impulse[j][0] = old_contact_tang_impulse[i][0];
                    coll->contanct_tang_impulse[j][1] = old_contact_tang_impulse[i][1];
                    break; // Early out
                }
            }

        }

    }

    uint16_t get_collision(const uint8_t obj1,
                           const uint8_t obj2) {
        uint16_t col_id = get_collision_id(obj1, obj2);

        if (!id_map_in_use[col_id]) {
            uint16_t i = 0;
            for(; i < MAX_COLLISION_COUNT; i++) {
                if (!is_collision_in_use[i]) {
                    break;
                }
            }

            // TODO error is there cannot be any more collisions
            is_collision_in_use[i] = true;
            id_collision_map[col_id] = i;
            id_map_in_use[col_id] = true;
            return i;
        }

        return id_collision_map[col_id];

    }

    void init() {
        memset(id_map_in_use, false, 65553);
        memset(is_collision_in_use, false, MAX_COLLISION_COUNT);
    }
};

#endif // CONTACT_MANAGER_H_
