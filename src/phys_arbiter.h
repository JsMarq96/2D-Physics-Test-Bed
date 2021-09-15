#ifndef PHYS_ARBITER_H_
#define PHYS_ARBITER_H_

#include "imgui/imgui.h"
#include "math.h"
#include "collision_types.h"
#include "types.h"
#include "utils.h"
#include <cstring>
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

// TODO: REWORK the whole contact point system
//  I dont like it one bit

//  Use arrays of strucutre, since we are going to use the
//  Whole object data
struct sPhysContactData {
    sVector3       contanct_point = {};
    float          normal_impulse  = 0.0f;
    float          normal_mass     = 0.0f;
    float          impulse_bias    = 0.0f;
    float          distance        = 0.0f;
    float          restitution     = 0.0f;
    uUIntTuple     point_id       = {};
};

union sFeatureDescriptor {
    struct {
        int ref_obj;
        int inc_obj;
    };
    long data = 0;
};

struct sPhysArbiter {
    // Arbiter data
    bool           enabled         [MAX_ARBITERS_SIZE] = {};
    bool           used_in_frame   [MAX_ARBITERS_SIZE] = {};
    sArbiterKey    keys            [MAX_ARBITERS_SIZE] = {};
    sVector3       separating_axis [MAX_ARBITERS_SIZE] = {};
    uint16_t       reference_ids   [MAX_ARBITERS_SIZE] = {};
    uint16_t       incident_ids    [MAX_ARBITERS_SIZE] = {};

    // Contact points data
    uint8_t          contact_size[MAX_ARBITERS_SIZE] = {0};
    sPhysContactData contact_data  [MAX_ARBITERS_SIZE][MAX_CONTANT_POINTS] = {};
    sFeatureDescriptor descriptors [MAX_ARBITERS_SIZE] = { 0 };

    void init() {
        for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
            used_in_frame[i] = false;
            enabled[i] = false;
        }
    }

    // Get the arbiter of a collision
    int get_arbiter_id(const int   obj1,
                       const int   obj2) {
        // TODO: Linear search, this could be skipped by hashing or another more efficient search
        sArbiterKey to_find;
        to_find.init(obj1, obj2);

        for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
            if (!enabled[i]) {
                continue;
            }

            if (to_find.is_equal(keys[i])) {
                used_in_frame[i] = true;
                return i;
            }
        }

        return -1;
    }

    int create_arbiter(const int obj1,
                       const int obj2) {
        for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
            if (enabled[i] || used_in_frame[i]) {
                continue;
            }

            keys[i].init(obj1, obj2);
            used_in_frame[i] = true;
            enabled[i] = true;
            return i;
        }

        return -1;
    }

    void remove_arbiter(const int id) {
        enabled[id] = false;
        used_in_frame[id] = false;
    }

    void remove_unused_arbiters() {
        for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
            if (enabled[i] && !used_in_frame[i]) {
                enabled[i] = false;
            }
            used_in_frame[i] = false;
        }
    }

    void add_manifold(const int arbiter_id,
                      const sCollisionManifold &manifold) {

        sFeatureDescriptor new_descriptor = {};
        new_descriptor.inc_obj = manifold.incident_face;
        new_descriptor.ref_obj = manifold.reference_face;

        used_in_frame[arbiter_id] = true;

        reference_ids[arbiter_id] = manifold.reference_index;
        incident_ids[arbiter_id] = manifold.incident_index;

        separating_axis[arbiter_id] = manifold.collision_normal;

        if (descriptors[arbiter_id].data == new_descriptor.data) {
            ImGui::Text("Common points");
            sPhysContactData *contact = contact_data[arbiter_id];

            sPhysContactData tmp_list[MAX_CONTANT_POINTS] = {};
            uint8_t tmp_size = 0;
            uint8_t left_to_copy = manifold.contact_point_count;
            sPhysContactData *tmp_list_it = tmp_list;

            // Rewrite the old points with the new data (if any)
            // NOTE: this probably could be O(n+m) instead of O(n*m)
            for(int i = 0; i < manifold.contact_point_count; i++) {
                tmp_list_it->distance = manifold.points_depth[i];
                tmp_list_it->contanct_point = manifold.contact_points[i];
                tmp_list_it->point_id = manifold.point_ids[i];

                for(int j = 0; j < MAX_CONTANT_POINTS; j++) {
                    if (manifold.point_ids[i].equal_to(contact[j].point_id)) {
                        tmp_list_it->normal_impulse = contact[j].normal_impulse;
                        tmp_list_it->normal_mass = contact[j].normal_mass;
                        tmp_list_it->impulse_bias = contact[j].impulse_bias;
                        break;
                    }
                }
                tmp_list_it++;
                tmp_size++;
            }
            memcpy(contact_data[arbiter_id], tmp_list, sizeof(tmp_list));
            contact_size[arbiter_id] = tmp_size;
            return;
        }
        // The descriptors does not match, so we celan and replace contact
        // points
        ImGui::Text("No common points");
        sPhysContactData *contact_it = contact_data[arbiter_id];
        for(int i = 0; i < manifold.contact_point_count; i++) {
            contact_it->distance = manifold.points_depth[i];
            contact_it->contanct_point = manifold.contact_points[i];
            contact_it->point_id = manifold.point_ids[i];

            contact_it++;
        }
        contact_size[arbiter_id] = manifold.contact_point_count;
        descriptors[arbiter_id] = new_descriptor;
    }
};

#endif // PHYS_ARBITER_H_
