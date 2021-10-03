#ifndef _PHYSICS_H_
#define _PHYSICS_H_

#include "math.h"
#include "math/dynamic_matrix.h"
#include "types.h"
#include "collision_types.h"
#include "collision_testing.h"

#include "phys_arbiter.h"
#include "phys_parameters.h"

#include <cstdint>

#include "imgui/imgui.h"

#define INSTANCE_SIZE 4

struct sPhysicsWorld {
  // Shared transforms
  sTransform  *transforms;

  sPhysArbiter arbiter = {};

  float     mass                  [INSTANCE_SIZE] = {0.0f};
  float     friction              [INSTANCE_SIZE] = {0.0f};
  float     restitution           [INSTANCE_SIZE] = {0.0f};
  bool      is_static             [INSTANCE_SIZE] = {false};
  sVector3  mass_center           [INSTANCE_SIZE] = {{0.0f}};
  sVector3  speed                 [INSTANCE_SIZE] = {{}};
  sVector3  angular_speed         [INSTANCE_SIZE] = {{}};
  sMat33    inv_inertia_tensors   [INSTANCE_SIZE] = {};

  void config_simulation();


  void step(const double elapsed_time, const sMat44 *prpj_mat);


  void generate_inertia_tensors();

  void apply_gravity(const double elapsed_time);

  // Integrate the speeds to the position & rotation
  void integrate (const double elapsed_time);

  void pre_step(const double elapsed_time) {
    // Empty.. for now
    // TODO: Prestep calculation for the non iterating parts of gauss seidel
  }

  void step(double elapsed_time) {
    // Get the number of arbiters used in each frame
    // TODO: This could be optimized by adding while checking the boradphase
    int collision_count = 0;
    for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
      if (!arbiter.used_in_frame[i]) {
        continue;
      }
      collision_count++;
    }

    sDynMatrix* jmj_mats = (sDynMatrix*) malloc(sizeof(sDynMatrix) * collision_count);
    uUIntTuple* mats_ids = (uUIntTuple*) malloc(sizeof(uUIntTuple) * collision_count);
    uint16_t mats_insert_id = 0;

    for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
      if (!arbiter.used_in_frame[i]) {
        continue;
      }
      uint16_t ref_id = arbiter.reference_ids[i];
      uint16_t inc_id = arbiter.incident_ids[i];

      // Store the ids
      mats_ids[mats_insert_id] = {ref_id, inc_id};

      // Get the mass centers in worldspace
      sVector3 mass_center_ref = {}, mass_center_inc = {};

      mass_center_ref = transforms[ref_id].apply(mass_center[ref_id]);
      mass_center_inc = transforms[inc_id].apply(mass_center[inc_id]);

      /**/
      sPhysContactData *contact = arbiter.contact_data[i];
      sVector3 normal = arbiter.separating_axis[i];


      for(int j = 0; j < arbiter.contact_size[i]; j++) {
        sVector3 r1 = contact[j].contanct_point.subs(mass_center_ref);
        sVector3 r2 = contact[j].contanct_point.subs(mass_center_inc);

        sVector3 ref_contact_cross_normal = cross_prod(r1,
                                                       normal);
        sVector3 inc_contact_cross_normal = cross_prod(r2,
                                                       normal);
      }
    }
  }

  inline void apply_impulse(const int       index,
                          const sVector3 &position,
                          const sVector3 &impulse) {
    sVector3 tmp = speed[index];
    float inv_mass = (is_static[index]) ? 0.0f : 1.0f / mass[index];
    tmp.x += inv_mass * impulse.x;
    tmp.y += inv_mass * impulse.y;
    tmp.z += inv_mass * impulse.z;

    speed[index] = tmp;

    angular_speed[index] = angular_speed[index].sum(inv_inertia_tensors[index].multiply(cross_prod(position, impulse)));
  }
};

#endif
