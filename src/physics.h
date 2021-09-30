#ifndef _PHYSICS_H_
#define _PHYSICS_H_

#include "math.h"
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
    for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
      if (!arbiter.used_in_frame[i]) {
        continue;
      }

      uint16_t ref_id = arbiter.reference_ids[i];
      uint16_t inc_id = arbiter.incident_ids[i];

      // Get the mass centers in worldspace
      sVector3 mass_center_ref = {}, mass_center_inc = {};

      mass_center_ref = transforms[ref_id].apply(mass_center[ref_id]);
      mass_center_inc = transforms[inc_id].apply(mass_center[inc_id]);


      sVector3 normal = arbiter.separating_axis[i];
      //sVector3 tangent = cross_prod(normal.invert(), cross());

      sPhysContactData *contact = arbiter.contact_data[i];
      for(int j = 0; j < arbiter.contact_size[i]; j++) {

        sVector3 r1 = contact[j].contanct_point.subs(mass_center_ref);
        sVector3 r2 = contact[j].contanct_point.subs(mass_center_inc);

        sVector3 ref_contact_cross_normal = cross_prod(r1,
                                                       normal);
        sVector3 inc_contact_cross_normal = cross_prod(r2,
                                                       normal);

        // t1 = (inv_inertia * (contact x normal)) x (contact - mass_Center)
        sVector3 t1 = cross_prod(inv_inertia_tensors[ref_id].multiply(ref_contact_cross_normal), contact[j].contanct_point.subs(mass_center_ref));
        sVector3 t2 = cross_prod(inv_inertia_tensors[inc_id].multiply(inc_contact_cross_normal), contact[j].contanct_point.subs(mass_center_inc));

        // Calculate the normal impulse mass
        /*float normal_mass = (mass[ref_id] != 0.0f) ? 1.0f / mass[ref_id] : 0.0f;
        normal_mass += (mass[inc_id] != 0.0f) ? 1.0f / mass[inc_id] : 0.0f;
        normal_mass += dot_prod(t1.sum(t2), arbiter.separating_axis[i]);*/

        float normal_mass = dot_prod(normal, normal.mult(((mass[ref_id] != 0.0f) ? 1.0f / mass[ref_id] : 0.0f) + ((mass[inc_id] != 0.0f) ? 1.0f / mass[inc_id] : 0.0f)));
        normal_mass += dot_prod(cross_prod(inv_inertia_tensors[ref_id].multiply(cross_prod(r1, normal)), r1).sum(cross_prod(inv_inertia_tensors[inc_id].multiply(cross_prod(r2, normal)), r2)), normal);

        contact[j].normal_mass = 1.0f / normal_mass;

        // Calculate the tangent impulse mass


        // Calcilate de bias impulse
        // 0.2 is teh bias factor and 0.01 is the penetration tollerance
        contact[j].impulse_bias = -BAUMGARTE_TERM * (1.0f / elapsed_time) * MAX(0.0f, -contact[j].distance - PENETRATION_SLOP);
        //contact[j].impulse_bias = -BAUMGARTE_TERM * (1.0f / elapsed_time) * -contact[j].distance;


        contact[j].restitution = MIN(restitution[ref_id], restitution[inc_id]);

        //contact[j].impulse_bias += contact[j].restitution

        // TODO: accolulate impulses..?
        sVector3 impulse = normal.mult(contact[j].normal_impulse);

        apply_impulse(ref_id, r1, impulse);
        apply_impulse(inc_id, r2, impulse.invert());

        sVector3 ref_contactd_speed = cross_prod(angular_speed[ref_id], r1).sum(speed[ref_id]);
        sVector3 inc_contactd_speed = cross_prod(angular_speed[inc_id], r2).sum(speed[inc_id]);

        // separating axis is the collision normal
        float new_relative_normal_speed = dot_prod(normal, ref_contactd_speed.subs(inc_contactd_speed));

        // If the speed is
        if (new_relative_normal_speed < -1.0f) {
          contact[i].impulse_bias += -contact[j].restitution * new_relative_normal_speed;
        }

      }
    }
  }

  void step(double elapsed_time) {
    for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
      if (!arbiter.used_in_frame[i]) {
        continue;
      }
      uint16_t ref_id = arbiter.reference_ids[i];
      uint16_t inc_id = arbiter.incident_ids[i];

      // Get the mass centers in worldspace
      sVector3 mass_center_ref = {}, mass_center_inc = {};

      mass_center_ref = transforms[ref_id].apply(mass_center[ref_id]);
      mass_center_inc = transforms[inc_id].apply(mass_center[inc_id]);

      sPhysContactData *contact = arbiter.contact_data[i];
      //ImGui::Text("Contact points %d", arbiter.contact_size[i]);
      for(int j = 0; j < arbiter.contact_size[i]; j++) {
        //ImGui::Text("Normal mass %f", contact[j].impulse_bias);

        // Compute relative velocity
        sVector3 ref_contact_center = contact[j].contanct_point.subs(mass_center_ref);
        sVector3 inc_contact_center = contact[j].contanct_point.subs(mass_center_inc);

        sVector3 ref_contactd_speed = cross_prod(angular_speed[ref_id], ref_contact_center).sum(speed[ref_id]);
        sVector3 inc_contactd_speed = cross_prod(angular_speed[inc_id], inc_contact_center).sum(speed[inc_id]);

        // separating axis is the collision normal
        float relative_normal_speed = dot_prod(arbiter.separating_axis[i], ref_contactd_speed.subs(inc_contactd_speed));

        //float force_normal_impulse = contact[j].normal_mass * -relative_normal_speed;//(-relative_normal_speed + contact[j].impulse_bias);
        float force_normal_impulse = contact[j].normal_mass * (-relative_normal_speed + contact[j].impulse_bias);

        //float force_normal_impulse =  contact[j].normal_mass * (-relative_normal_speed + contact[j].impulse_bias);
        //float force_normal_impulse =  contact[j].normal_mass * (-relative_normal_speed + contact[j].impulse_bias + (contact[j].restitution * -relative_normal_speed));

       // float force_normal_impulse = contact[j].normal_mass * (-relative_normal_speed + contact[j].impulse_bias);

        //ImGui::Text("impulse %f", force_normal_impulse);
        //force_normal_impulse = MAX(force_normal_impulse, 0.0f);

        // Accululating impulses & clampping
        float tmp_impulse = contact[j].normal_impulse;
        contact[j].normal_impulse = tmp_impulse + force_normal_impulse;

        if (contact[j].normal_impulse > 0.0f) {
          contact[j].normal_impulse = 0.0f;
        }

        force_normal_impulse = contact[j].normal_impulse - tmp_impulse;

        sVector3 normal_impulse = arbiter.separating_axis[i].mult(force_normal_impulse);

        //ImGui::Text("normal imp %f %f %f", normal_impulse.x, normal_impulse.y, normal_impulse.z);

        apply_impulse(ref_id, ref_contact_center, normal_impulse);
        apply_impulse(inc_id, inc_contact_center, normal_impulse.invert());

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
