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

  sMat33    rot_inv_inertia_tensors   [INSTANCE_SIZE] = {};


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
          // FRICTION
        // Precalculate JM^-1J for the friction constraint

        sVector3 r1 = contact[j].contanct_point.subs(mass_center_ref);
        sVector3 r2 = contact[j].contanct_point.subs(mass_center_inc);

        sVector3 r1_cross_n = cross_prod(r1, normal);
        sVector3 r2_cross_n = cross_prod(r2, normal);

        // Calculate mass
        float linear_mass = ((mass[ref_id] == 0.0f) ? 0.0f : 1.0f / mass[ref_id]) + ((mass[inc_id] == 0.0f) ? 0.0f : 1.0f / mass[inc_id]);
        float angular_mass = dot_prod(inv_inertia_tensors[ref_id].multiply(r1_cross_n).sum(inv_inertia_tensors[inc_id].multiply(r2_cross_n)), normal);
        //float angular_mass = dot_prod(rot_inv_inertia_tensors[ref_id].multiply(r1_cross_n).sum(rot_inv_inertia_tensors[inc_id].multiply(r2_cross_n)), normal);
        //float angular_mass = dot_prod(r1_cross_n, rot_inv_inertia_tensors[ref_id].multiply(r1_cross_n)) + dot_prod(r2_cross_n, rot_inv_inertia_tensors[inc_id].multiply(r2_cross_n));
        //float angular_mass = dot_prod(r1_cross_n, inv_inertia_tensors[ref_id].multiply(r1_cross_n)) + dot_prod(r2_cross_n, inv_inertia_tensors[inc_id].multiply(r2_cross_n));
        float constraint_mass = (linear_mass + angular_mass);
        contact->normal_mass = (constraint_mass > 0.0f) ? 1.0f / constraint_mass : 0.0f;
        //contact->normal_mass = constraint_mass;

        // Calculate bias for the Baumgarte stabilization
        contact->impulse_bias = (-BAUMGARTE_TERM / elapsed_time) * MAX(0.0f, -contact->distance - PENETRATION_SLOP);

        // WARM START
        // Generate normal impulse from prev iteration
        sVector3 warm_normal_impulse = normal.mult(contact->normal_impulse);

        //apply_impulse(ref_id, r1, warm_normal_impulse.invert());
        //apply_impulse(inc_id, r2, warm_normal_impulse);
        // Restutitution
        // if the diference in speed among the normal is bigger than > -1.0 apply restitution ??
      }
    }
  }

  // TODO: transformar matriz de inercia a coordenadas de mundo
  // Solo rotacion..? Al ser para velocidades devera de estar bien

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

      sVector3 normal = arbiter.separating_axis[i];
      sPhysContactData *contact = arbiter.contact_data[i];
      //ImGui::Text("Contact points %d", arbiter.contact_size[i]);
      for(int j = 0; j < arbiter.contact_size[i]; j++) {
        sVector3 r1 = contact[j].contanct_point.subs(mass_center_ref);
        sVector3 r2 = contact[j].contanct_point.subs(mass_center_inc);

        // Compute top part xD
        sVector3 linear_velocity_1 = speed[ref_id].sum(cross_prod(angular_speed[ref_id], r1));
        sVector3 linear_velocity_2 = speed[inc_id].sum(cross_prod(angular_speed[inc_id], r2));
        sVector3 relative_velocity = linear_velocity_2.subs(linear_velocity_1);

        relative_velocity = speed[inc_id].sum(cross_prod(angular_speed[inc_id], r2)).subs(speed[ref_id]).subs(cross_prod(angular_speed[ref_id], r1));
        //sVector3 relative_velocity = speed[inc_id].subs(speed[ref_id]);
        float projection = dot_prod(relative_velocity, normal);

        // If is > 0.0 the velocity is separating the objects and the constraint is solved
        // Impulse
        float lambda = (-projection) * contact->normal_mass;
        //float lambda = (-projection + contact->impulse_bias) * contact->normal_mass;

        lambda = MAX(lambda, 0.0f);
        // Clamp the impulse
        //float old_impulse = contact->normal_impulse;
        //contact->normal_impulse = MAX(old_impulse + lambda, 0.0f);
        //lambda = contact->normal_impulse - old_impulse;

        sVector3 impulse = normal.mult(lambda);

        apply_impulse(ref_id, r1, impulse.invert());
        apply_impulse(inc_id, r2, impulse);
        //apply_impulse(ref_id, r1, impulse);
        //apply_impulse(inc_id, r2, impulse.invert());
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

    //angular_speed[index] = angular_speed[index].sum(rot_inv_inertia_tensors[index].multiply(cross_prod(position, impulse)));
    angular_speed[index] = angular_speed[index].sum(inv_inertia_tensors[index].multiply(cross_prod(position, impulse)));
  }
};

#endif
