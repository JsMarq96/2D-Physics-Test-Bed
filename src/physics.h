#ifndef _PHYSICS_H_
#define _PHYSICS_H_

#include "math.h"
#include "types.h"
#include "collision_types.h"
#include "collision_testing.h"

#define INSTANCE_SIZE 4

struct sPhysicsWorld {
  // Shared transforms
  sTransform  *transforms;

  float     mass                  [INSTANCE_SIZE] = {0.0f};
  float     friction              [INSTANCE_SIZE] = {0.0f};
  float     restitution           [INSTANCE_SIZE] = {0.0f};
  bool      is_static             [INSTANCE_SIZE] = {false};
  sVector3  mass_center           [INSTANCE_SIZE] = {{0.0f}};
  sVector3  speed                 [INSTANCE_SIZE] = {{}};
  sVector3  angular_speed         [INSTANCE_SIZE] = {{}};
  sMat33    inv_inertia_tensors   [INSTANCE_SIZE] = {};
  bool      is_awake              [INSTANCE_SIZE] = { false };


  void config_simulation() {
    generate_inertia_tensors();

    for(int i = 0; i < INSTANCE_SIZE; i++) {
      is_awake[i] = true;
    }
  }

  // Based on https://en.wikipedia.org/wiki/List_of_moments_of_inertia 
  void generate_inertia_tensors() {
     for(int i = 0; i < INSTANCE_SIZE; i++) {
       if (is_static[i]) {
        //inv_inertia_tensors[i].set_identity();
        inv_inertia_tensors[i].sx1 = 0.0f;
        inv_inertia_tensors[i].sy2 = 0.0f;
        inv_inertia_tensors[i].tmp3 = 0.0f;
        continue;
       }
        sVector3 scale = transforms[i].scale;

        sMat33 tmp = {};
        tmp.set_identity();

        float half_x = scale.x / 2.0f;
        float half_y = scale.y / 2.0f;
        float half_z = scale.z / 2.0f;

        tmp.sx1 = mass[i] * (half_y * half_y + half_z * half_z) / 12.0f;
        tmp.sy2 = mass[i] * (half_x * half_x + half_z * half_z) / 12.0f;
        tmp.tmp3 = mass[i] * (half_x * half_x + half_y * half_y) / 12.0f;
        tmp.invert(&inv_inertia_tensors[i]);
     }
  }

  void apply_gravity(const double elapsed_time) {
    for(int i = 0; i < INSTANCE_SIZE; i++) {
      if (is_static[i] || !is_awake[i]) {
        continue;
      }

      // Integrate gravity
      // TODO: Gravity constatn cleanup
      speed[i].y += -0.98f * elapsed_time;
    }

  }

  void update(const double elapsed_time) {
    for(int i = 0; i < INSTANCE_SIZE; i++) {
      if (is_static[i] || !is_awake[i]) {
        continue;
      }

      // Put to sleep if the speed is near to 0
      if (speed[i].magnitude() <= 0.005f) {
      //  is_awake[i] = false;
     //   continue;
      }

      // Integrate speed
      transforms[i].position.x += speed[i].x * elapsed_time;
      transforms[i].position.y += speed[i].y * elapsed_time;
      transforms[i].position.z += speed[i].z * elapsed_time;

      // Integrate angular speed
      sQuaternion4 tmp_quat = {0.0f, angular_speed[i].x, angular_speed[i].y, angular_speed[i].z};
      tmp_quat = tmp_quat.multiply(0.5f).multiply(elapsed_time);

      sQuaternion4 rot_quat = transforms[i].rotation;
      rot_quat = rot_quat.sum(tmp_quat.multiply(rot_quat));
      rot_quat = rot_quat.normalize();

      transforms[i].set_rotation(rot_quat);
    }
  }

  inline void add_impulse(const int       index,
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

  void resolve_collision(const sCollisionManifold &manifold,
                         const double             elapsed_time) {
    // Impulse resolution ===
    int obj1 = manifold.obj1_index, obj2 = manifold. obj2_index; 

    float inv_mass1 = (is_static[obj1]) ? 0.0f : 1.0f / mass[obj1];
    float inv_mass2 = (is_static[obj2]) ? 0.0f : 1.0f / mass[obj2];

    // Get the mass centers in worldspace
    sVector3 mass_center_obj1 = {}, mass_center_obj2 = {};

    mass_center_obj1 = transforms[obj1].apply(mass_center[obj1]);
    mass_center_obj2 = transforms[obj2].apply(mass_center[obj2]);

    // Bounciness of the materials
    float col_restitution = MIN(restitution[obj1], restitution[obj2]); 

    // Compute an impulse for each contact point
    // using the formula https://www.euclideanspace.com/physics/dynamics/collision/index.htm
    float max_depth = FLT_MAX;

    sVector3 normal = manifold.collision_normal;
    // Added 3 iterations
    for(int p = 0; p < 1; p++) {
    for(int i = 0; i < manifold.contact_point_count; i++) {
      sVector3 r1 = mass_center_obj1.subs(manifold.contact_points[i]);
      sVector3 r2 = mass_center_obj2.subs(manifold.contact_points[i]);

      sVector3 t1 = cross_prod(inv_inertia_tensors[obj1].multiply(cross_prod(r1, normal)), r1);
      sVector3 t2 = cross_prod(inv_inertia_tensors[obj2].multiply(cross_prod(r2, normal)), r2);


      // COmpute constraint mass
      float normal_mass = (is_static[obj1]) ? 0.0f : 1.0f / mass[obj1];
      normal_mass += (is_static[obj2]) ? 0.0f : 1.0f / mass[obj2];
      normal_mass += dot_prod(normal, t1.sum(t2));
      normal_mass = 1.0f / normal_mass;

      // Impulse bias
      float impulse_bias = -(0.10f / elapsed_time) * MAX(0.0f, -manifold.points_depth[i] - 0.001f);

      std::cout << impulse_bias << std::endl;

      sVector3 obj1_contact_speed = speed[obj1].sum(cross_prod(angular_speed[obj1], r1));
      sVector3 obj2_contact_speed = speed[obj2].sum(cross_prod(angular_speed[obj2], r2));

      float collision_relative_speed = dot_prod(normal, obj2_contact_speed.subs(obj1_contact_speed));

      float impulse_force = normal_mass * (-collision_relative_speed + impulse_bias);// + (col_restitution * -collision_relative_speed));
      //float impulse_force = normal_mass * (-dot_prod(obj2_contact_speed.subs(obj1_contact_speed), normal));

      sVector3 impulse = normal.mult(impulse_force);

      add_impulse(obj1, r1, impulse.invert());
      add_impulse(obj2, r2, impulse);
      }
    }
  }
};

#endif
