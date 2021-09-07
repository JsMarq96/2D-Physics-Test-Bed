#ifndef _PHYSICS_H_
#define _PHYSICS_H_

#include "math.h"
#include "types.h"
#include "collision_types.h"

#define INSTANCE_SIZE 4

struct sPhysicsWorld {
  sTransform  *transforms;

  float     mass              [INSTANCE_SIZE] = {0.0f};
  float     restitution       [INSTANCE_SIZE] = {0.0f};
  bool      is_static         [INSTANCE_SIZE] = {false};
  sVector3  mass_center       [INSTANCE_SIZE] = {{0.0f}};
  sVector3  speed             [INSTANCE_SIZE] = {{}};
  sVector3  angular_speed  [INSTANCE_SIZE] = {{}};
  sMat33    inv_inertia_tensors   [INSTANCE_SIZE] = {};

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
        std::cout << inv_inertia_tensors[i].sx1 << " " <<  inv_inertia_tensors[i].sy2 << std::endl;
     }
  }

  void apply_gravity(const double elapsed_time) {
    for(int i = 0; i < INSTANCE_SIZE; i++) {
      if (is_static[i]) {
        continue;
      }

      // Integrate gravity
      // TODO: Gravity constatn cleanup
      speed[i].y += -0.98f * elapsed_time;
    }

  }

  void update(const double elapsed_time) {
    for(int i = 0; i < INSTANCE_SIZE; i++) {
      if (is_static[i]) {
        continue;
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

      //transforms[i].set_rotation(rot_quat);
    }
  }

  inline void add_impulse(const int       index,
                          const sVector3 &impulse) {
    sVector3 tmp = speed[index];
    float inv_mass = (is_static[index]) ? 0.0f : 1.0f / mass[index];
    tmp.x += inv_mass * impulse.x;
    tmp.y += inv_mass * impulse.y;
    tmp.z += inv_mass * impulse.z;

    //TODO: Dont forget, disabled for debugging
    //speed[index] = tmp;
  }

  void resolve_collision(const sCollisionManifold &manifold) {
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

    // 1/ma+1/mb+(ra×n)•([Ia]-1(ra×n))+(rb×n)•([Ib]-1(rb×n))
    // INVMAS + (ra x n) dot (Inv[InertiaA] * (ra x n)) +
    //          (rb x n) dot (Inv[InectiaB] * (rb x n))
    // Compute an impulse for each contact point
    // using the formula https://www.euclideanspace.com/physics/dynamics/collision/index.htm 
    // me = invM1 + invM2 + dot( r1 x n, invI1 * (r1 x n) ) + dot( r2 x n, invI2 * (r2 x n) )
    float max_depth = FLT_MAX;
    for(int i = 0; i < manifold.contact_point_count; i++) {
      sVector3 point_center_1 = manifold.contact_points[i].subs(mass_center_obj1);
      sVector3 point_center_2 = manifold.contact_points[i].subs(mass_center_obj2);
      
      sVector3 contact_speed_1 = cross_prod(angular_speed[obj1], point_center_1).sum(speed[obj1]);
      sVector3 contact_speed_2 = cross_prod(angular_speed[obj2], point_center_2).sum(speed[obj2]);

      float relative_speed_among_normal = dot_prod(manifold.collision_normal,
                                                   { contact_speed_1.x - contact_speed_2.x,
                                                    contact_speed_1.y - contact_speed_2.y, 
                                                    contact_speed_1.z - contact_speed_2.z}); 


      if (relative_speed_among_normal > 0.0f) {
        continue;
      }

      sVector3 point_normal_cross1 = cross_prod(point_center_1, manifold.collision_normal);
      sVector3 point_normal_cross2 = cross_prod(point_center_2, manifold.collision_normal);
      
      // The impulse force is the relative speed divided by the sum of the inverse masses
      float impulse_force_common = -relative_speed_among_normal;//-(1.0f + col_restitution) * relative_speed_among_normal;
      float to_divide = inv_mass1 + inv_mass2;
      
      //to_divide += dot_prod( point_center_1, inv_inertia_tensors[obj1].multiply(point_normal_cross1) );
      //to_divide += dot_prod( point_center_2, inv_inertia_tensors[obj2].multiply(point_normal_cross2) );

      sVector3 t1 = cross_prod(inv_inertia_tensors[obj1].multiply(point_normal_cross1), point_center_1);
      sVector3 t2 = cross_prod(inv_inertia_tensors[obj2].multiply(point_normal_cross2), point_center_2);
      to_divide += dot_prod(t1.sum(t2), manifold.collision_normal);

      float impulse_force_complete = impulse_force_common / to_divide;
      //impulse_force_common /= manifold.contact_point_count;

      sVector3 impulse = manifold.collision_normal.mult(impulse_force_complete);

      add_impulse(obj1, impulse);
      add_impulse(obj2, impulse.invert());

      //std::cout << inv_inertia_tensors[obj1].sx1 << std::endl;
      //sVector3 t = cross_prod(point_center_1, impulse);
      // std::cout << impulse.x << " " << impulse.y << " " << impulse.z << std::endl;
      angular_speed[obj1] = angular_speed[obj1].sum(inv_inertia_tensors[obj1].multiply(cross_prod(point_center_1, impulse)));
      angular_speed[obj2] = angular_speed[obj2].sum(inv_inertia_tensors[obj2].multiply(cross_prod(point_center_2, impulse.invert())));

      max_depth = MIN(manifold.points_depth[i], max_depth);
    }

    // Penetration correction
    // TODO: Work a bit better the solution, without moving the objects http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/
    float penetration_allowance = 0.0001f;
      float penetration = MAX(-max_depth - penetration_allowance, 0.0f) / (inv_mass1 + inv_mass2);
      //std::cout << manifold.points_depth[i] << " " << penetration << std::endl;
      penetration *= 0.3f * manifold.contact_point_count;
      sVector3 correction = {penetration * manifold.collision_normal.x, penetration * manifold.collision_normal.y, penetration * manifold.collision_normal.z};

      transforms[obj1].position = transforms[obj1].position.sum(sVector3{inv_mass1 * correction.x, inv_mass1 * correction.y, inv_mass1 * correction.z});
      transforms[obj2].position = transforms[obj2].position.sum(sVector3{-inv_mass2 * correction.x, -inv_mass2 * correction.y, -inv_mass2 * correction.z});

  }
};

#endif
