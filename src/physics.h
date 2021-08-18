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
  sVector3  rotational_speed  [INSTANCE_SIZE] = {{}};
  sMat33    inv_inertia_tensors   [INSTANCE_SIZE] = {};

  // Based on https://en.wikipedia.org/wiki/List_of_moments_of_inertia 
  void generate_inertia_tensors() {
     for(int i = 0; i < INSTANCE_SIZE; i++) {
        sVector3 scale = transforms[i].scale;

        sMat33 tmp;

        tmp.sx1 = mass[i] * (scale.y * scale.y + scale.z * scale.z) / 12.0f;
        tmp.sy2 = mass[i] * (scale.x * scale.x + scale.z * scale.z) / 12.0f;
        tmp.tmp3 = mass[i] * (scale.x * scale.x + scale.y * scale.y) / 12.0f;
        tmp.invert(&inv_inertia_tensors[i]);
     }
  }


  void update(const double elapsed_time) {
    for(int i = 0; i < INSTANCE_SIZE; i++) {
      if (is_static[i]) {
        continue;
      }

      transforms[i].position.x += speed[i].x * elapsed_time;
      transforms[i].position.y += speed[i].y * elapsed_time;
      transforms[i].position.z += speed[i].z * elapsed_time;

      
    }
  }

  inline void add_impulse(const int       index,
                          const sVector3 &impulse) {
    sVector3 tmp = speed[index];
    float inv_mass = (is_static[index]) ? 0.0f : 1.0f / mass[index];
    tmp.x += inv_mass  * impulse.x;
    tmp.y += inv_mass * impulse.y;
    tmp.z += inv_mass * impulse.z;

    speed[index] = tmp;
  }

  void calculate_gravity() {
    for(int i = 0; i < INSTANCE_SIZE; i++) {
      if (is_static[i]) {
        continue;
      }

      add_impulse(i, {0.0f, -0.09f, 0.0f});
    }
  }


  void resolve_collision(const sCollisionManifold &manifold) {
    // Impulse resolution ===
    int obj1 = manifold.obj1_index, obj2 = manifold. obj2_index;
    float relative_speed_among_normal = dot_prod(manifold.collision_normal, 
                                                { speed[obj1].x - speed[obj2].x ,
                                                  speed[obj1].y - speed[obj2].y, 
                                                  speed[obj1].z - speed[obj2].z});

    float inv_mass1 = (is_static[obj1]) ? 0.0f : 1.0f / mass[obj1];
    float inv_mass2 = (is_static[obj2]) ? 0.0f : 1.0f / mass[obj2];

    // Get the mass centers in worldspace
    sVector3 mass_center_obj1, mass_center_obj2;

    mass_center_obj1 = transforms[obj1].apply(mass_center[obj1]);
    mass_center_obj2 = transforms[obj2].apply(mass_center[obj2]);

    // Bounciness of the materials
    float col_restitution = MIN(restitution[obj1], restitution[obj2]);
   
    // The impulse force is the relative speed divided by the sum of the inverse masses
    float impulse_force_common = -(1.0f + col_restitution) * relative_speed_among_normal;

    // 1/ma+1/mb+(ra×n)•([Ia]-1(ra×n))+(rb×n)•([Ib]-1(rb×n))
    // INVMAS + (ra x n) dot (Inv[InertiaA] * (ra x n)) +
    //          (rb x n) dot (Inv[InectiaB] * (rb x n))
    // Compute an impulse for each contact point
    // using the formula https://www.euclideanspace.com/physics/dynamics/collision/index.htm 
    for(int i = 0; i < manifold.contact_point_count; i++) {
      sVector3 point_center_1 = mass_center_obj1.subs(manifold.contact_points[i]);
      sVector3 point_center_2 = mass_center_obj2.subs(manifold.contact_points[i]);

      sVector3 point_normal_cross1 = cross_prod(point_center_1, manifold.collision_normal);
      sVector3 point_normal_cross2 = cross_prod(point_center_2, manifold.collision_normal);
      
      float to_divide = inv_mass1 + inv_mass2;

      to_divide += dot_prod( point_normal_cross1, inv_inertia_tensors[i].multiply(point_normal_cross1) );
      to_divide += dot_prod( point_normal_cross2, inv_inertia_tensors[i].multiply(point_normal_cross2) ); 

      float impulse_force_complete = impulse_force_common / to_divide;

      sVector3 impulse = manifold.collision_normal.mult(impulse_force_complete);

      add_impulse(obj1, impulse);
      add_impulse(obj2, impulse.invert());

      rotational_speed[obj1] = rotational_speed[obj1].subs(inv_inertia_tensors[obj1].multiply(cross_prod(point_center_1, impulse)));
      rotational_speed[obj2] = rotational_speed[obj2].sum(inv_inertia_tensors[obj2].multiply(cross_prod(point_center_2, impulse)));
    }

    // Penetration correction
    // TODO: Work a bit better the solution, without moving the objects http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/

    float penetration = 0.0f; 
    penetration *= 0.2f / (inv_mass1 + inv_mass2);

    sVector3 correction = {penetration * manifold.collision_normal.x, penetration * manifold.collision_normal.y, penetration * manifold.collision_normal.z};

    transforms[obj1].position = transforms[obj1].position.sum(sVector3{-inv_mass1 * correction.x, -inv_mass1 * correction.y, -inv_mass1 * correction.z});
    transforms[obj2].position = transforms[obj2].position.sum(sVector3{inv_mass2 * correction.x, inv_mass2 * correction.y, inv_mass2 * correction.z});
  }
};

#endif
