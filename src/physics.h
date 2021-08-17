#ifndef _PHYSICS_H_
#define _PHYSICS_H_

#include "math.h"
#include "types.h"
#include "collision_types.h"

#define INSTANCE_SIZE 4

struct sPhysicsWorld {
  sTransform  *transforms;

  float     mass      [INSTANCE_SIZE] = {0.0f};
  float     restitution[INSTANCE_SIZE] = {0.0f};
  bool      is_static [INSTANCE_SIZE] = {false};
  sVector3  speed     [INSTANCE_SIZE] = {{}};


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

    // Bounciness of the materials
    float col_restitution = MIN(restitution[obj1], restitution[obj2]);
   
    // The impulse force is the relative speed divided by the sum of the inverse masses
    float impulse_force = -(1.0f + col_restitution) * relative_speed_among_normal / 
                           (inv_mass1 + inv_mass2);

    sVector3 impulse = {manifold.collision_normal.x * impulse_force, manifold.collision_normal.y * impulse_force, manifold.collision_normal.z * impulse_force };
    
    add_impulse(obj1, impulse);
    add_impulse(obj2, impulse.invert());

    // Penetration correction
    // TODO: Work a bit better the solution, without moving the objects http://allenchou.net/2013/12/game-physics-constraints-sequential-impulse/

    float penetration = 0.0f;
    for(int i = 0; i < manifold.contact_point_count; i++) {
      penetration = MIN(manifold.points_depth[i], penetration);
    }

    penetration *= 0.2f / (inv_mass1 + inv_mass2);

    sVector3 correction = {penetration * manifold.collision_normal.x, penetration * manifold.collision_normal.y, penetration * manifold.collision_normal.z};

    transforms[obj1].position = transforms[obj1].position.sum(sVector3{-inv_mass1 * correction.x, -inv_mass1 * correction.y, -inv_mass1 * correction.z});
    transforms[obj2].position = transforms[obj2].position.sum(sVector3{inv_mass2 * correction.x, inv_mass2 * correction.y, inv_mass2 * correction.z});
  }
};

#endif
