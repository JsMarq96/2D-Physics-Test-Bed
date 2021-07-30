#ifndef _PHYSICS_H_
#define _PHYSICS_H_

#include "math.h"
#include "types.h"
#include "collision_types.h"

#define INSTANCE_SIZE 4

struct sPhysicsWorld {
  sTransform  *transforms;

  float     mass      [INSTANCE_SIZE] = {0.0f};
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
    tmp.x += 1.0f / mass[index] * impulse.x;
    tmp.y += 1.0f / mass[index] * impulse.y;
    tmp.z += 1.0f / mass[index] * impulse.z;

    speed[index] = tmp;
  }

  void calculate_gravity() {
    for(int i = 0; i < INSTANCE_SIZE; i++) {
      if (is_static[i]) {
        continue;
      }

      add_impulse(i, {0.0f, -0.0009f, 0.0f});
    }
  }
};

#endif
