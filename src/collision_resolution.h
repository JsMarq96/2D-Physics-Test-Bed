#ifndef __COLLISION_RESOLUTION_H_
#define __COLLISION_RESOLUTION_H_

#include "math.h"
#include "collision_types.h"
#include "imgui/imgui.h"

#define MAX_INSTANCES  120

enum ePhysAttributes : unsigned char {
  IS_STATIC = 0b00001
};

struct sCollisionResolution {
    bool        enabled   [MAX_INSTANCES] = { false };
    sMat44       models   [MAX_INSTANCES] = { {} };
    unsigned char attribs [MAX_INSTANCES] = { 0b00 };
    float       mass      [MAX_INSTANCES] = { 0.0f };
    sVector3    speed     [MAX_INSTANCES] = { {0.0f, 0.0f, 0.0f} };
   
    inline void update_positions(const double time_delta) {
      for(int i = 0; i < MAX_INSTANCES; i++) {
        if (!enabled[i]){
          continue;
        }
        
        if (attribs[i] & IS_STATIC) {
          continue;
        }

        models[i].px += speed[i].x * time_delta;
        models[i].py += speed[i].y * time_delta;
        models[i].pz += speed[i].z * time_delta;

        // Gravity my guy
        speed[i] = {0.0f, -0.009f, 0.0f};
      }
    }

    inline void add_impulse(const int       index, 
                            const sVector3  impulse) {
      float inv_mass = (attribs[index] & IS_STATIC) ? 0.0f : mass[index];
      sVector3 old_speed = speed[index];
      speed[index] = {old_speed.x + (impulse.x * inv_mass), old_speed.y + (impulse.y * inv_mass), old_speed.z + (impulse.z * inv_mass)};
    }

    inline void resolve_collision(const sCollisionManifold &manifold) {
      sVector3 speed_1 = speed[manifold.obj1_index], speed_2 = speed[manifold.obj2_index]; 
      sVector3 rel_speed = { speed_1.x - speed_2.x, speed_1.y - speed_2.y, speed_1.z - speed_2.z };
     
      // AKA contanct speed
      float speed_rl_normal = dot_prod(manifold.collision_normal, rel_speed);
      ImGui::Text("Objects %d %d", manifold.obj1_index, manifold.obj2_index); 
      ImGui::Text("speed rl %f %f %f", rel_speed.x, rel_speed.y, rel_speed.z);
      ImGui::Text("Normal %f %f %f", manifold.collision_normal.x, manifold.collision_normal.y, manifold.collision_normal.z);

      if (speed_rl_normal == 0.0f) {
        ImGui::Text("earle");
        return;
      }

     float p1_inv_mass = (attribs[manifold.obj1_index] & IS_STATIC) ? 0.0f : mass[manifold.obj1_index];
     float p2_inv_mass = (attribs[manifold.obj2_index] & IS_STATIC) ? 0.0f : mass[manifold.obj2_index];

    float imp_magnitude = -1.0f * speed_rl_normal / (p1_inv_mass + p2_inv_mass);

    sVector3 collision_impulse = manifold.collision_normal;
    collision_impulse.x *= imp_magnitude;
    collision_impulse.y *= imp_magnitude;
    collision_impulse.z *= imp_magnitude;

    ImGui::Text("Impulse: %f %f %f", collision_impulse.x, collision_impulse.y, collision_impulse.z);

    add_impulse(manifold.obj1_index, collision_impulse.invert());
    add_impulse(manifold.obj2_index, collision_impulse);
  }
};

#endif
