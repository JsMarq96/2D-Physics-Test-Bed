#include "physics.h"
#include "imgui/imgui.h"
#include <X11/Xlib.h>

#include "math.h"
#include "render_cubes.h"

void sPhysicsWorld::step(const double elapsed_time, const sMat44 *proj_mat) {
    sCubeRenderer renderer;

    cube_renderer_init(&renderer);
    // 0.- Apply gravity
    apply_gravity(elapsed_time);
    // 1.- Broadphase detection
    // TODO: O(n^2) look up no bueno
     // 2.- Manifold integration: If tehr eis a collision, we add it to the
     // arbiter
     for(int i = 0; i < INSTANCE_SIZE; i++) {
        for (int j = i + 1; j < INSTANCE_SIZE; j++) {
            // Test if there is an exisiting arbiter from previus step
            int arbiter_id = arbiter.get_arbiter_id(i, j);

            if (arbiter_id < 0) {
                arbiter_id = arbiter.create_arbiter(i, j);
            }

            sCollisionManifold manifold = {};

            manifold.obj1_index = i;
            manifold.obj2_index = j;

            if (SAT_test(transforms[i],
                         transforms[j],
                         &manifold)) {
                // Collision detected!
                arbiter.add_manifold(arbiter_id, manifold);
                ImGui::Text("Collision between %i %i", i, j);
            } else {
                // No collision detected
                arbiter.remove_arbiter(arbiter_id);
            }
        }
    }
    ImGui::Separator();

    int contact_count = 0;
    for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
        if (arbiter.used_in_frame[i]){
            contact_count += arbiter.contact_size[i];
            ImGui::Text("Arbiter %d Contact count %d", i, arbiter.contact_size[i]);
        }
    }

    sMat44 *models = (sMat44*) malloc(sizeof(sMat44) * contact_count);
    sVector4 *colors = (sVector4*) malloc(sizeof(sVector4) * contact_count);

    int it_index = 0;
    for(int i = 0; i < MAX_ARBITERS_SIZE; i++) {
        if (arbiter.used_in_frame[i]){
            for(int j = 0; j < arbiter.contact_size[i]; j++) {
                models[it_index].set_identity();
                models[it_index].set_position(arbiter.contact_data[i][j].contanct_point);
                models[it_index].set_scale({0.05f, 0.1f, 0.1f});
                colors[it_index] = {1.0f, 0.0f, 0.0f, 1.0f};
                it_index++;
            }
        }
    }

    cube_renderer_render(&renderer, models, colors, contact_count, proj_mat);


    // 3.- Prestep (?)
    pre_step(elapsed_time);

    // 4.- Generate impulses
    for(int i = 0; i < PHYS_SOLVER_ITERATIONS; i++) {
        step(elapsed_time);
    }

    for(int i = 0; i < INSTANCE_SIZE; i++) {
        //ImGui::Text("%d speed %f %f %f", i, speed[i].x, speed[i].y, speed[i].z);
     }
     ImGui::Separator();


    // 5.- Apply impulses
    integrate(elapsed_time);


    // 6.- Remove unused arbiters
    arbiter.remove_unused_arbiters();
}


void sPhysicsWorld::config_simulation() {
    generate_inertia_tensors();
    arbiter.init();
}


// Based on https://en.wikipedia.org/wiki/List_of_moments_of_inertia
void sPhysicsWorld::generate_inertia_tensors() {
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

void sPhysicsWorld::apply_gravity(const double elapsed_time) {
    for(int i = 0; i < INSTANCE_SIZE; i++) {
      if (is_static[i]) {
        continue;
      }

      // Integrate gravity
      // TODO: Gravity constatn cleanup
      speed[i].y += -0.98f * elapsed_time;
    }
}

void sPhysicsWorld::integrate(const double elapsed_time) {
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

      transforms[i].set_rotation(rot_quat);
    }
}
