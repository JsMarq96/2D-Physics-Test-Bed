#ifndef PHYSICS_H_
#define PHYSICS_H_

#include "imgui/imgui.h"
#include "math.h"
#include "collision_detection.h"
#include "math/vector.h"
#include "phys_parameters.h"
#include "types.h"

#define PHYS_INSTANCE_COUNT 100

struct sSpeed {
    sVector3 linear = {0.0f, 0.0f, 0.0f};
    sVector3 angular = {0.0f, 0.0f, 0.0f};
};

struct sPhysWorld {
    bool               enabled             [PHYS_INSTANCE_COUNT] = {};
    bool               is_static           [PHYS_INSTANCE_COUNT] = {};
    eColiderTypes      shape               [PHYS_INSTANCE_COUNT] = {};

    sTransform         *transforms                               = NULL;
    sSpeed             obj_speeds          [PHYS_INSTANCE_COUNT];

    float              mass                [PHYS_INSTANCE_COUNT] = {};
    float              inv_mass            [PHYS_INSTANCE_COUNT] = {};
    float              restitution         [PHYS_INSTANCE_COUNT] = {};

    sMat33             inv_inertia_tensors [PHYS_INSTANCE_COUNT] = {};

    sCollisionManifold _manifolds          [PHYS_INSTANCE_COUNT] = {};
    int                _manifold_count                           = 0;
    int                curr_frame_col_count                      = 0;


    inline float get_radius_of_collider(const int id) const {
        return MAX(transforms[id].scale.x, MAX(transforms[id].scale.y, transforms[id].scale.z)) / 2.0f;
    };

    void set_default_values() {
        // Set default values
        memset(enabled, false, sizeof(enabled));
        memset(is_static, false, sizeof(is_static));
        memset(obj_speeds, 0.0f, sizeof(obj_speeds));

        _manifold_count = 0;
    }

    // Set the initial values & calculate the inv_inertia tensors
    void init(sTransform *i_transforms) {
        transforms = i_transforms;

        // COnfigure Intertia Tensors
        sMat33 inertia_tensor = {};
        for(int i = 0; i < PHYS_INSTANCE_COUNT; i++) {
            if (is_static[i] || !enabled[i]) {
                continue;
            }

            inertia_tensor.set_identity();

            if (shape[i] == SPHERE_COLLIDER) {
                float radius = get_radius_of_collider(i);
                inertia_tensor.mat_values[0][0] = 2.0f/5.0f * mass[i] * radius * radius;
                inertia_tensor.mat_values[1][1] = 2.0f/5.0f * mass[i] * radius * radius;
                inertia_tensor.mat_values[2][2] = 2.0f/5.0f * mass[i] * radius * radius;
            }

            inertia_tensor.invert(&inv_inertia_tensors[i]);
        }

        for(int i = 0; i < PHYS_INSTANCE_COUNT; i++) {
            inv_mass[i] = (is_static[i]) ? 0.0 : 1.0f/mass[i];
        }

    }

    // Apply collisions & speeds, check for collisions, and resolve them
    void step(const double elapsed_time) {
        // 1 - Rotate inertia tensors
        for(int i = 0; i < PHYS_INSTANCE_COUNT; i++) {
            sMat33 r_mat = {}, r_mat_t = {}, inv_inertia = {};
            r_mat.convert_quaternion_to_matrix(transforms[i].rotation);
            r_mat.transponse_to(&r_mat_t);

            // Rotate the inertia tensor: I^-1 = r * I^-1 * r^t
            inv_inertia_tensors[i].multiply_to(&r_mat_t, &inv_inertia);
            r_mat.multiply_to(&inv_inertia, &inv_inertia_tensors[i]);
        }

        // 2 - Apply gravity
        apply_gravity(elapsed_time);

        // 3 - Collision Detection
        for(int i = 0; i < PHYS_INSTANCE_COUNT; i++) {
            for(int j = i+1; j < PHYS_INSTANCE_COUNT; j++) {
                if (!enabled[i] || !enabled[j]) {
                    continue;
                }

                // Skip the test if its between two static bodies
                if (is_static[i] && is_static[j]) {
                    continue;
                }

                // If there is a collision, store it in the manifold array
                // Test the different colliders
                if (shape[i] == SPHERE_COLLIDER && shape[j] == SPHERE_COLLIDER) {
                    if (test_sphere_sphere_collision(transforms[i].position,
                                                     get_radius_of_collider(i),
                                                     transforms[j].position,
                                                     get_radius_of_collider(j),
                                                     &_manifolds[_manifold_count])) {
                        _manifolds[_manifold_count].obj1 = i;
                        _manifolds[_manifold_count].obj2 = j;
                        _manifold_count++;
                    }
                } else if (shape[i] == PLANE_COLLIDER && shape[j] == SPHERE_COLLIDER) {
                    sVector3 origin = {0.0f, 0.0f, 0.0f};
                    sVector3 normal = {0.0f, 1.0f, 0.0f};

                    origin = transforms[i].apply(origin);
                    normal = transforms[i].apply_rotation(normal);

                    if (test_plane_sphere_collision(transforms[j].position,
                                                    get_radius_of_collider(j),
                                                    origin,
                                                    normal,
                                                    &_manifolds[_manifold_count])) {
                        _manifolds[_manifold_count].obj1 = j;
                        _manifolds[_manifold_count].obj2 = i;
                        _manifold_count++;
                    }
                } else if (shape[i] == SPHERE_COLLIDER && shape[j] == PLANE_COLLIDER) {
                    sVector3 origin = {0.0f, 0.0f, 0.0f};
                    sVector3 normal = {0.0f, 1.0f, 0.0f};

                    origin = transforms[j].apply(origin);
                    normal = transforms[j].apply_rotation(normal);

                    if (test_plane_sphere_collision(transforms[i].position,
                                                    get_radius_of_collider(i),
                                                    origin,
                                                    normal,
                                                    &_manifolds[_manifold_count])) {
                        _manifolds[_manifold_count].obj1 = i;
                        _manifolds[_manifold_count].obj2 = j;
                        _manifold_count++;
                    }

                }

            }
        }

        // 4 - Collision Resolution
        for(int iter = 0; iter < PHYS_SOLVER_ITERATIONS; iter++) {
            for(int i = 0; i < _manifold_count; i++) {
                impulse_response(_manifolds[i], elapsed_time);
            }

        }

        // 5 - Integrate solutions
        integrate(elapsed_time);

        curr_frame_col_count = _manifold_count;
        _manifold_count = 0;
    }

    void debug_speeds() const {
        // Debug speeds
        for(int i = 0; i < PHYS_INSTANCE_COUNT; i++) {
            if (!enabled[i]) {
                continue;
            }
            const sSpeed *speed = &obj_speeds[i];

            char instance_name []= "Obj 00";
            instance_name[5] = 48 + i;

            if(ImGui::TreeNode(instance_name)) {
                ImGui::Text("Position: %f %f %f", transforms[i].position.x, transforms[i].position.y, transforms[i].position.z);
                ImGui::Text("Linear speed: %f %f %f", speed->linear.x, speed->linear.y, speed->linear.z);
                ImGui::Text("Angular speed: %f %f %f", speed->angular.x, speed->angular.y, speed->angular.z);
                ImGui::TreePop();
            }

        }
        ImGui::Text("Collision num: %i", _manifold_count);
    }

    // Apply the speeds to the position
    void integrate(const double elapsed_time) {
        for(int i = 0; i < PHYS_INSTANCE_COUNT; i++) {
            if (is_static[i] || !enabled[i]) {
                continue;
            }

            sTransform *transf = &transforms[i];

            // Integrate linear speed: pos += speed * elapsed_time
            transf->position = transf->position.sum(obj_speeds[i].linear.mult(elapsed_time));

            // Integrate angular speed
            sQuaternion4 rotation_incr = obj_speeds[i].angular.get_pure_quaternion();
            rotation_incr = rotation_incr.multiply(0.5).multiply(elapsed_time);

            sQuaternion4 rotation = transf->rotation;
            rotation = rotation.sum(rotation_incr.multiply(rotation));
            rotation = rotation.normalize();

            transf->set_rotation(rotation);

            // Add some energy loss to the system
            obj_speeds[i].linear = obj_speeds[i].linear.mult(0.999f);
            obj_speeds[i].angular = obj_speeds[i].angular.mult(0.999f);
        }
    }

    // Apply the gravity based
    // TODO: Gravioty constant cleanup
    void apply_gravity(const double elapsed_time) {
        for(int i = 0; i < PHYS_INSTANCE_COUNT; i++) {
            if (is_static[i] || !enabled[i]) {
                continue;
            }

            obj_speeds[i].linear.y += -0.98f * elapsed_time;
        }
    }

    void impulse_response(const sCollisionManifold &manifold, const float elapsed_time) {
        int id_1 = manifold.obj1;
        int id_2 = manifold.obj2;

        sTransform *transf_1 = &transforms[id_1];
        sTransform *transf_2 = &transforms[id_2];

        sSpeed *speed_1 = &obj_speeds[id_1];
        sSpeed *speed_2 = &obj_speeds[id_2];

        // Calculate impulse response for each contact point
        for(int i = 0; i < manifold.contanct_points_count; i++) {
            // NORMAL IMPULSE ========
            // Vector from the center to the collision point
            sVector3 r1 = manifold.contact_points[i].subs(transf_1->position);
            sVector3 r2 = manifold.contact_points[i].subs(transf_2->position);

            sVector3 r1_cross_n = cross_prod(r1, manifold.normal);
            sVector3 r2_cross_n = cross_prod(r2, manifold.normal);

            // Calculate the collision momenton, aka the contact speed
            float collision_momentun = dot_prod(speed_1->linear.subs(speed_2->linear), manifold.normal) +
                                       dot_prod(r1_cross_n, speed_1->angular) -
                                       dot_prod(r2_cross_n, speed_2->angular);

            float linear_mass = inv_mass[id_1] + inv_mass[id_2];
            float angular_mass = dot_prod(r1_cross_n, inv_inertia_tensors[id_1].multiply(r1_cross_n)) +
                                 dot_prod(r2_cross_n, inv_inertia_tensors[id_2].multiply(r2_cross_n));

            // Baumgarte correction for the impulse
            float bias = -BAUMGARTE_TERM / elapsed_time * MIN(0.0f, manifold.contact_depth[i] + PENETRATION_SLOP);

            // Restitution constant
            float e = MIN(restitution[id_1], restitution[id_2]);

            float impulse_magnitude = (1 + e) * (collision_momentun + bias) / (linear_mass + angular_mass);

            sVector3 impulse = manifold.normal.mult(impulse_magnitude);

            if (!is_static[id_2]) {
                speed_2->linear = speed_2->linear.sum(impulse.mult(inv_mass[id_2]));
                speed_2->angular = speed_2->angular.sum(inv_inertia_tensors[id_2].multiply(cross_prod(r2, impulse)));
            }

            // Invert the impulse for the other body
            impulse = impulse.mult(-1.0f);

            if (!is_static[id_1]) {
                speed_1->linear = speed_1->linear.sum(impulse.mult(inv_mass[id_1]));
                speed_1->angular = speed_1->angular.sum(inv_inertia_tensors[id_1].multiply(cross_prod(r1, impulse)));
            }

            // FRICTION IMPULSES =====

            // Calculate the tangent wrenches
            sVector3 tangents[2] = {};
            plane_space(manifold.normal, &tangents[0], &tangents[1]);

            for(int tang = 0; tang < 2; tang++) {
                sVector3 r1_cross_t = cross_prod(r1, tangents[tang]);
                sVector3 r2_cross_t = cross_prod(r2, tangents[tang]);

                // Calculate the momentun & impulse, but with the tangent wrench instead of the normal
                collision_momentun = dot_prod(speed_1->linear.subs(speed_2->linear), tangents[tang]) +
                                     dot_prod(r1_cross_t, speed_1->angular) -
                                     dot_prod(r2_cross_t, speed_2->angular);

                angular_mass = dot_prod(r1_cross_t, inv_inertia_tensors[id_1].multiply(r1_cross_t)) +
                               dot_prod(r2_cross_t, inv_inertia_tensors[id_2].multiply(r2_cross_t));

                float friction_impulse_magnitude = collision_momentun / (linear_mass + angular_mass);
                sVector3 friction_impulse = tangents[tang].mult(friction_impulse_magnitude);

                if (!is_static[id_2]) {
                    speed_2->linear = speed_2->linear.sum(friction_impulse.mult(inv_mass[id_2]));
                    speed_2->angular = speed_2->angular.sum(inv_inertia_tensors[id_2].multiply(cross_prod(r2, friction_impulse)));
                }

                friction_impulse = friction_impulse.mult(-1.0f);

                if (!is_static[id_1]) {
                    speed_1->linear = speed_1->linear.sum(friction_impulse.mult(inv_mass[id_1]));
                    speed_1->angular = speed_1->angular.sum(inv_inertia_tensors[id_1].multiply(cross_prod(r1, friction_impulse)));
                }

            }
        }
    }
};

#endif // PHYSICS_H_
