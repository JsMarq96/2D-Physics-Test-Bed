#ifndef PHYSICS_H_
#define PHYSICS_H_

#include "imgui/imgui.h"
#include "math.h"
#include "collision_detection.h"
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
                        _manifolds[_manifold_count].obj1 = i;
                        _manifolds[_manifold_count].obj2 = j;
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
                        _manifolds[_manifold_count].obj1 = j;
                        _manifolds[_manifold_count].obj2 = i;
                        _manifold_count++;
                    }

                }

            }
        }

        ImGui::Text("Collision num: %i", _manifold_count);

        // 4 - Collision Resolution
        for(int iter = 0; iter < PHYS_SOLVER_ITERATIONS; iter++) {
            for(int i = 0; i < _manifold_count; i++) {
                impulse_response(_manifolds[i], elapsed_time);
            }

        }

        // 5 - Integrate solutions
        integrate(elapsed_time);

        // Debug speeds
        for(int i = 0; i < PHYS_INSTANCE_COUNT; i++) {
            if (!enabled[i]) {
                continue;
            }
            sSpeed *speed = &obj_speeds[i];

            char instance_name []= "Obj 00";
            instance_name[5] = 48 + i;

            if(ImGui::TreeNode(instance_name)) {
                ImGui::Text("Position: %f %f %f", transforms[i].position.x, transforms[i].position.y, transforms[i].position.z);
                ImGui::Text("Linear speed: %f %f %f", speed->linear.x, speed->linear.y, speed->linear.z);
                ImGui::Text("Angular speed: %f %f %f", speed->angular.x, speed->angular.y, speed->angular.z);
                ImGui::TreePop();
            }

        }

        _manifold_count = 0;
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

    // TODO: no anade velocidad angular
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
            //sVector3 r1 = transf_1->position.subs(manifold.contact_points[i]);
            //sVector3 r2 = transf_2->position.subs(manifold.contact_points[i]);
            sVector3 r1 = manifold.contact_points[i].subs(transf_1->position);
            sVector3 r2 = manifold.contact_points[i].subs(transf_2->position);



            sVector3 r1_cross_n = cross_prod(r1, manifold.normal);
            sVector3 r2_cross_n = cross_prod(r2, manifold.normal);

            // Contact speed: (a_speed + (a_ang_speed x ra)) - (b_speed + (b_ang_speed x rb))
            sVector3 contact_speed = (speed_1->linear.sum(cross_prod(speed_1->angular, r1)));
            contact_speed = contact_speed.subs(speed_2->linear.sum(cross_prod(speed_2->angular, r2)));

            // Contac's linear mass: 1.0f / mass1 + 1.0f / mass2
            float linear_mass = inv_mass[id_1] + inv_mass[id_2];

            // Contact's angular mass: dot(inv_in1 * r1_cross_n + inv_in2 * r2_cross_n, normal)
            float angular_mass = dot_prod(inv_inertia_tensors[id_1].multiply(r1_cross_n).sum(inv_inertia_tensors[id_2].multiply(r2_cross_n)),
                                          manifold.normal);

            float proj_magnitude = dot_prod(contact_speed, manifold.normal);

            // Baumgarte Stabilization
            float bias = -BAUMGARTE_TERM * (1.0f / elapsed_time) * MIN(0.0f, (manifold.contact_depth[i] + PENETRATION_SLOP));

            // Calculate impulse
            float impulse_magnitude = proj_magnitude / (linear_mass + angular_mass);

            std::cout << "Impl " << impulse_magnitude << std::endl;

            sVector3 impulse = manifold.normal.mult(-impulse_magnitude + bias);

            // Apply impulse
            if (!is_static[id_1]) {
                speed_1->linear = speed_1->linear.sum(impulse.mult(inv_mass[id_1]));
                speed_1->angular = speed_1->angular.sum(inv_inertia_tensors[id_1].multiply(cross_prod(r1, impulse)));
            }

            // Invert the direction of the impulse, when apliying it to the other body
            impulse = impulse.mult(-1.0f);

            if (!is_static[id_2]) {
                speed_2->linear = speed_2->linear.sum(impulse.mult(inv_mass[id_2]));
                speed_2->angular = speed_2->angular.sum(inv_inertia_tensors[id_2].multiply(cross_prod(r2, impulse)));
            }

            continue;

            // FRICTION IMPULSE ========
            sVector3 tangents[2] = {};
            // ????
            plane_space(manifold.normal, &tangents[0], &tangents[1]);

            // Recalculate contact speed
            contact_speed = (speed_1->linear.sum(cross_prod(speed_1->angular, r1)));
            contact_speed = contact_speed.subs(speed_2->linear.sum(cross_prod(speed_2->angular, r2)));


            for(int j = 0; j < 2; j++) {
                // Calculate Tanget Momentum
                sVector3 r1_cross_t = cross_prod(r1, tangents[j]);
                sVector3 r2_cross_t = cross_prod(r2, tangents[j]);

                angular_mass = dot_prod(inv_inertia_tensors[id_1].multiply(r1_cross_t).sum(inv_inertia_tensors[id_2].multiply(r2_cross_t)),
                                        tangents[j]);

                proj_magnitude = dot_prod(contact_speed, tangents[j]);

                float tan_impulse_magnitude = proj_magnitude / (linear_mass + angular_mass);

                sVector3 tang_impulse = tangents[j].mult(-tan_impulse_magnitude);

                // Apply impulse
                if (!is_static[id_1]) {
                    speed_1->linear = speed_1->linear.sum(tang_impulse.mult(inv_mass[id_1]));
                    speed_1->angular = speed_1->angular.sum(inv_inertia_tensors[id_1].multiply(cross_prod(r2, tang_impulse)));
                }

                // Invert the direction of the impulse, when apliying it to the other body
                tang_impulse = tang_impulse.mult(-1.0f);

                if (!is_static[id_2]) {
                    speed_2->linear = speed_2->linear.sum(tang_impulse.mult(inv_mass[id_2]));
                    speed_2->angular = speed_2->angular.sum(inv_inertia_tensors[id_2].multiply(cross_prod(r2, tang_impulse)));
                }

            }
        }
    }
};

#endif // PHYSICS_H_
