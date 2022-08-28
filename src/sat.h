#ifndef SAT_H_
#define SAT_H_

#include "collider_mesh.h"
#include "collision_detection.h"
#include "face_clipping.h"
#include "math.h"
#include "mesh.h"
#include "vector.h"
#include <cfloat>
#include <cstdint>
#include <sys/types.h>

namespace SAT {

    /* Find the smallest distance of the nearest point on mesh2
     * to one of the faces of mesh1 */
    inline bool test_face_face_collision(const sColliderMesh &mesh1,
                                         const sColliderMesh &mesh2,
                                         uint32_t *face_collision_index,
                                         float *collision_distance) {
        float largest_distance = -FLT_MAX;
        uint32_t face_index = 0;

        for(uint32_t i = 0; i < mesh1.face_count; i++) {
            sPlane face_plane = mesh1.get_plane_of_face(i);
            sVector3 support_mesh2 = mesh2.get_support(face_plane.normal.invert());

            float distance = face_plane.distance(support_mesh2);
            if (largest_distance < distance) {
                face_index = i;
                largest_distance = distance;
            }
        }

        *face_collision_index = face_index;
        *collision_distance = largest_distance;

        return (largest_distance <= 0.0f);
    }

    inline void get_bounds_of_mesh_on_axis(const sColliderMesh &mesh,
                                           const sVector3 &axis,
                                           float *min,
                                           float *max) {
        float min_shape = FLT_MAX;
        float max_shape = -FLT_MAX;

        for(uint32_t i = 0; i < mesh.vertices_count; i++) {
            float projection = dot_prod(mesh.vertices[i], axis);

            min_shape = MIN(min_shape, projection);
            max_shape = MAX(max_shape, projection);
        }

        *min = min_shape;
        *max = max_shape;
    }

    inline bool test_face_face_non_support_collision(const sColliderMesh &mesh1,
                                                     const sColliderMesh &mesh2,
                                                     uint32_t *collision_face1,
                                                     float *distance) {
        float smallest_distance = FLT_MAX;
        uint32_t col_face1 = 0;

        for(uint32_t i_face1 = 0; i_face1 < mesh1.face_count; i_face1++) {
            sVector3 axis_to_test = mesh1.normals[i_face1];

            float mesh1_min, mesh1_max;
            get_bounds_of_mesh_on_axis(mesh1,
                                       axis_to_test,
                                       &mesh1_min,
                                       &mesh1_max);

            float mesh2_min, mesh2_max;
            get_bounds_of_mesh_on_axis(mesh2,
                                       axis_to_test,
                                       &mesh2_min,
                                       &mesh2_max);

            float total_shape_len = MAX(mesh1_max, mesh2_max) - MIN(mesh1_min, mesh2_min);
            float shape1_len = mesh1_max - mesh1_min;
            float shape2_len = mesh2_max - mesh2_min;

            float overlap_distance = total_shape_len - (shape1_len + shape2_len);

            if (overlap_distance > 0.0f) {
                return false; // No overlaping
            } else if (overlap_distance <= smallest_distance) {
                smallest_distance = overlap_distance;
                col_face1 = i_face1;
            }
        }

        *distance = smallest_distance;
        *collision_face1 = col_face1;
        return true;
    }

    inline bool test_edge_edge_collision(const sColliderMesh &mesh1,
                                         const sColliderMesh &mesh2,
                                         uint32_t *collision_edge1,
                                         uint32_t *collision_edge2,
                                         float *distance) {
        float largest_distance = -FLT_MAX;
        uint32_t smallest_edge1 = 0;
        uint32_t smallest_edge2 = 0;
        for(uint32_t i_edge1 = 0; mesh1.edge_cout > i_edge1; i_edge1++) {
            sVector3 edge1 = mesh1.get_edge(i_edge1).normalize();

            for(uint32_t i_edge2 = 0; mesh2.edge_cout > i_edge2; i_edge2++) {
                sVector3 edge2 = mesh2.get_edge(i_edge2).normalize();

                sVector3 new_axis = cross_prod(edge1, edge2);

                // Avoid test if the cross product are facing on the same direction
                if (new_axis.magnitude() < 0.0001f) {
                    continue;
                }

                float mesh1_max, mesh1_min;
                float mesh2_max, mesh2_min;

                get_bounds_of_mesh_on_axis(mesh1,
                                           new_axis,
                                           &mesh1_min,
                                           &mesh1_max);
                get_bounds_of_mesh_on_axis(mesh2,
                                           new_axis,
                                           &mesh2_min,
                                           &mesh2_max);
                float total_projection = MAX(mesh1_max, mesh2_max) - MIN(mesh1_min, mesh2_min);
                float penetration_on_axis = total_projection - ((mesh1_max - mesh1_min) + (mesh2_max - mesh2_min));

                if (largest_distance < penetration_on_axis) {
                    largest_distance = penetration_on_axis;
                    smallest_edge1 = i_edge1;
                    smallest_edge2 = i_edge2;
                }
            }
        }

        *collision_edge1 = smallest_edge1;
        *collision_edge2 = smallest_edge2;
        *distance = largest_distance;
        return (largest_distance <= 0.0f);
    }


    enum eCollisionType : uint8_t {
       FACE_1_COL = 0,
       FACE_2_COL,
       EDGE_EDGE_COL,
       NONE
    };


   inline bool SAT_collision_test(const sColliderMesh &mesh1,
                                   const sColliderMesh &mesh2,
                                   sCollisionManifold *manifold) {

        uint32_t collision_face_mesh1 = 0;
        float collision_distance_mesh1 = 0.0f;
        uint32_t collision_face_mesh2 = 0;
        float collision_distance_mesh2 = 0.0f;

        eCollisionType collision = NONE;

        // Test faces of mesh2 collider vs collider 1
        if (!test_face_face_collision(mesh2,
                                      mesh1,
                                      &collision_face_mesh2,
                                      &collision_distance_mesh2)) {
            return false;
        }


        // Test faces of mesh1 collider vs collider 2
        if (!test_face_face_collision(mesh1,
                                      mesh2,
                                      &collision_face_mesh1,
                                      &collision_distance_mesh1)) {
            return false;
        }


        // Test cross product of the edges
        uint32_t mesh1_collidion_edge = 0, mesh2_collision_edge = 0;
        float edge_edge_distance = 0.0f;
        if (!test_edge_edge_collision(mesh1,
                                      mesh2,
                                      &mesh1_collidion_edge,
                                      &mesh2_collision_edge,
                                      &edge_edge_distance)) {
            return false;
        }


        // TODO: Manifold and contact point extraction

        // Collision cases:
        //  Edge v Edge
        //  Face v (edge or Face)
        //  http://vodacek.zvb.cz/archiv/293.html
        //
        sPlane crop_plane = {};

        sVector3 separating_axis = {};
        const sColliderMesh *reference_mesh, *incident_mesh;
        uint32_t reference_face = 0, incident_face = 0;

        const float max_face_separation = MIN(collision_distance_mesh1, collision_distance_mesh2);
        const float k_edge_rel_tolerance = 0.90f;
        const float k_face_rel_toletance = 0.98f;
        const float k_abs_tolerance = 0.5f * 0.005f;

        // Select the reference object and the incident
         if (collision_distance_mesh1 + 0.005f > collision_distance_mesh2) {
             reference_mesh = &mesh1;
             incident_mesh = &mesh2;

             reference_face = collision_face_mesh1;
         } else {
             reference_mesh = &mesh2;
             incident_mesh = &mesh1;

             reference_face = collision_face_mesh2;
         }

         sVector3 reference_normal = reference_mesh->normals[reference_face];

         float ref_proj = dot_prod(reference_normal,
                                   reference_mesh->get_support(reference_normal));
         float inc_proj = dot_prod(reference_normal,
                                   incident_mesh->get_support(reference_normal));

         if (inc_proj < ref_proj) {
             reference_normal = reference_normal.invert();
         }

         manifold->normal = reference_normal;

         // Choose indicent face
         incident_face = 0;
         float incident_facing = dot_prod(reference_normal,
                                          incident_mesh->normals[0]);
         for(uint32_t i = 1; i < incident_mesh->face_count; i++) {
             float tmp_incident = dot_prod(reference_normal,
                                           incident_mesh->normals[i]);

             if (tmp_incident < incident_facing) {
                 incident_facing = tmp_incident;
                 incident_face = i;
             }

         }

         sVector3 contact_points[12] = {};
         manifold->contanct_points_count = clipping::face_face_clipping(reference_mesh,
                                                                        reference_face,
                                                                        incident_mesh,
                                                                        incident_face,
                                                                        manifold->contact_points);

         // Get clipping depths
         sPlane reference_plane = reference_mesh->get_plane_of_face(reference_face);
         for(uint32_t i = 0; i < manifold->contanct_points_count; i++) {
             // Distance minus the face epsilon
             manifold->contact_depth[i] = reference_plane.distance(manifold->contact_points[i]) ;
         }

         return true;
    }


    inline void project_to_axis(const sVector3 &axis,
                                const sColliderMesh &box_mesh,
                                float *min_on_axis,
                                float *max_on_axis) {
        float min = FLT_MAX;
        float max = -FLT_MAX;

        for(int i = 0; i < box_mesh.vertices_count; i++) {
            float projection = dot_prod(box_mesh.vertices[i], axis);
            min = MIN(min, projection);
            max = MAX(max, projection);
        }

        *min_on_axis = min;
        *max_on_axis = max;
    }

    inline float overlap(const float min_1,
                         const float max_1,
                         const float min_2,
                         const float max_2) {
        float p = MAX(max_1, max_2) - MIN(min_1, min_2);

        return ((max_2 - min_2) + (max_1 - min_1)) - p;
    }

    inline bool SAT_sphere_cube_collision(const sVector3 &sphere_center,
                                          const float sphere_radius,
                                          const sTransform &cube_transform,
                                          const sColliderMesh &box_mesh,
                                          sCollisionManifold *manifold) {
        sVector3 axis_normals[3] = {
             cube_transform.apply_rotation({0.0f, 1.0f, 0.0f}).normalize(),
             cube_transform.apply_rotation({1.0f, 0.0f, 0.0f}).normalize(),
             cube_transform.apply_rotation({0.0f, 0.0f, 1.0f}).normalize()  };

        int min_axis = -1;
        float min_separation = FLT_MAX;

        for(int i = 0; i < 3; i++) {
            float proj_sphere_center = dot_prod(sphere_center, axis_normals[i]);
            float box_min = 0.0f, box_max = 0.0f;

            project_to_axis(axis_normals[i],
                            box_mesh,
                            &box_min,
                            &box_max);

            float axis_overlap = overlap(box_min,
                                         box_max,
                                         proj_sphere_center - sphere_radius,
                                         proj_sphere_center + sphere_radius);
            // If there is no overlap, then the sat test is negative,
            // so early exit
            if (axis_overlap <= 0.001f) {
                return false;
            }

            if (min_separation > axis_overlap) {
                min_axis = i;
                min_separation = axis_overlap;
            }
        }

        // Collision manifold generation
        sVector3 col_axis = axis_normals[min_axis];
        float sphere_center_proj = dot_prod(sphere_center, col_axis);
        float box_center_proj = dot_prod(cube_transform.position, col_axis);

        // Invert the axis based on the relative position to the sphere to the center
        if (sphere_center_proj < box_center_proj) {
            col_axis = col_axis.invert();
        }

        manifold->contact_points[0] = sphere_center.sum(col_axis.invert().mult(sphere_radius));
        manifold->contact_depth[0] = -min_separation;
        manifold->normal = col_axis;
        manifold->contanct_points_count = 1;

        return true;
    }
};

#endif // SAT_H_
