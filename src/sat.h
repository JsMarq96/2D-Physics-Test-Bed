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

        float face_bias = 0.9f;
        float edge_bias = 0.8f;

        std::cout << "1:" << collision_distance_mesh1 << " 2:" << collision_distance_mesh2 << " e:" << edge_edge_distance << std::endl;

        sVector3 separating_axis = {};

        if (collision_distance_mesh1 < collision_distance_mesh2) {
            if (collision_distance_mesh1 < edge_edge_distance) {
                // Face 1
                separating_axis = mesh1.normals[collision_face_mesh1];
            } else {
                // Edge
                separating_axis = cross_prod(mesh1.get_edge(mesh1_collidion_edge),
                                             mesh2.get_edge(mesh2_collision_edge));
            }
        } else {
            if (collision_distance_mesh2 < edge_edge_distance) {
                // Face 2
                separating_axis = mesh2.normals[collision_face_mesh2];
            } else {
                //edge
                separating_axis = cross_prod(mesh1.get_edge(mesh1_collidion_edge),
                                             mesh2.get_edge(mesh2_collision_edge));
            }
        }

        float mesh1_max_facing = -FLT_MAX;
        float mesh1_min_facing = FLT_MAX;
        uint32_t mesh1_max_index = 0;
        uint32_t mesh1_min_index = 0;
        for(uint32_t i = 0; i <  mesh1.face_count; i++) {
            float dot_facing = dot_prod(mesh1.normals[i], separating_axis);

            if (mesh1_max_facing < dot_facing) {
                mesh1_max_facing = dot_facing;
                mesh1_max_index = i;
            }
            if (mesh1_min_facing > dot_facing) {
                mesh1_min_facing = dot_facing;
                mesh1_min_index = i;
            }
        }

        float mesh2_max_facing = -FLT_MAX;
        float mesh2_min_facing = FLT_MAX;
        uint32_t mesh2_max_index = 0;
        uint32_t mesh2_min_index = 0;
        for(uint32_t i = 0; i <  mesh2.face_count; i++) {
            float dot_facing = dot_prod(mesh2.normals[i], separating_axis);

            if (mesh2_max_facing < dot_facing) {
                mesh2_max_facing = dot_facing;
                mesh2_max_index = i;
            }
            if (mesh2_min_facing > dot_facing) {
                mesh2_min_facing = dot_facing;
                mesh2_min_index = i;
            }
        }

        const sColliderMesh *reference_mesh, *incident_mesh;
        uint32_t reference_face = 0, incident_face = 0;
        if (mesh2_min_facing < mesh1_min_facing + 0.001f) {
            // Mesh 2 has the reference face
            reference_mesh = &mesh2;
            reference_face = mesh2_min_index;

            incident_mesh = &mesh1;
            incident_face = mesh1_max_index;
        } else {
            // Mesh 1 fas the reference face
            reference_mesh = &mesh1;
            reference_face = mesh1_min_index;

            incident_mesh = &mesh2;
            incident_face = mesh2_max_index;
        }


        manifold->normal = reference_mesh->normals[reference_face];
        sPlane reference_plane = reference_mesh->get_plane_of_face(reference_face);

        manifold->contact_points[0] = reference_plane.origin_point;
        manifold->contact_points[1] = incident_mesh->plane_origin[incident_face];
        manifold->contanct_points_count = 2;
        //return true;
        manifold->contanct_points_count = clipping::face_face_clipping(*incident_mesh,
                                                                       incident_face,
                                                                       *reference_mesh,
                                                                       reference_face,
                                                                       manifold->contact_points);

        for(uint32_t i = 0; i < manifold->contanct_points_count; i++) {
            manifold->contact_depth[i] = reference_plane.distance(manifold->contact_points[i]);
            //manifold->contact_points[i] = manifold->contact_points[i].sum(manifold->normal);
            //manifold->contact_depth[i] = MIN(0.0f, manifold->contact_depth[i]);
            //std::cout << manifold->contact_depth[i] << std::endl;
        }

        return true;
    }
};

#endif // SAT_H_
