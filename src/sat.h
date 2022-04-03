#ifndef SAT_H_
#define SAT_H_

#include "collider_mesh.h"
#include "math.h"
#include "vector.h"
#include <cstdint>

/* Find the smallest distance of the nearest point on mesh2
 * to one of the faces of mesh1 */
inline bool test_face_face_collision(const sColliderMesh &mesh1,
                                     const sColliderMesh &mesh2,
                                     uint32_t *face_collision_index,
                                     float *collision_distance) {
    float largest_distance = -10000.0f;
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

    std::cout << largest_distance << std::endl;

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

inline bool test_edge_edge_collision(const sColliderMesh &mesh1,
                                     const sColliderMesh &mesh2,
                                     uint32_t *collision_edge1,
                                     uint32_t *collision_edge2,
                                     float *distance) {
    for(uint32_t i_edge1 = 0; mesh1.edge_cout > i_edge1; i_edge1++) {
        sVector3 edge1 = mesh1.get_edge(i_edge1).normalize();

        for(uint32_t i_edge2 = 0; mesh2.edge_cout > i_edge2; i_edge2++) {
            sVector3 edge2 = mesh2.get_edge(i_edge2).normalize();

            sVector3 new_axis = cross_prod(edge1, edge2);

            // Avoid test if the cross product are facing on the same direction
            if (new_axis.is_equal({0.0f, 0.0f, 0.0f})) {
                continue;
            }

            // Test in they overlap on the projection of the axis
            float mesh1_min = 0.0f, mesh1_max = 0.0f;
            float mesh2_min = 0.0f, mesh2_max = 0.0f;

            get_bounds_of_mesh_on_axis(mesh1, new_axis, &mesh1_min, &mesh1_max);
            get_bounds_of_mesh_on_axis(mesh2, new_axis, &mesh2_min, &mesh2_max);

            float total_shape_len = MAX(mesh1_max, mesh2_max) - MIN(mesh1_min, mesh2_min);
            float shape1_len = mesh1_max - mesh1_min;
            float shape2_len = mesh2_max - mesh2_min;

            if ((shape1_len + shape2_len) > total_shape_len) {
                *collision_edge1 = i_edge1;
                *collision_edge2 = i_edge2;
                *distance = (shape1_len + shape2_len) - total_shape_len;
                return true;
            }
        }
    }

    return false;
}


inline bool SAT_collision_test(const sColliderMesh &mesh1,
                               const sColliderMesh &mesh2) {

    uint32_t collision_face_mesh1 = 0;
    float collision_distance_mesh1 = 0.0f;
    uint32_t collision_face_mesh2 = 0;
    float collision_distance_mesh2 = 0.0f;

    // Test faces of mesh1 collider vs collider 2
    if (!test_face_face_collision(mesh1,
                                  mesh2,
                                  &collision_face_mesh1,
                                  &collision_distance_mesh1)) {
        return false;
    }

    // Test faces of mesh2 collider vs collider 1
    if (!test_face_face_collision(mesh2,
                                  mesh1,
                                  &collision_face_mesh2,
                                  &collision_distance_mesh2)) {
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

    return true;
}


#endif // SAT_H_
