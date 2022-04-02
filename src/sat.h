#ifndef SAT_H_
#define SAT_H_

#include "collider_mesh.h"
#include "vector.h"

/* Find the smallest distance of the nearest point on mesh2
 * to one of the faces of mesh1 */
inline bool test_face_face_collision(const sColliderMesh &mesh1,
                                     const sColliderMesh &mesh2,
                                     uint32_t *face_collision_index,
                                     float *collision_distance) {
    float largest_distance = 10000.0f;
    uint32_t face_index = 0;

    for(uint32_t i  = 0; i < mesh1.face_count; i++) {
        sPlane face_plane = mesh1.get_plane_of_face(i);
        sVector3 support_mesh2 = mesh2.get_support(face_plane.normal.invert());

        float distance = face_plane.distance(support_mesh2);

        if (largest_distance > distance) {
            face_index = i;
            largest_distance = distance;
        }
    }

    *face_collision_index = face_index;
    *collision_distance = largest_distance;

    return (largest_distance <= 0.0f);
}


inline bool SAT_collision_test(const sColliderMesh &mesh1, const sColliderMesh &mesh2) {

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

    return false;
}


#endif // SAT_H_
