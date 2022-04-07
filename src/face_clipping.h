#ifndef FACE_CLIPPING_H_
#define FACE_CLIPPING_H_

#include "collider_mesh.h"
#include "data_structs/swapable_stack.h"
#include "geometry.h"
#include "vector.h"
#include <cstdint>
#include <exception>

namespace clipping {
    // Crop Mesh2's face to mesh1's face
    inline uint32_t face_face_clipping(const sColliderMesh &mesh1,
                                       const uint32_t face_1,
                                       const sColliderMesh &mesh2,
                                       const uint32_t face_2,
                                       sVector3 *clip_points) {

            // Sutherland-Hodgman Cliping

            // Select one face to do the clipping, and another to do be the clipped
            // Compute clipping planes:
            // Half point of the edge as the origin, and the normal is
            // from face plane's origin to the half-point.
            // ONLY NEEDS TO CLIP WITH ONE PLANE! THE COLLISION PLAN
            // https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12562
            sVector3 *to_clip = mesh2.get_face(face_2);

            sPlane reference_face = mesh1.get_plane_of_face(face_1);

            sVector3 face_origin = mesh1.plane_origin[face_1];
            uint32_t num_of_clipped_points = 0;

            // Perform clipping agains the plane
            for(uint32_t i = 0; i < mesh2.face_stride; i++) {
                    sVector3 vert1 = to_clip[i];
                    sVector3 vert2 = to_clip[(i + 1) % mesh2.face_stride];

                    float distance_vert1 = reference_face.distance(vert1);
                    float distance_vert2 = reference_face.distance(vert2);

                    if (distance_vert1 < 0.0f && distance_vert2 < 0.0f) {
                        // Add the vert2
                        clip_points[num_of_clipped_points++] = vert2;
                    } else if (distance_vert1 >= 0.0f && distance_vert2 < 0.0f) {
                        // Add intersection point & vert2
                        clip_points[num_of_clipped_points++] = reference_face.get_intersection_point(vert1,
                                                                                                     vert2);
                        clip_points[num_of_clipped_points++] = vert2;
                    } else if (distance_vert1 < 0.0f && distance_vert2 >= 0.0f) {
                        // Add intersection point
                        clip_points[num_of_clipped_points++] = reference_face.get_intersection_point(vert1,
                                                                                                     vert2);
                    }
                    // If both are outside, do nothing
            }

            // Iterate all the clipping planes
            //   Iterate all the edges of the clipped
            //      If both vertices are inside,
            //         then we add the second(last) point
            //      If the first is outside, and the second is inside,
            //         we add the intersection point, and the second
            //      If the firs is inside and the second is outside,
            //         we only add the intersection point
            //      Both vertecis are outside,
            //         we dont add any points
            return num_of_clipped_points;
    }
};

#endif // FACE_CLIPPING_H_
