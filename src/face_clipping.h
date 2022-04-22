#ifndef FACE_CLIPPING_H_
#define FACE_CLIPPING_H_

#include "collider_mesh.h"
#include "data_structs/swapable_stack.h"
#include "geometry.h"
#include "vector.h"
#include <cstdint>
#include <cstring>
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
            // ONLY NEEDS TO CLIP WITH ONE PLANE! THE COLLISION PLANE
            // https://pybullet.org/Bullet/phpBB3/viewtopic.php?t=12562
            sVector3 *to_clip = (sVector3*) malloc(sizeof(sVector3) * 8);
            memcpy(to_clip, mesh2.get_face(face_2), sizeof(sVector3) * mesh2.face_stride);
            uint32_t num_of_points_to_clip = mesh2.face_stride;

            //std::cout << face_2 << ":  "<< reference_face.origin_point.x << " " << reference_face.origin_point.y << " " << reference_face.origin_point.z << std::endl;
            // Perform clipping agains the plane
            for(uint32_t clip_plane = 0; clip_plane < mesh1.face_stride; clip_plane++) {
                sPlane reference_face = mesh1.get_plane_of_face(mesh1.get_neighboor_of_face(face_1, clip_plane));
                uint32_t num_of_clipped_points = 0;

                for(uint32_t i = 0; i < num_of_points_to_clip; i++) {
                    sVector3 vert1 = to_clip[i];
                    sVector3 vert2 = to_clip[(i + 1) % mesh2.face_stride];

                    float distance_vert1 = reference_face.distance(vert1);
                    float distance_vert2 = reference_face.distance(vert2);

                    //std::cout << vert1.x << " + " << vert1.y << " + " << vert1.z << std::endl;
                    //std::cout << distance_vert1 << " " << distance_vert2 << std::endl;

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
                num_of_points_to_clip = num_of_clipped_points;
                memcpy(to_clip, clip_points, sizeof(sVector3) * num_of_clipped_points);
            }

            std::cout << num_of_points_to_clip << std::endl;
            sPlane reference_face = mesh1.get_plane_of_face(face_1);
            uint32_t num_of_clipped_points = 0;

                for(uint32_t i = 0; i < num_of_points_to_clip; i++) {
                    sVector3 vert1 = to_clip[i];
                    sVector3 vert2 = to_clip[(i + 1) % mesh2.face_stride];

                    float distance_vert1 = reference_face.distance(vert1);
                    float distance_vert2 = reference_face.distance(vert2);

                    //std::cout << vert1.x << " + " << vert1.y << " + " << vert1.z << std::endl;
                    //std::cout << distance_vert1 << " " << distance_vert2 << std::endl;

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


            free(to_clip);

            //   Iterate all the edges of the clipped
            //      If both vertices are inside,
            //         then we add the second(last) point
            //      If the first is outside, and the second is inside,
            //         we add the intersection point, and the second
            //      If the firs is inside and the second is outside,
            //         we only add the intersection point
            //      Both vertecis are outside,
            //         we dont add any points
            return num_of_points_to_clip;
    }
};

#endif // FACE_CLIPPING_H_
