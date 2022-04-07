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

            sSwapableVector3Stacks swapable_stack = {};
            swapable_stack.init(mesh2.face_stride * 2);
            for(uint32_t i = 0; i < mesh2.face_stride; i++) {
                //if (reference_face.distance(to_clip[i]) < 0.0005f) {
                    swapable_stack.add_element_to_current_stack(to_clip[i]);
                //}
            }

            sVector3 *cropping_plane_vertices = mesh1.get_face(face_1);
            sVector3 face_origin = mesh1.plane_origin[face_1];
            uint32_t num_of_clipped_points = 0;

            for(uint32_t edge_i = 0; edge_i < mesh1.face_stride; edge_i++) {
                    // Compute the croppiong plane
                    uint32_t edge_j = (edge_i + 1) % mesh1.face_stride;
                    sVector3 middle_edge = cropping_plane_vertices[edge_j].subs(cropping_plane_vertices[edge_i]).mult(0.5f);
                    middle_edge = cropping_plane_vertices[edge_i].sum(middle_edge);

                    sPlane cropping_plane = {};
                    cropping_plane.normal = middle_edge.subs(face_origin).normalize();
                    cropping_plane.origin_point = middle_edge;

                    // Perform clipping agains the plane
                    for(uint32_t i = 0; i < swapable_stack.get_current_stacks_size(); i++) {
                            sVector3 vert1 = swapable_stack.get_element_from_current_stack(i);
                            sVector3 vert2 = swapable_stack.get_element_from_current_stack((i + 1) % swapable_stack.get_current_stacks_size());

                            float distance_vert1 = cropping_plane.distance(vert1);
                            float distance_vert2 = cropping_plane.distance(vert2);

                            if (distance_vert1 < 0.0f && distance_vert2 < 0.0f) {
                                // Add the vert2
                                swapable_stack.add_element_to_secundary_stack(vert2);
                            } else if (distance_vert1 >= 0.0f && distance_vert2 < 0.0f) {
                                // Add intersection point & vert2
                                swapable_stack.add_element_to_secundary_stack(cropping_plane.get_intersection_point(vert1,
                                                                                                                    vert2));
                                swapable_stack.add_element_to_secundary_stack(vert2);
                            } else if (distance_vert1 < 0.0f && distance_vert2 >= 0.0f) {
                                // Add intersection point
                                swapable_stack.add_element_to_secundary_stack(cropping_plane.get_intersection_point(vert1,
                                                                                                                    vert2));
                            }
                            // If both are outside, do nothing
                    }

                    swapable_stack.clean_current_stack();
                    swapable_stack.swap();
            }
            for(num_of_clipped_points = 0; num_of_clipped_points <  swapable_stack.get_current_stacks_size(); num_of_clipped_points++) {
                clip_points[num_of_clipped_points] = swapable_stack.get_element_from_current_stack(num_of_clipped_points);
            }
            //std::cout << num_of_clipped_points << " " << mesh2.face_stride * 2 << std::endl;

            swapable_stack.clean_current_stack();
            swapable_stack.clean();

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
