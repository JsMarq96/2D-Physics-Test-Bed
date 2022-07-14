#ifndef FACE_CLIPPING_H_
#define FACE_CLIPPING_H_

#include "collider_mesh.h"
#include "geometry.h"
#include "math.h"
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
            sVector3 *to_clip = (sVector3*) malloc(sizeof(sVector3) * 15);
            memcpy(to_clip, mesh2.get_face(face_2), sizeof(sVector3) * mesh2.face_stride);
            uint32_t num_of_points_to_clip = mesh2.face_stride;

            sPlane reference_plane = mesh1.get_plane_of_face(face_1);

            // first clip agains the reference plane
            uint32_t num_of_clipped_points = 0;

            // Perform clipping agains the neighboring planes
            for(uint32_t clip_plane = 0; clip_plane < mesh1.face_stride; clip_plane++) {
                //std::cout << clip_plane << " " << mesh1.face_stride << std::endl;
                sPlane clipping_face = mesh1.get_plane_of_face(mesh1.get_neighboor_of_face(face_1, clip_plane));
                uint32_t num_of_clipped_points = 0;

                for(uint32_t i = 0; i < num_of_points_to_clip; i++) {
                    sVector3 vert1 = to_clip[i];
                    sVector3 vert2 = to_clip[(i + 1) % mesh2.face_stride];

                    float distance_vert1 = clipping_face.distance(vert1);
                    float distance_vert2 = clipping_face.distance(vert2);

                    if (distance_vert1 < 0.0001f && distance_vert2 < 0.0001f) {
                        // Add the vert2
                        clip_points[num_of_clipped_points++] = vert2;
                    } else if (distance_vert1 >= 0.0001f && distance_vert2 < 0.0001f) {
                        // Add intersection point & vert2
                        clip_points[num_of_clipped_points++] = clipping_face.get_intersection_point(vert1,
                                                                                                     vert2);
                        clip_points[num_of_clipped_points++] = vert2;
                    } else if (distance_vert1 < 0.0001f && distance_vert2 >= 0.0001f) {
                        // Add intersection point
                        clip_points[num_of_clipped_points++] = clipping_face.get_intersection_point(vert1,
                                                                                                     vert2);
                    }
                    // If both are outside, do nothing
                }
                num_of_points_to_clip = num_of_clipped_points;
                memcpy(to_clip, clip_points, sizeof(sVector3) * num_of_clipped_points);
            }

            // Clipping against the reference plane
            for(uint32_t i = 0; i < num_of_points_to_clip; i++) {
                sVector3 vert1 = to_clip[i];
                sVector3 vert2 = to_clip[(i + 1) % mesh2.face_stride];

                float distance_vert1 = reference_plane.distance(vert1);
                float distance_vert2 = reference_plane.distance(vert2);

                if (distance_vert1 <= 0.0001f && distance_vert2 <= 0.0001f) {
                    // Add the vert2
                    clip_points[num_of_clipped_points++] = vert2;
                } else if (distance_vert1 > 0.0001f && distance_vert2 <= 0.0001f) {
                    // Add intersection point & vert2
                    clip_points[num_of_clipped_points++] = vert2;
                    clip_points[num_of_clipped_points++] = reference_plane.get_intersection_point(vert1,
                                                                                                  vert2);
                } else if (distance_vert1 <= 0.0001f && distance_vert2 > 0.0001f) {
                    // Add intersection point
                    clip_points[num_of_clipped_points++] = reference_plane.get_intersection_point(vert2,
                                                                                                  vert1);
                }
                // If both are outside, do nothing
            }
            num_of_points_to_clip = num_of_clipped_points;
            memcpy(to_clip, clip_points, sizeof(sVector3) * num_of_clipped_points);
            num_of_clipped_points = 0;

            // TODO: clip agains adjacent faces

            free(to_clip);
            //return num_of_clipped_points;


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

    inline uint32_t edge_edge_clipping(const sColliderMesh &mesh1,
                                       const uint32_t edge_1,
                                       const sColliderMesh &mesh2,
                                       const uint32_t edge_2,
                                       sVector3 *clip_point,
                                       float *distance) {
        const sVector3 edge_mesh1 = mesh1.get_edge(edge_1);
        const sVector3 edge_mesh2 = mesh2.get_edge(edge_2);

        // https://math.stackexchange.com/questions/846054/closest-points-on-two-line-segments
        const sVector3 p1x = mesh1.vertices[mesh1.edges[edge_1].x], p1y = mesh1.vertices[mesh1.edges[edge_1].y];
        const sVector3 p2x = mesh2.vertices[mesh2.edges[edge_2].x], p2y = mesh2.vertices[mesh2.edges[edge_2].y];

        const float edge_1_len = edge_mesh1.magnitude();
        const float edge_2_len = edge_mesh2.magnitude();

        const sVector3 p1_p2 = p1x.subs(p2x);
        const float d12 = dot_prod(edge_mesh1, edge_mesh2);
        const float d_edge1_p1 = dot_prod(edge_mesh1, p1_p2);
        const float d_edge2_p1 = dot_prod(edge_mesh2, p1_p2);

        const float denom = edge_1_len * edge_2_len - d12 * d12;
        const float f1 = (d12 * d_edge2_p1 - d_edge1_p1 * edge_2_len) / denom;
        const float f2 = (d12 * f1 + d_edge2_p1) / edge_2_len;

        const sVector3 point_1 = p1x.sum(edge_mesh1.mult(f1));
        const sVector3 point_2 = p2x.sum(edge_mesh2.mult(f2));

        *clip_point = point_1.sum(point_2).mult(0.5f);
        *distance = point_1.subs(point_2).magnitude();

        return 1;
    }
};

#endif // FACE_CLIPPING_H_
