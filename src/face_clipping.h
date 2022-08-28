#ifndef FACE_CLIPPING_H_
#define FACE_CLIPPING_H_

#include "collider_mesh.h"
#include "geometry.h"
#include "math.h"
#include "vector.h"
#include <cstdint>
#include <cstring>
#include <exception>

#define FACE_EPSILON 0.001f

namespace clipping {
    // Crop Mesh2's face to mesh1's face
    inline uint32_t face_face_clipping(const sColliderMesh *ref_mesh,
                                       const uint32_t ref_face,
                                       const sColliderMesh *inc_mesh,
                                       const uint32_t inc_face,
                                       sVector3 *clip_points) {
        // Fill clipping planes
        sPlane clipping_planes[5] = {};
        uint32_t clip_plane_count = (ref_mesh->face_stride == 4) ?  5 : 4;

        clipping_planes[0] = ref_mesh->get_plane_of_face(ref_face);
        for(uint16_t i = 0; ref_mesh->face_stride > i; i++) {
            clipping_planes[i + 1] = ref_mesh->get_plane_of_face(ref_mesh->get_neighboor_of_face(ref_face,
                                                                                                 i));
        }

        // 10 is the max number of points that can be had from clipping two quads
        sVector3 points_to_clip[10] = {};
        uint32_t points_to_clip_count = inc_mesh->face_stride;
        memcpy(points_to_clip,
               inc_mesh->get_face(inc_face),
               sizeof(sVector3) * points_to_clip_count);

        sVector3 clipped_points[10] = {};
        uint32_t clipped_points_count = 0;

        // Sutherland-Hogman clipping for all the clipping planes
        for(uint32_t plane_id = 0; plane_id < clip_plane_count; plane_id++) {
            sPlane &clip_plane = clipping_planes[plane_id];
            uint32_t num_clipped_points = 0;

            for(uint32_t i = 0; i < points_to_clip_count; i++) {
                sVector3 &v1 = points_to_clip[i];
                sVector3 &v2 = points_to_clip[(i+1) % points_to_clip_count];

                float dist_v1 = clip_plane.distance(v1);
                float dist_v2 = clip_plane.distance(v2);

                if (dist_v1 <= FACE_EPSILON && dist_v2 <= FACE_EPSILON) {
                    // Both points inside, store the second
                    clipped_points[clipped_points_count++] = v2;
                } else if (dist_v1 >= FACE_EPSILON && dist_v2 <= FACE_EPSILON) {
                    // First point outside, store the intersection point and the second
                    clipped_points[clipped_points_count++] = v2;
                    clipped_points[clipped_points_count++] = clip_plane.get_intersection_point(v1,
                                                                                               v2);
                } else if (dist_v1 <= FACE_EPSILON && dist_v2 >= FACE_EPSILON) {
                    // Second point outside, store the intersection point
                    clipped_points[clipped_points_count++] = clip_plane.get_intersection_point(v1,
                                                                                               v2);
                }
                // If both points are outisde, skip then
            }

            points_to_clip_count = clipped_points_count;
            clipped_points_count = 0;
            memcpy(points_to_clip, clipped_points, sizeof(sVector3) * points_to_clip_count);
        }

        memcpy(clip_points, points_to_clip, sizeof(sVector3) * points_to_clip_count);
        return points_to_clip_count;
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
