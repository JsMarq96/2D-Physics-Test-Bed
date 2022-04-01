#ifndef COLLIDER_MESH_H_
#define COLLIDER_MESH_H_

#include "math.h"
#include "transform.h"
#include "vector.h"
#include "geometry.h"
#include <cstdint>

enum eFaceSize : uint32_t {
    FACE_TRI = 3,
    FACE_QUAD = 4
};


struct sColliderMesh {
    sVector3   *vertices = NULL;
    sVector3   *normals = NULL;
    sVector3   *plane_origin = NULL;

    uint32_t   vertices_count = 0;
    uint32_t   face_count = 0;

    eFaceSize  face_stride = FACE_QUAD;

    void init_cuboid(const sTransform &transform) {
        int box_LUT_vertices[6 * 4] = { 4, 5, 7, 6,   6, 7, 3, 2,   1, 3, 7, 5,   0, 1, 3, 2,   0, 1, 5, 4,   0, 2, 6, 4};

        vertices = (sVector3*) malloc(sizeof(sVector3) * 6 * 4);
        normals = (sVector3*) malloc(sizeof(sVector3) * 6);
        plane_origin = (sVector3*) malloc(sizeof(sVector3) * 6);

        vertices_count = 6 * 4;
        face_count = 6;

        // Vertices
        sVector3 raw_points[8] = {};
        raw_points[0] = transform.apply(sVector3{0.0f, 0.0f, 0.0f});
        raw_points[1] = transform.apply(sVector3{1.0f, 0.0f, 0.0f});
        raw_points[2] = transform.apply(sVector3{0.0f, 1.0f, 0.0f});
        raw_points[3] = transform.apply(sVector3{1.0f, 1.0f, 0.0f});
        raw_points[4] = transform.apply(sVector3{0.0f, 0.0f, 1.0f});
        raw_points[5] = transform.apply(sVector3{1.0f, 0.0f, 1.0f});
        raw_points[6] = transform.apply(sVector3{0.0f, 1.0f, 1.0f});
        raw_points[7] = transform.apply(sVector3{1.0f, 1.0f, 1.0f});

        for(uint32_t i = 0; i < 6*4; i++) {
            vertices[i] = raw_points[box_LUT_vertices[i]];
        }

        // Face origin
        for(int i = 0; i < 6; i++) {
            sVector3 center = {};

            for(int j = 0; j < 4; j++) {
                sVector3 tmp = raw_points[ box_LUT_vertices[(i * 4) + j] ];
                center.x += tmp.x;
                center.y += tmp.y;
                center.z += tmp.z;
            }

            center.x /= 4.0f;
            center.y /= 4.0f;
            center.z /= 4.0f;

            plane_origin[i] = center;//transform.apply_without_scale(center);
        }

        // Face normals
        normals[0] = transform.apply_rotation(sVector3{0.0f, 0.0f, 1.0f});
        normals[1] = transform.apply_rotation(sVector3{0.0f, 1.0f, 0.0f});
        normals[2] = transform.apply_rotation(sVector3{1.0f, 0.0f, 0.0f});
        normals[3] = transform.apply_rotation(sVector3{0.0f, 0.0f, -1.0f});
        normals[4] = transform.apply_rotation(sVector3{0.0f, -1.0f, 0.0f});
        normals[5] = transform.apply_rotation(sVector3{-1.0f, 0.0f, 0.0f});

        face_stride = FACE_QUAD;
    }

    void clean() {
        free(vertices);
        free(normals);
        free(plane_origin);
    }


    inline sVector3* get_face(const uint32_t face_index) const {
        return &vertices[face_index * face_stride];
    }

    inline sPlane get_plane_of_face(const uint32_t face_index) const {
        return sPlane{plane_origin[face_index], normals[face_index]};
    }

    inline bool test_face_sphere_collision(const uint32_t face_index,
                                           const sVector3 &sphere_origin,
                                           const float sphere_radius) const {
        sVector3 *face_vertices = get_face(face_index);

        sPlane curr_plane = sPlane{plane_origin[face_index], normals[face_index]};

        float angle_sum = 0.0f;
        if (curr_plane.distance(sphere_origin) <= sphere_radius) {
            for(int i = 0; i < face_stride; i++) {
                int j = j % face_stride;

                angle_sum += dot_prod(face_vertices[i], face_vertices[j]);
            }

            return angle_sum <= 0.0f;
        }

        return false;
    }
};

#endif // COLLIDER-MESH_H_
