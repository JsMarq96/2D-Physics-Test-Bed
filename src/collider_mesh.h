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

struct sEdgeTuple {
    uint32_t x;
    uint32_t y;

    inline bool is_equal(const sEdgeTuple &edge) const {
        return (x == edge.x && y == edge.y) || (x == edge.y && y == edge.x);
    }
};

// TODO: Parse an obj or a mesh object into a colliderMesh

struct sColliderMesh {
    sVector3   *vertices = NULL;
    sVector3   *normals = NULL;
    sVector3   *plane_origin = NULL;
    sEdgeTuple *edges = NULL;

    uint32_t   vertices_count = 0;
    uint32_t   face_count = 0;
    uint32_t   edge_cout = 0;

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
        raw_points[0] = transform.apply(sVector3{0.0f, 0.0f, 0.0f}.sum({-0.5f, -0.5f, -0.5f}));
        raw_points[1] = transform.apply(sVector3{1.0f, 0.0f, 0.0f}.sum({-0.5f, -0.5f, -0.5f}));
        raw_points[2] = transform.apply(sVector3{0.0f, 1.0f, 0.0f}.sum({-0.5f, -0.5f, -0.5f}));
        raw_points[3] = transform.apply(sVector3{1.0f, 1.0f, 0.0f}.sum({-0.5f, -0.5f, -0.5f}));
        raw_points[4] = transform.apply(sVector3{0.0f, 0.0f, 1.0f}.sum({-0.5f, -0.5f, -0.5f}));
        raw_points[5] = transform.apply(sVector3{1.0f, 0.0f, 1.0f}.sum({-0.5f, -0.5f, -0.5f}));
        raw_points[6] = transform.apply(sVector3{0.0f, 1.0f, 1.0f}.sum({-0.5f, -0.5f, -0.5f}));
        raw_points[7] = transform.apply(sVector3{1.0f, 1.0f, 1.0f}.sum({-0.5f, -0.5f, -0.5f}));

        for(uint32_t i = 0; i < 6*4; i++) {
            vertices[i] = raw_points[box_LUT_vertices[i]];
        }

        sVector3 cuboid_center = transform.apply({0.5f, 0.5f, 0.5f});

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

            // The plane normal goes from the center  of the cuboid to the
            // origin of the plane
            normals[i] = center.subs(cuboid_center).normalize();
        }

        // Face normals
        /*normals[0] = transform.apply_rotation(sVector3{0.0f, 0.0f, 1.0f});
        normals[1] = transform.apply_rotation(sVector3{0.0f, 1.0f, 0.0f});
        normals[2] = transform.apply_rotation(sVector3{1.0f, 0.0f, 0.0f});
        normals[3] = transform.apply_rotation(sVector3{0.0f, 0.0f, -1.0f});
        normals[4] = transform.apply_rotation(sVector3{0.0f, -1.0f, 0.0f});
        normals[5] = transform.apply_rotation(sVector3{-1.0f, 0.0f, 0.0f});*/

        face_stride = FACE_QUAD;

        // Edge extraction
        edges = (sEdgeTuple*) malloc(sizeof(sEdgeTuple) * face_count * 2);
        edge_cout = 0;
        // TODO: this is not very efficient... Better way?
        // Iterate throught every face, and thrugh every vertex
        // If there has not beel included, then add them to the list
        // Maybe a use of a map or a set to speedup..?
        // Maybe a hald-edfe can do a bit better... TODO
        for(uint32_t i = 0; i < face_count; i++) {
            uint32_t curr_face_index = i * face_stride;

            for(uint32_t v1 = 0; v1 < face_stride; v1++) {
                uint32_t v2 = (v1 + 1) % face_stride;

                sEdgeTuple curr_edge = {curr_face_index + v1, curr_face_index + v2};
                bool is_inside = false;
                sVector3 vec1 = vertices[curr_edge.x];
                sVector3 vec2 = vertices[curr_edge.y];

                // iterate thrugh to all the edges, to test if its inside
                for(uint32_t ed = 0; ed < edge_cout; ed++) {
                    sEdgeTuple &edge_to_test = edges[ed];

                    bool is_equal = vertices[edge_to_test.x].is_equal(vec1) && vertices[edge_to_test.y].is_equal(vec2);
                    is_equal = is_equal || (vertices[edge_to_test.x].is_equal(vec2) && vertices[edge_to_test.y].is_equal(vec1));

                    if (is_equal) {
                        is_inside = true;
                        break;
                    }
                }

                if (!is_inside) {
                    edges[edge_cout++] = curr_edge;
                }
            }
        }
    }

    void clean() {
        free(vertices);
        free(normals);
        free(plane_origin);
        free(edges);
    }

    void apply_transform(const sTransform &transf) {
        for(int i = 0; i < vertices_count; i++) {
            vertices[i] = transf.apply(vertices[i]);
        }

        for(int i = 0; i < face_count; i++) {
            normals[i] = transf.apply_rotation(normals[i]);
            plane_origin[i] = transf.apply(plane_origin[i]);
        }
    }

    inline sVector3 get_support(const sVector3 &direction) const {
        float best_projection = -FLT_MAX;
        uint32_t support_index = 0;

        for(int i = 0; i < vertices_count; i++) {
            float projection = dot_prod(vertices[i], direction);

            if (projection > best_projection) {
                support_index = i;
                best_projection = projection;
            }
        }

        return vertices[support_index];
    }


    inline sVector3* get_face(const uint32_t face_index) const {
        return &vertices[face_index * face_stride];
    }

    inline sPlane get_plane_of_face(const uint32_t face_index) const {
        return sPlane{plane_origin[face_index], normals[face_index]};
    }

    inline sVector3 get_edge(const uint32_t edge_id) const {
        sEdgeTuple edge_indices = edges[edge_id];
        return vertices[edge_indices.x].subs(vertices[edge_indices.y]);
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
