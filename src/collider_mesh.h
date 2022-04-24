#ifndef COLLIDER_MESH_H_
#define COLLIDER_MESH_H_

#include "kv_storage.h"
#include "math.h"
#include "mesh.h"
#include "transform.h"
#include "vector.h"
#include "geometry.h"
#include <cstddef>
#include <cstdint>

enum eFaceSize : uint32_t {
    FACE_TRI = 3,
    FACE_QUAD = 4
};

struct sEdgeIndexTuple {
    uint32_t x;
    uint32_t y;

    inline bool is_equal(const sEdgeIndexTuple &edge) const {
        return (x == edge.x && y == edge.y) || (x == edge.y && y == edge.x);
    }
};

// TODO: Parse an obj or a mesh object into a colliderMesh

struct sColliderMesh {
    sVector3   *vertices = NULL;
    sVector3   *normals = NULL;
    sVector3   *plane_origin = NULL;
    sEdgeIndexTuple *edges = NULL;
    uint32_t   *face_connections = NULL;

    uint32_t   vertices_count = 0;
    uint32_t   face_count = 0;
    uint32_t   edge_cout = 0;

    eFaceSize  face_stride = FACE_QUAD;

    sVector3 mesh_center = {};


    void load_collider_mesh(const sMesh &mesh) {
        vertices = (sVector3*) malloc(sizeof(sVector3) * mesh.indexing_count);
        normals = (sVector3*) malloc(sizeof(sVector3) * mesh.face_count);
        plane_origin = (sVector3*) malloc(sizeof(sVector3) * mesh.face_count);

        vertices_count = mesh.indexing_count;
        face_count = mesh.face_count;
        face_stride = FACE_TRI; // Is a triangled mesh

        // Load vertices & calculate center (avg point)
        mesh_center = {0.0f, 0.0f, 0.0f};
        for(uint32_t i = 0; i < mesh.indexing_count; i++) {
            vertices[i] = mesh.vertices[mesh.vertices_index[i]].vertex;
            mesh_center = mesh_center.sum(vertices[i]);
        }

        mesh_center = mesh_center.mult(1.0f / mesh.indexing_count);

        // Load faces
        for(uint32_t i = 0; i < mesh.face_count; i++) {
            sVector3 face_plane_center = {0.0f, 0.0f, 0.0f};

            // Compute the middle point of the face
            face_plane_center = face_plane_center.sum(vertices[(i * face_stride)]);
            face_plane_center = face_plane_center.sum(vertices[(i * face_stride) + 1]);
            face_plane_center = face_plane_center.sum(vertices[(i * face_stride) + 2]);

            plane_origin[i] = face_plane_center.mult(1.0f / face_stride);

            normals[i] = mesh.face_normals[i].normalize();
        }

        // Store Edges
        // TODO:  umber of edges?
        edges = (sEdgeIndexTuple*) malloc(sizeof(sEdgeIndexTuple) * face_count * 2);
        edge_cout = 0;

        uint32_t *edge_face_connections = (uint32_t*) malloc(sizeof(uint32_t) * face_count * 4);

        face_connections = (uint32_t*) malloc(sizeof(uint32_t) * 3 * face_count);
        uint32_t *face_conn_count = (uint32_t*) malloc(sizeof(uint32_t) * face_count);
        memset(face_conn_count, 0, sizeof(uint32_t) * face_count);

        // TODO: this is not very efficient... Better way?
        // Iterate throught every face, and thrugh every vertex
        // If there has not beel included, then add them to the list
        // Maybe a use of a map or a set to speedup..?
        // Maybe a hald-edfe can do a bit better... TODO
        for(uint32_t i = 0; i < face_count; i++) {
            uint32_t curr_face_index = i * face_stride;

            const uint32_t *current_face = &mesh.face_vertices[curr_face_index];

            for(uint32_t v1 = 0; v1 < face_stride; v1++) {
                uint32_t v2 = (v1 + 1) % face_stride;

                sEdgeIndexTuple curr_edge = {current_face[v1], current_face[v2]};
                bool is_inside = false;
                sVector3 vec1 = vertices[curr_edge.x];
                sVector3 vec2 = vertices[curr_edge.y];

                // iterate thrugh to all the edges, to test if its inside
                uint32_t ed = 0;
                for(; ed < edge_cout; ed++) {
                    sEdgeIndexTuple &edge_to_test = edges[ed];

                    bool is_equal = vertices[edge_to_test.x].is_equal(vec1) && vertices[edge_to_test.y].is_equal(vec2);
                    is_equal = is_equal || (vertices[edge_to_test.x].is_equal(vec2) && vertices[edge_to_test.y].is_equal(vec1));

                    if (is_equal) {
                        is_inside = true;
                        break;
                    }
                }

                // For the edge face connections, if it inside, then it is
                // already on the list, and is in the first of the tuple list
                // so we use it, and set the other part of the tuples,
                // since this indicates that the current edge, coneccets both
                // faces
                if (!is_inside) {
                    edge_face_connections[edge_cout] = i;
                    edges[edge_cout++] = curr_edge;
                    //std::cout << edge_cout << '/' << face_count * 2 <<   std::endl;
                } else {
                    uint32_t neighboor_face = edge_face_connections[ed];
                    face_connections[(i*3) + face_conn_count[i]] = neighboor_face;
                    face_connections[(neighboor_face * 3) + face_conn_count[neighboor_face]] = i;

                    face_conn_count[neighboor_face]++;
                    face_conn_count[i]++;
                }
            }
        }
        free(face_conn_count);
        free(edge_face_connections);
    }

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

        mesh_center = transform.apply({0.0f, 0.0f, 0.0f});

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
            normals[i] = center.subs(mesh_center).normalize();
        }


        face_stride = FACE_QUAD;


        // Edge extraction
        edges = (sEdgeIndexTuple*) malloc(sizeof(sEdgeIndexTuple) * face_count * 2);
        edge_cout = 0;

        uint32_t *edge_face_connections = (uint32_t*) malloc(sizeof(uint32_t) * face_count * 4);

        face_connections = (uint32_t*) malloc(sizeof(uint32_t) * 4 * face_count);
        uint32_t *face_conn_count = (uint32_t*) malloc(sizeof(uint32_t) * face_count);
        memset(face_conn_count, 0, sizeof(uint32_t) * face_count);

        // TODO: this is not very efficient... Better way?
        // Iterate throught every face, and thrugh every vertex
        // If there has not beel included, then add them to the list
        // Maybe a use of a map or a set to speedup..?
        // Maybe a hald-edfe can do a bit better... TODO
        for(uint32_t i = 0; i < face_count; i++) {
            uint32_t curr_face_index = i * face_stride;

            for(uint32_t v1 = 0; v1 < face_stride; v1++) {
                uint32_t v2 = (v1 + 1) % face_stride;

                sEdgeIndexTuple curr_edge = {curr_face_index + v1, curr_face_index + v2};
                bool is_inside = false;
                sVector3 vec1 = vertices[curr_edge.x];
                sVector3 vec2 = vertices[curr_edge.y];

                // iterate thrugh to all the edges, to test if its inside
                uint32_t edge_it = 0;
                for(; edge_it < edge_cout; edge_it++) {
                    sEdgeIndexTuple &edge_to_test = edges[edge_it];

                    bool is_equal = vertices[edge_to_test.x].is_equal(vec1) && vertices[edge_to_test.y].is_equal(vec2);
                    is_equal = is_equal || (vertices[edge_to_test.x].is_equal(vec2) && vertices[edge_to_test.y].is_equal(vec1));

                    if (is_equal) {
                        is_inside = true;
                        break;
                    }
                }

                if (!is_inside) {
                    edge_face_connections[edge_cout] = i;
                    edges[edge_cout++] = curr_edge;
                    //std::cout << edge_cout << '/' << face_count * 2 <<   std::endl;
                }  else {
                    uint32_t neighboor_face = edge_face_connections[edge_it];
                    face_connections[(i*4) + face_conn_count[i]] = neighboor_face;
                    face_connections[(neighboor_face * 4) + face_conn_count[neighboor_face]] = i;

                    face_conn_count[neighboor_face]++;
                    face_conn_count[i]++;
                }

            }
        }
        free(face_conn_count);
        free(edge_face_connections);
    }

    void clean() {
        free(vertices);
        free(normals);
        free(plane_origin);
        free(edges);
        free(face_connections);
    }

    void apply_transform(const sTransform &transf) {
        for(int i = 0; i < vertices_count; i++) {
            vertices[i] = transf.apply(vertices[i]);
        }

        for(int i = 0; i < face_count; i++) {
            normals[i] = transf.apply_rotation(normals[i]);
            plane_origin[i] = transf.apply(plane_origin[i]);
        }

        std::cout << vertices[0].x << " " << std::endl;
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

    inline uint32_t get_neighboor_of_face(const uint32_t face_id,
                                          const uint32_t neighboor) const {
        return face_connections[(face_id * face_stride) + neighboor];
    }

    inline sVector3* get_face(const uint32_t face_index) const {
        return &vertices[face_index * face_stride];
    }

    inline sPlane get_plane_of_face(const uint32_t face_index) const {
        return sPlane{plane_origin[face_index], normals[face_index]};
    }

    inline sVector3 get_edge(const uint32_t edge_id) const {
        sEdgeIndexTuple edge_indices = edges[edge_id];
        return vertices[edge_indices.x].subs(vertices[edge_indices.y]);
    }

    inline bool test_face_sphere_collision(const uint32_t face_index,
                                           const sVector3 &sphere_origin,
                                           const float sphere_radius,
                                           float *distance) const {
        sVector3 *face_vertices = get_face(face_index);

        sPlane curr_plane = sPlane{plane_origin[face_index], normals[face_index]};

        float plane_distance = curr_plane.distance(sphere_origin);

        if (plane_distance <= sphere_radius) {
            sVector3 origin_on_plane = curr_plane.project_point(sphere_origin);

            // iF the total angle is arround 360 de grees or 6.28 rads, is inside
            float angle_sum = 0.00;
            for(uint32_t i = 0; i <= face_stride; i++) {
                uint32_t j = (i+1) % face_stride;
                sVector3 v1 = curr_plane.project_point(face_vertices[i]).subs(origin_on_plane);
                sVector3 v2 = curr_plane.project_point(face_vertices[j]).subs(origin_on_plane);

                 angle_sum += acos(dot_prod(v1,v2) / (v1.magnitude() * v2.magnitude()));
            }
            *distance =plane_distance - sphere_radius;
            //std::cout << plane_distance - sphere_radius  << " ini " << angle_sum<< std::endl;
            return angle_sum >= 6.2f;
        }

        return false;
    }
};

#endif // COLLIDER-MESH_H_
