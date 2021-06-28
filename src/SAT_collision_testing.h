//
// Created by jsmar on 16/06/2021.
//

#ifndef QUEST_DEMO_SAT_COLLISION_TESTING_H
#define QUEST_DEMO_SAT_COLLISION_TESTING_H

#include <float.h>

#include "math.h"
#include "geometry.h"

#include "collision_types.h"


struct sOBB {
    sVector3 vertices[4];
    int vertex_count = 4;

    sVector3 normals[4];
    int normals_count = 4;

    sPlane  face_planes[4];
    int faces_count = 4;

    void init(const sMat44 *transform, const sQuaternion4 rotation) {
        int faces_vertex_index[4][2] = {
            {1, 3},
            {2, 3},
            {0, 2},
            {0, 1}
        };

        vertices[0] = sVector3{};
        vertices[1] = {1.0f, 0.0f, 0.0f};
        vertices[2] = {0.0f, 1.0f, 0.0f};
        vertices[3] = {1.0f, 1.0f, 0.0f};

        for(int i = 0; i < 4; i++) {
            vertices[i] = transform->multiply(vertices[i]);
            std::cout << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << std::endl;
        }
        std::cout << "============" << std::endl;

        normals[0] = rotate_vector3(sVector3{1.0f, 0.0f, 0.0f}, rotation);
        normals[1] = rotate_vector3(sVector3{0.0f, 1.0f, 0.0f}, rotation);
        normals[2] = rotate_vector3(sVector3{-1.0f, 0.0f, 0.0f}, rotation);
        normals[3] = rotate_vector3(sVector3{0.0f, -1.0f, 0.0f}, rotation);

        for(int i = 0; i < 4; i++) {
            std::cout << normals[i].x << " " << normals[i].y << " " << normals[i].z << std::endl;
        }
        std::cout << "++++++++++++++++" << std::endl;
        vertex_count = 4;
        normals_count = 4;
        faces_count = 4;
        // Gerate vertex planes
        for(int i = 0; i < faces_count; i++) {
            face_planes[i].origin_point = sVector3{
                        vertices[faces_vertex_index[i][0]].x + vertices[faces_vertex_index[i][1]].x / 2.0f, 
                        vertices[faces_vertex_index[i][0]].y + vertices[faces_vertex_index[i][1]].y / 2.0f, 
                        vertices[faces_vertex_index[i][0]].z + vertices[faces_vertex_index[i][1]].z / 2.0f};
            face_planes[i].normal = normals[i];
            std::cout << face_planes[i].origin_point.x << " " << face_planes[i].origin_point.y << " " << face_planes[i].origin_point.z << std::endl;
        }
        std::cout << "####################" << std::endl;
    }
};

inline void get_OBB_raw_vertex(const sMat44 transform,
                               sVector3 *result) {
    result[0] = sVector3{};
    result[1] = {1.0f, 0.0f, 0.0f};
    result[2] = {0.0f, 1.0f, 0.0f};
    result[3] = {1.0f, 1.0f, 0.0f};

    for(int i = 0; i < 8; i++) {
        result[i] = transform.multiply(result[i]);
    }
}


inline sVector3 get_support_of_shape(const sVector3 direction, const sVector3 *vertexs, const int vertex_count) {
    sVector3 best_vect{};
    float best_proj = FLT_MIN;
    
    for(int i = 0; i < vertex_count; i++) {
        float proj = dot_prod(vertexs[i], direction);

        if (proj > best_proj) {
            best_vect = vertexs[i];
            best_proj = proj;
        }
    }

    return best_vect;
}

inline float SAT_test_face_directions(const sOBB *poly_a, const sOBB *poly_b, int *selected_face_index) {
    float max_distance = FLT_MIN;
    int max_face_index = -1;
    for(int i = 0; i < poly_a->faces_count; i++) {
        sVector3 support_point = get_support_of_shape(poly_a->normals[i], poly_b->vertices, poly_b->vertex_count);

        float distance = poly_a->face_planes[i].distance(support_point);

        if (distance > max_distance) {
            max_distance = distance;
            max_face_index = i;
        }
    }

    *selected_face_index = max_face_index;
    return max_distance;
}

inline bool SAT_test_OBB(const sMat44 *obb1_transf, const sQuaternion4 rot_1, const sMat44 *obb2_transf, const sQuaternion4 rot_2) {
    int coll_face_index_a, coll_face_index_b;
    sOBB obb_a, obb_b;

    obb_a.init(obb1_transf, rot_1);
    obb_b.init(obb2_transf, rot_2);

    float coll_face_a_dist = SAT_test_face_directions(&obb_a, &obb_b, &coll_face_index_a);
    if (coll_face_a_dist > 0.0f) {
        return false;
    }

    float coll_face_b_dist = SAT_test_face_directions(&obb_b, &obb_a, &coll_face_index_b);
    if (coll_face_b_dist > 0.0f) {
        return false;
    }

    return true;
}


inline bool intersect_vertex_group_on_axis(const sVector3 obb1[8],
                                           const sVector3 obb2[8],
                                           const sVector3 axis,
                                           float          *diff) {
    float min_1 = FLT_MAX;
    float max_1 = FLT_MIN;
    float min_2 = FLT_MAX;
    float max_2 = FLT_MIN;

    if (axis.x == 0 && axis.y == 0 && axis.z == 0) {
        return true;
    }

    // Get the min and max position for each vertex group
    // proyected on teh vertex
    for (int i = 0; i < 8; i++) {
        float dist_1 = dot_prod(obb1[i], axis);
        float dist_2 = dot_prod(obb2[i], axis);
        min_1 = (dist_1 < min_1) ? dist_1 : min_1;
        max_1 = (dist_1 > max_1) ? dist_1 : max_1;
        min_2 = (dist_2 < min_2) ? dist_2 : min_2;
        max_2 = (dist_2 > max_2) ? dist_2 : max_2;
    }

    float total_span = MAX(max_1, max_2) - MIN(min_1, min_2);
    float sum = max_1 - min_1 + max_2 - min_2;

    *diff = sum - total_span;

    //std::cout << *diff << std::endl;

    return sum >= total_span;
}


inline bool SAT_OBB_v_OBB(const sMat44 obb1_transform,
                          const sQuaternion4 obb1_rotation,
                          const sMat44 obb2_transform,
                          const sQuaternion4 obb2_rotation,
                          sCollisionManifold  *result_manifold) {
    int OBB_faces_indexing[6][4] = {
            {0, 1, 2, 3},
            {6, 7, 3, 1},
            {2, 3, 7, 5},
            {5, 4, 7, 6},
            {4, 6, 0, 1},
            {5, 4, 2, 0}
    };
    sVector3 obb1_vertex[8] = {};
    sVector3 obb2_vertex[8] = {};

    sVector3 col_normal{};

    get_OBB_raw_vertex(obb1_transform,
                       &obb1_vertex[0]);
    get_OBB_raw_vertex(obb2_transform,
                       &obb2_vertex[0]);

    sVector3 norms_1[] = {
            rotate_vector3(sVector3{1.0f, 0.0f, 0.0f}, obb1_rotation),
            rotate_vector3(sVector3{0.0f, 1.0f, 0.0f}, obb1_rotation),

            rotate_vector3(sVector3{-1.0f, 0.0f, 0.0f}, obb1_rotation),
            rotate_vector3(sVector3{0.0f, -1.0f, 0.0f}, obb1_rotation)
    };

    sVector3 norms_2[] = {
            rotate_vector3(sVector3{1.0f, 0.0f, 0.0f}, obb2_rotation),
            rotate_vector3(sVector3{0.0f, 1.0f, 0.0f}, obb2_rotation),

            rotate_vector3(sVector3{-1.0f, 0.0f, 0.0f}, obb2_rotation),
            rotate_vector3(sVector3{0.0f, -1.0f, 0.0f}, obb2_rotation)
    };

    float min_diff = FLT_MAX;
    float diff = FLT_MAX;
    int col_case = -1;

    // Test all axis, since we need the face too
    // Test normals of OBB1
    for (int i = 0; i < 6; i++) {
        if(!intersect_vertex_group_on_axis(obb1_vertex,
                                           obb2_vertex,
                                           norms_1[i],
                                           &diff)) {
            return false;
        }

        if (min_diff > diff) {
            col_normal = norms_1[i];
            min_diff = diff;
            col_case = i;
        }
    }

    // Test normals of OBB2
    for (int i = 0; i < 6; i++) {
        if(!intersect_vertex_group_on_axis(obb1_vertex,
                                           obb2_vertex,
                                           norms_1[i],
                                           &diff)) {
            return false;
        }

        if (min_diff > diff) {
            col_normal = norms_1[i];
            min_diff = diff;
            col_case = i + 6;
        }
    }

    // TODO: test corners


    /// http://www.randygaul.net/2013/03/28/custom-physics-engine-part-2-manifold-generation/

    sVector3 reference_face[4] = {};
    sVector3 indent_face[4] = {};

    int reference_face_index = 0;
    int indent_face_index = -1;

    // Get the Incident face
    if (col_case < 6) {
        // The reference face is on OBB1
        float min_dot = FLT_MAX;
        float dot = 0.f;

        for(int i = 0; i < 6; i++) {
            dot = dot_prod(col_normal, norms_2[i]);
            if (dot < min_dot) {
                min_dot = dot;
                indent_face_index = i;
            }
        }

        reference_face_index = col_case;

        reference_face[0] = obb1_vertex[OBB_faces_indexing[reference_face_index][0]];
        reference_face[1] = obb1_vertex[OBB_faces_indexing[reference_face_index][1]];
        reference_face[2] = obb1_vertex[OBB_faces_indexing[reference_face_index][2]];
        reference_face[3] = obb1_vertex[OBB_faces_indexing[reference_face_index][3]];

        indent_face[0] = obb2_vertex[OBB_faces_indexing[indent_face_index][0]];
        indent_face[1] = obb2_vertex[OBB_faces_indexing[indent_face_index][1]];
        indent_face[2] = obb2_vertex[OBB_faces_indexing[indent_face_index][2]];
        indent_face[3] = obb2_vertex[OBB_faces_indexing[indent_face_index][3]];

    } else if (col_case < 12) {
        // The reference face is on OBB2
        float min_dot = FLT_MAX;
        float dot = 0.0f;

        for(int i = 0; i < 6; i++) {
            dot = dot_prod(col_normal, norms_1[i]);
            if (dot < min_dot) {
                min_dot = dot;
                indent_face_index = i;
            }
        }

        reference_face_index = col_case - 6;

        reference_face[0] = obb2_vertex[OBB_faces_indexing[reference_face_index][0]];
        reference_face[1] = obb2_vertex[OBB_faces_indexing[reference_face_index][1]];
        reference_face[2] = obb2_vertex[OBB_faces_indexing[reference_face_index][2]];
        reference_face[3] = obb2_vertex[OBB_faces_indexing[reference_face_index][3]];

        indent_face[0] = obb1_vertex[OBB_faces_indexing[indent_face_index][0]];
        indent_face[1] = obb1_vertex[OBB_faces_indexing[indent_face_index][1]];
        indent_face[2] = obb1_vertex[OBB_faces_indexing[indent_face_index][2]];
        indent_face[3] = obb1_vertex[OBB_faces_indexing[indent_face_index][3]];
    }

    int index = 0;

    sPlane refence_plane;

    refence_plane.origin_point = sVector3{
            (reference_face[0].x + reference_face[1].x + reference_face[2].x + reference_face[3].x) / 4.0f,
            (reference_face[0].y + reference_face[1].y + reference_face[2].y + reference_face[3].y) / 4.0f,
            (reference_face[0].z + reference_face[1].z + reference_face[2].z + reference_face[3].z) / 4.0f
    };

    refence_plane.normal = col_normal;

    // Skip clipping for OBBs... Lets see how it goes

    for(int i = 0; i < 4; i++) {
        float dist = refence_plane.distance(indent_face[i]);
        if (dist <= 0.0f) {
            result_manifold->points_depth[index] = dist;
            result_manifold->contact_points[index++] = indent_face[i];
        }
    }

    result_manifold->contact_point_count = index;
    result_manifold->collision_normal = col_normal;

    return true;
}

/*

https://ia801303.us.archive.org/30/items/GDC2013Gregorius/GDC2013-Gregorius.pdf
https://www.randygaul.net/2013/03/28/custom-physics-engine-part-2-manifold-generation/
https://gamedevelopment.tutsplus.com/tutorials/understanding-sutherland-hodgman-clipping-for-physics-engines--gamedev-11917
Numerical boutsness


inline float get_axis_overlap(const float size1,
                              const float size2,
                              const float min_distance){
    return (size1 / 2.0f) + (size2 / 2.0f) - ABS(min_distance);
}

inline bool SAT_AABB_AABB_collision(const sVector3        aabb1_center,
                                    const sVector3        aabb1_size,
                                    const sVector3        aabb2_center,
                                    const sVector3        aabb2_size,
                                    sCollisionManifold    *result) {
    if (aabb1_center.x <= (aabb2_center.x + aabb2_size.x) && (aabb1_center.x + aabb1_size.x) >= aabb2_center.x) {
        return false;
    }
    if (aabb1_center.y <= (aabb2_center.y + aabb2_size.y) && (aabb1_center.y + aabb1_size.y) >= aabb2_center.y) {
        return false;
    }
    if (aabb1_center.z <= (aabb2_center.z + aabb2_size.z) && (aabb1_center.z + aabb1_size.z) >= aabb2_center.z) {
        return false;
    }

    // Fill collision manifold
    float overlap_x = get_axis_overlap(aabb1_size.x,
                                       aabb2_size.x,
                                       aabb1_center.x - aabb1_center.x);

    float overlap_y = get_axis_overlap(aabb1_size.y,
                                       aabb2_size.y,
                                       aabb1_center.y - aabb1_center.y);

    float overlap_z = get_axis_overlap(aabb1_size.z,
                                       aabb2_size.z,
                                       aabb1_center.z - aabb1_center.z);

    return true;
}*/

#endif //QUEST_DEMO_SAT_COLLISION_TESTING_H
