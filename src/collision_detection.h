#ifndef COLLISION_DETECTION_H_
#define COLLISION_DETECTION_H_

#include "math.h"
#include "geometry.h"
#include "types.h"
#include "vector.h"
#include <cstdint>

#define MAX_COL_POINTS 2


enum eColiderTypes : uint8_t {
    SPHERE_COLLIDER = 0,
    PLANE_COLLIDER,
    CUBE_COLLIDER,
    COLLIDER_COUNT
};


struct sCollisionManifold {
    uint8_t   obj1;
    uint8_t   obj2;

    sVector3 normal;

    sVector3 contact_points[MAX_COL_POINTS] = {};
    float    contact_depth [MAX_COL_POINTS] = {};
    int contanct_points_count = 0;
};


// ===========================
// COLLISION METHODS
// ===========================

inline bool test_cube_cube_collision(const sTransform &cube1_trasform,
                                     const sRawGeometry &cube1_geometry,
                                     const sTransform &cube2_trasform,
                                     const sRawGeometry &cube2_geometry,
                                     sCollisionManifold *manifold);

inline bool test_cube_sphere_collision(const sTransform &cube_transform,
                                       const sRawGeometry &cube_geometry,
                                       const sVector3 &sphere_center,
                                       const float radius,
                                       sCollisionManifold *manifold) {
    sVector3 sphere_direction = sphere_center.subs(cube_transform.position).normalize();
    sVector3 cube_support = cube_geometry.get_support_point(sphere_direction);
    sVector3 sphere_to_cube = cube_support.subs(sphere_center);

    float distance = sphere_to_cube.magnitude();

    if (distance < radius) {
        std::cout << distance << " " << radius << std::endl;
        manifold->normal = sphere_to_cube.normalize().mult(-1.0f);

        manifold->contact_points[0] = sphere_center.sum(manifold->normal.mult(radius));
        manifold->contact_depth[0] = distance - radius;

        manifold->contanct_points_count = 1;
        return true;
    }
    return false;
}


inline bool test_sphere_sphere_collision(const sVector3  &center1,
                                         const float radius1,
                                         const sVector3 &center2,
                                         const float radius2,
                                         sCollisionManifold *manifold) {

    sVector3 center1_to_2 = center1.subs(center2);
    float center_distance = center1_to_2.magnitude();
    float total_radius = radius1 + radius2;

    if (center_distance < total_radius) {
        // The spheres are colliding
        manifold->normal = center1_to_2.normalize().mult(-1.0f);

        manifold->contact_points[0] = center1.sum(manifold->normal.mult(radius1));
        manifold->contact_depth[0] = center_distance - total_radius;

        manifold->contanct_points_count = 1;

        return true;
    }
    return false;
}

inline bool test_plane_sphere_collision(const sVector3 &sphere_center,
                                        const float radius,
                                        const sVector3 &plane_origin,
                                        const sVector3 &plane_normal,
                                        sCollisionManifold *manifold) {
    // Based arround the signed distance of the plane
    sPlane plane;
    plane.origin_point = plane_origin;
    plane.normal = plane_normal;

    float distance = plane.distance(sphere_center) - radius;

    if (distance < 0.0f) {
        manifold->normal = plane_normal.mult(-1.0f).normalize();
        manifold->contact_depth[0] = distance;
        manifold->contact_points[0] = plane.project_point(sphere_center).sum(plane_origin.mult(distance));

        //std::cout << manifold->contact_depth[0] << std::endl;
        //std::cout << manifold->contact_points[0].x << " " << manifold->contact_points[0].y << " " << manifold->contact_points[0].z << std::endl;
        manifold->contanct_points_count = 1;

        //std::cout << "====================" << std::endl;
        return true;
    }

    return false;
}

// TODO: sphere plane function

#endif // COLLISION_DETECTION_H_
