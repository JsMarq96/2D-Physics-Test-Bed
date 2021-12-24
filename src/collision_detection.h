#ifndef COLLISION_DETECTION_H_
#define COLLISION_DETECTION_H_

#include "math.h"
#include "geometry.h"
#include <cstdint>

#define MAX_COL_POINTS 2


enum eColiderTypes : uint8_t {
    SPHERE_COLLIDER = 0,
    PLANE_COLLIDER,
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
        manifold->normal = center1_to_2.normalize();

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
