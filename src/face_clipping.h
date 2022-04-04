#ifndef FACE_CLIPPING_H_
#define FACE_CLIPPING_H_

#include "collider_mesh.h"
#include "vector.h"
#include <cstdint>

namespace CLIPPING {
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
        return 0;
    }
};

#endif // FACE_CLIPPING_H_
