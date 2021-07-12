#ifndef __GEOMETRY_H_
#define __GEOMETRY_H_

#include "math.h"

struct sLineSegment {
    sVector3  p1;
    sVector3  p2;
};

struct sPlane {
    sVector3  origin_point = sVector3{};
    sVector3  normal       = sVector3{};

    inline float distance(const sVector3 p) const {
        return dot_prod(normal, sVector3{  p.x - origin_point.x,  p.y - origin_point.y, p.z - origin_point.z });
    }

    inline void apply_transform(const sMat44 *transf) {
      /*sMat44 rot_mat;
      memcpy(&rot_mat, transf, sizeof(sMat44));
      rot_mat.set_position(sVector3{0.0f, 0.0f, 0.0f});
*/
      origin_point = transf->multiply(origin_point);
      // normal = rotate_vector3;
      //TODO: rotate normal
    }
};


#endif // __GEOMETRY_H_
