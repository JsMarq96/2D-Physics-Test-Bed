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
        return dot_prod(normal, sVector3{  origin_point.x - p.x,  origin_point.y - p.y, origin_point.z - p.z });
    }
};


#endif // __GEOMETRY_H_