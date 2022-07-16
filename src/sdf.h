#ifndef SDF_H_
#define SDF_H_

#include "math.h"
#include "vector.h"

namespace SDF {

    inline float SIGN(const float x) {
        return (x > 0.0f) ? 1.0f : -1.0f;
    }

    inline float CLAMP(const float x, const float min, const float max) {
        return (x > max) ? max : ((x < min) ? min : x);
    }

    inline float MIN(const float x1, const float x2) {
        return (x1 > x2) ? x2 : x1;
    }


    // Based arround https://iquilezles.org/articles/triangledistance/
    inline float triangle(const sVector3& point,
                          const sVector3& v1,
                          const sVector3& v2,
                          const sVector3& v3,
                          const sVector3& normal) {
        const sVector3 v21 = v2.subs(v1), p1 = point.subs(v1);
        const sVector3 v32 = v3.subs(v2), p2 = point.subs(v2);
        const sVector3 v13 = v1.subs(v3), p3 = point.subs(v3);

        float k = SIGN(dot_prod(cross_prod(v21, normal), p1));
        k += SIGN(dot_prod(cross_prod(v32, normal), p2));
        k += SIGN(dot_prod(cross_prod(v13, normal), p3));

        if (k < 2.0f) {
            // Inside distance
            const sVector3 d1 = v21.mult(CLAMP(dot_prod(v21, p1)/dot_prod(v21, v21), 0.0f, 1.0f)).subs(p1);
            const sVector3 d2 = v32.mult(CLAMP(dot_prod(v32, p2)/dot_prod(v32, v32), 0.0f, 1.0f)).subs(p2);
            const sVector3 d3 = v13.mult(CLAMP(dot_prod(v13, p3)/dot_prod(v13, v13), 0.0f, 1.0f)).subs(p3);
            return MIN( MIN(dot_prod(d1, d1), dot_prod(d1, d2) ), dot_prod(d3, d3) );
        }

        // Outisde test
        return dot_prod(normal, p1) * dot_prod(normal, p1) / dot_prod(normal, normal);
    }

    // Perform an sdf against 2 triangles
    inline float quad(const sVector3& point,
                      const sVector3& v1,
                      const sVector3& v2,
                      const sVector3& v3,
                      const sVector3& v4,
                      const sVector3& normal) {
        return MIN(triangle(point,
                            v1,
                            v2,
                            v3,
                            normal),
                   triangle(point,
                            v2,
                            v3,
                            v4,
                            normal));
    }
};

#endif // SDF_H_
