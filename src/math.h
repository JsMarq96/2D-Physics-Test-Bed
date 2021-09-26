//
// Created by jsmar on 20/05/2021.
//
#ifndef __MATH_H_
#define __MATH_H_

/**
 * Single header Math library with usefull utilities and structures
 * by Juan S. Marquerie
 * */


#include "math/vector.h"
#include "math/matrix33.h"
#include "math/matrix44.h"
#include "math/quaternion.h"

// TODO: cleanup a bit, its getting a bit messy bro
// tired fixing it a bit

//// VECTOR QUATERNION FUNCTIONS
inline sQuaternion4 sVector3::get_pure_quaternion() const {
  return sQuaternion4{0.0f, x, y, z};
}

inline sVector3 sQuaternion4::get_vector() const {
        return {x, y, z};
    }


//// VECTOR MATRIX FUNCTIONS
inline sVector3 sVector3::rotate(const sQuaternion4 &quat) const {
        sMat44 rot = {};
        rot.convert_quaternion_to_matrix(quat);
        sVector4 v2 {x, y, z, 1.0f};

        sVector4 yt = rot.multiply(v2);

        return sVector3{yt.x, yt.y, yt.z};
    }



//// FUNCTIONS
inline float ABS(float x) { return (x < 0.0f) ? x * -1.0f : x; }
inline float MAX(float x, float y) { return (x >= y) ? x : y; }
inline float MIN(float x, float y) { return (x < y) ? x : y; }
inline int MAX(int x, int y) { return (x >= y) ? x : y; }
inline int MIN(int x, int y) { return (x < y) ? x : y; }

inline float LERP(const float a,
                  const float b,
                  const float alpha) {
  return a + (b - a) * alpha;
}
inline sVector3 LERP_3D(const sVector3 &v1,
                        const sVector3 &v2,
                        const float alpha) {
  return sVector3{LERP(v1.x, v2.x, alpha),
                  LERP(v1.y, v2.y, alpha),
                  LERP(v1.z, v2.z, alpha) };
}

inline float to_radians(float degree) { return degree * (M_PI / 180.0); }

inline float abs_diff(const float  x,
                      const float  y) {
    return (x > y) ? x - y : y - x;
}

inline float dot_prod(const sQuaternion4& quat1,
                      const sQuaternion4& quat2) {
  return quat1.w * quat2.w + quat1.x * quat2.x + quat1.y * quat2.y + quat1.z * quat2.z;
}

inline float dot_prod(const sVector3 &v1, const sVector3 &v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline sVector3 cross_prod(const sVector3 &v1, const sVector3 &v2) {
    return sVector3{v1.y * v2.z - v1.z * v2.y,
                    v1.z * v2.x - v1.x * v2.z,
                    v1.x * v2.y - v1.y * v2.x};
}


#endif // __MATH_H_
