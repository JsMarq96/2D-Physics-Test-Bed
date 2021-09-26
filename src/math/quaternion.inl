#include "quaternion.h"

#include <cmath>

inline sQuaternion4 sQuaternion4::conjugate() const {
  return sQuaternion4{w, -x, -y, -z};
}

    inline sQuaternion4 sQuaternion4::inverse() const {
        float norm = sqrt(w*w + x*x + y*y + z*z);
        return sQuaternion4{w / norm, -x / norm, -y / norm, -z / norm};
    }

    inline sQuaternion4 sQuaternion4::normalize() const {
        float norm = sqrt(w*w + x*x + y*y + z*z);
        return sQuaternion4{w / norm, x / norm, y / norm, z / norm};
    }

    inline sQuaternion4 sQuaternion4::multiply(const sQuaternion4 &q) const {
      return {(w * q.w) - (x * q.x) - (y * q.y) - (z * q.z),
              (w * q.x) + (x * q.w) - (y * q.z) + (z * q.y),
              (w * q.y) + (x * q.z) + (y * q.w) - (z * q.x),
              (w * q.z) - (x * q.y) + (y * q.x) + (z * q.w)
              };
    };

    inline sQuaternion4 sQuaternion4::multiply(const float num) const {
      return sQuaternion4{w * num, x * num, y * num, z * num};
    }

    inline sQuaternion4 sQuaternion4::sum(const sQuaternion4 &quat) const {
      return sQuaternion4{w + quat.w, x + quat.x, y + quat.y, z + quat.z};
    }
