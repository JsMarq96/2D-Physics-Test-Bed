#include <iostream>
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

inline float dot_prod(const sVector3 &v1, const sVector3 &v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline sVector3 cross_prod(const sVector3 &v1, const sVector3 &v2) {
    return sVector3{v1.y * v2.z - v1.z * v2.y,
                    v1.z * v2.x - v1.x * v2.z,
                    v1.x * v2.y - v1.y * v2.x};
}

inline sVector3 rotate_vector3(const sVector3 v, const sQuaternion4 quat) {
    sMat44 rot;
    convert_quaternion_to_matrix(&quat, &rot);
    sVector4 v2 {v.x, v.y, v.z, 1.0f};

    sVector4 yt = rot.multiply(v2);

    return sVector3{yt.x, yt.y, yt.z};
    /*sVector3 result{};
    sVector3 q = sVector3{quat.x, quat.y, quat.z};
    float s = quat.w;

    sVector3 t = cross_prod(q, v);
    t.x *= 2.0f;
    t.y *= 2.0f;
    t.z *= 2.0f;

    sVector3 tmp = cross_prod(q, t);

    result.x = v.x + (s * t.x) + tmp.x;
    result.y = v.y + (s * t.y) + tmp.y;
    result.z = v.z + (s * t.z) + tmp.z;

    return result;*/
}
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
inline void
convert_quaternion_to_matrix(const sQuaternion4 *quat, sMat44 *mat) {
    mat->set_identity();
    mat->mat_values[0][0] = 1.0f - (2.0f * (quat->y * quat->y + quat->z * quat->z));
    mat->mat_values[1][0] = 2.0f * (quat->x * quat->y + quat->z * quat->w);
    mat->mat_values[2][0] = 2.0f * (quat->x * quat->z - quat->y * quat->w);

    mat->mat_values[0][1] = 2.0f * (quat->x * quat->y - quat->z * quat->w);
    mat->mat_values[1][1] = 1.0f - (2.0f * (quat->x * quat->x - quat->z * quat->z));
    mat->mat_values[2][1] = 2.0f * (quat->y * quat->z + quat->x * quat->w);

    mat->mat_values[0][2] = 2.0f * (quat->x * quat->z + quat->y * quat->w);
    mat->mat_values[1][2] = 2.0f * (quat->y * quat->z - quat->x * quat->w);
    mat->mat_values[2][2] = 1.0f - (2.0f * (quat->x * quat->x + quat->y * quat->y));
    mat->mat_values[3][3] = 1.0f;
    mat->transpose();
}

// QUATERNION =============================

    inline sQuaternion4 sQuaternion4::inverse() {
        float norm = w*w + x*x + y*y + z*z;
        return sQuaternion4{w / norm, -x / norm, -y / norm, -z / norm};
    }

    inline sQuaternion4 sQuaternion4::normalize() {
        float norm = w*w + x*x + y*y + z*z;
        return sQuaternion4{w / norm, x / norm, y / norm, z / norm};
    }

    inline sQuaternion4 sQuaternion4::multiply(const sQuaternion4 &q) const {
      return {(w * q.x) + (x * q.w) + (y * q.z) - (z * q.y),
              (w * q.y) + (y * q.w) + (z * q.x) - (x * q.z),
              (w * q.z) + (z * q.w) + (x * q.y) - (y * q.x),
              (w * q.w) - (x * q.x) - (y * q.y) - (z * q.z)
              };
    };

    inline sQuaternion4 sQuaternion4::multiply(const float num) const {
      return sQuaternion4{w * num, x * num, y * num, z * num};
    }

    inline sQuaternion4 sQuaternion4::sum(const sQuaternion4 &quat) const {
      return sQuaternion4{w + quat.w, x + quat.x, y + quat.y, z + quat.z};
    }

// MATRIX44 ==========================
//
    inline void
    sMat44::set_identity() {
        sx1 = 1.0f; sy1 = 0.0f; sz1 = 0.0f; tmp1 = 0.0f;
        sx2 = 0.0f; sy2 = 1.0f; sz2 = 0.0f; tmp2 = 0.0f;
        sx3 = 0.0f; sy3 = 0.0f; sz3 = 1.0f; tmp3 = 0.0f;
        px = 0.0f; py = 0.0f; pz = 0.0f; tmp4 = 1.0f;
    }

    ///
    /// SIMPLE OPERATIONS
    ///

    inline void sMat44::set_position(const sVector3 &vec) {
        px = vec.x;
        py = vec.y;
        pz = vec.z;
    }

    inline void sMat44::add_position(const sVector3 &vec) {
        px += vec.x;
        py += vec.y;
        pz += vec.z;
    }

    inline void sMat44::set_scale(const sVector3 vec) {
        sx1 = vec.x;
        sy2 = vec.y;
        sz3 = vec.z;
    }


    ///
    /// COMPLEX OPERATIONS
    ///

    // TODO: Wrap loops
    // TODO: Optimize via SIMD
    inline void
    sMat44::multiply(const sMat44   *B) {
        sMat44 result;
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 4; y++) {
                float tmp = 0.0f;
                for (int i = 0; i < 4; i++) {
                    tmp += mat_values[x][i] * B->mat_values[i][y];
                }

                result.mat_values[x][y] = tmp;
            }
        }

        memcpy(mat_values, result.mat_values, sizeof(float) * 16);
    }

    inline void
    sMat44::rotate(const sQuaternion4 *quat) {
        sMat44 tmp_mat, tmp_mat2;
        convert_quaternion_to_matrix(quat, &tmp_mat);
        tmp_mat.invert(&tmp_mat2);
        multiply(&tmp_mat2);
    }

    inline void
    sMat44::scale(const sVector3 vect) {
        sMat44 tmp_mat, tmp_mat2;
        tmp_mat.set_scale(vect);
        tmp_mat.invert(&tmp_mat2);
        multiply(&tmp_mat2);
    }

    inline sVector4 sMat44::multiply(const sVector4   vect) const {
        sVector4 result {};
        for (int i = 0; i < 4; i++) {
            result.raw_values[i] = (vect.raw_values[0] * mat_values[i][0]) +
                                   (vect.raw_values[1] * mat_values[i][1]) +
                                   (vect.raw_values[2] * mat_values[i][2]) +
                                   (vect.raw_values[3] * mat_values[i][3]);
        }

        return result;
    }

    inline sVector3 sMat44::multiply(const sVector3   vect) const{
        float x = sx1 * vect.x + (sx2 * vect.y + (sx3 * vect.z + px));
        float y = sy1 * vect.x + (sy2 * vect.y + (sy3 * vect.z + py));
        float z = sz1 * vect.x + (sz2 * vect.y + (sz3 * vect.z + pz));
        return sVector3{x, y, z};
    }

  inline sQuaternion4 sMat44::multiply(const sQuaternion4 &quat) const {
        sQuaternion4 result {};
        for (int i = 0; i < 4; i++) {
            result.raw_values[i] = (quat.raw_values[0] * mat_values[i][0]) +
                                   (quat.raw_values[1] * mat_values[i][1]) +
                                   (quat.raw_values[2] * mat_values[i][2]) +
                                   (quat.raw_values[3] * mat_values[i][3]);
        }

        return result;

  }

    inline void sMat44::transpose_to(sMat44* result) const {
      result->mat_values[0][0] = mat_values[0][0];
      result->mat_values[0][1] = mat_values[1][0];
      result->mat_values[0][2] = mat_values[2][0];
      result->mat_values[0][3] = mat_values[3][0];
      result->mat_values[1][0] = mat_values[0][1];
      result->mat_values[1][1] = mat_values[1][1];
      result->mat_values[1][2] = mat_values[2][1];
      result->mat_values[1][3] = mat_values[3][1];
      result->mat_values[2][0] = mat_values[0][2];
      result->mat_values[2][1] = mat_values[1][2];
      result->mat_values[2][2] = mat_values[2][2];
      result->mat_values[2][3] = mat_values[3][2];
      result->mat_values[3][0] = mat_values[0][3];
      result->mat_values[3][1] = mat_values[1][3];
      result->mat_values[3][2] = mat_values[2][3];
      result->mat_values[3][3] = mat_values[3][3];
    }

  inline void sMat44::transpose() {
      sMat44 tmp;
      tmp.mat_values[0][0] = mat_values[0][0];
      tmp.mat_values[0][1] = mat_values[1][0];
      tmp.mat_values[0][2] = mat_values[2][0];
      tmp.mat_values[0][3] = mat_values[3][0];
      tmp.mat_values[1][0] = mat_values[0][1];
      tmp.mat_values[1][1] = mat_values[1][1];
      tmp.mat_values[1][2] = mat_values[2][1];
      tmp.mat_values[1][3] = mat_values[3][1];
      tmp.mat_values[2][0] = mat_values[0][2];
      tmp.mat_values[2][1] = mat_values[1][2];
      tmp.mat_values[2][2] = mat_values[2][2];
      tmp.mat_values[2][3] = mat_values[3][2];
      tmp.mat_values[3][0] = mat_values[0][3];
      tmp.mat_values[3][1] = mat_values[1][3];
      tmp.mat_values[3][2] = mat_values[2][3];
      tmp.mat_values[3][3] = mat_values[3][3];
      memcpy(&mat_values, &tmp, sizeof(mat_values));
    }

    // Yoinked from a stackoverlof that yoinked from the MESA implmentation
    // of GLU
    // https://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
    // It uses Sarrus' rule
    // TODO: SIMD...?
    inline void sMat44::invert(sMat44 *result) const {
        float inv[16], det;
        int i;

        inv[0] = raw_values[5]  * raw_values[10] * raw_values[15] -
                 raw_values[5]  * raw_values[11] * raw_values[14] -
                 raw_values[9]  * raw_values[6]  * raw_values[15] +
                 raw_values[9]  * raw_values[7]  * raw_values[14] +
                 raw_values[13] * raw_values[6]  * raw_values[11] -
                 raw_values[13] * raw_values[7]  * raw_values[10];

        inv[4] = -raw_values[4]  * raw_values[10] * raw_values[15] +
                 raw_values[4]  * raw_values[11] * raw_values[14] +
                 raw_values[8]  * raw_values[6]  * raw_values[15] -
                 raw_values[8]  * raw_values[7]  * raw_values[14] -
                 raw_values[12] * raw_values[6]  * raw_values[11] +
                 raw_values[12] * raw_values[7]  * raw_values[10];

        inv[8] = raw_values[4]  * raw_values[9] * raw_values[15] -
                 raw_values[4]  * raw_values[11] * raw_values[13] -
                 raw_values[8]  * raw_values[5] * raw_values[15] +
                 raw_values[8]  * raw_values[7] * raw_values[13] +
                 raw_values[12] * raw_values[5] * raw_values[11] -
                 raw_values[12] * raw_values[7] * raw_values[9];

        inv[12] = -raw_values[4]  * raw_values[9] * raw_values[14] +
                  raw_values[4]  * raw_values[10] * raw_values[13] +
                  raw_values[8]  * raw_values[5] * raw_values[14] -
                  raw_values[8]  * raw_values[6] * raw_values[13] -
                  raw_values[12] * raw_values[5] * raw_values[10] +
                  raw_values[12] * raw_values[6] * raw_values[9];

        inv[1] = -raw_values[1]  * raw_values[10] * raw_values[15] +
                 raw_values[1]  * raw_values[11] * raw_values[14] +
                 raw_values[9]  * raw_values[2] * raw_values[15] -
                 raw_values[9]  * raw_values[3] * raw_values[14] -
                 raw_values[13] * raw_values[2] * raw_values[11] +
                 raw_values[13] * raw_values[3] * raw_values[10];

        inv[5] = raw_values[0]  * raw_values[10] * raw_values[15] -
                 raw_values[0]  * raw_values[11] * raw_values[14] -
                 raw_values[8]  * raw_values[2] * raw_values[15] +
                 raw_values[8]  * raw_values[3] * raw_values[14] +
                 raw_values[12] * raw_values[2] * raw_values[11] -
                 raw_values[12] * raw_values[3] * raw_values[10];

        inv[9] = -raw_values[0]  * raw_values[9] * raw_values[15] +
                 raw_values[0]  * raw_values[11] * raw_values[13] +
                 raw_values[8]  * raw_values[1] * raw_values[15] -
                 raw_values[8]  * raw_values[3] * raw_values[13] -
                 raw_values[12] * raw_values[1] * raw_values[11] +
                 raw_values[12] * raw_values[3] * raw_values[9];

        inv[13] = raw_values[0]  * raw_values[9] * raw_values[14] -
                  raw_values[0]  * raw_values[10] * raw_values[13] -
                  raw_values[8]  * raw_values[1] * raw_values[14] +
                  raw_values[8]  * raw_values[2] * raw_values[13] +
                  raw_values[12] * raw_values[1] * raw_values[10] -
                  raw_values[12] * raw_values[2] * raw_values[9];

        inv[2] = raw_values[1]  * raw_values[6] * raw_values[15] -
                 raw_values[1]  * raw_values[7] * raw_values[14] -
                 raw_values[5]  * raw_values[2] * raw_values[15] +
                 raw_values[5]  * raw_values[3] * raw_values[14] +
                 raw_values[13] * raw_values[2] * raw_values[7] -
                 raw_values[13] * raw_values[3] * raw_values[6];

        inv[6] = -raw_values[0]  * raw_values[6] * raw_values[15] +
                 raw_values[0]  * raw_values[7] * raw_values[14] +
                 raw_values[4]  * raw_values[2] * raw_values[15] -
                 raw_values[4]  * raw_values[3] * raw_values[14] -
                 raw_values[12] * raw_values[2] * raw_values[7] +
                 raw_values[12] * raw_values[3] * raw_values[6];

        inv[10] = raw_values[0]  * raw_values[5] * raw_values[15] -
                  raw_values[0]  * raw_values[7] * raw_values[13] -
                  raw_values[4]  * raw_values[1] * raw_values[15] +
                  raw_values[4]  * raw_values[3] * raw_values[13] +
                  raw_values[12] * raw_values[1] * raw_values[7] -
                  raw_values[12] * raw_values[3] * raw_values[5];

        inv[14] = -raw_values[0]  * raw_values[5] * raw_values[14] +
                  raw_values[0]  * raw_values[6] * raw_values[13] +
                  raw_values[4]  * raw_values[1] * raw_values[14] -
                  raw_values[4]  * raw_values[2] * raw_values[13] -
                  raw_values[12] * raw_values[1] * raw_values[6] +
                  raw_values[12] * raw_values[2] * raw_values[5];

        inv[3] = -raw_values[1] * raw_values[6] * raw_values[11] +
                 raw_values[1] * raw_values[7] * raw_values[10] +
                 raw_values[5] * raw_values[2] * raw_values[11] -
                 raw_values[5] * raw_values[3] * raw_values[10] -
                 raw_values[9] * raw_values[2] * raw_values[7] +
                 raw_values[9] * raw_values[3] * raw_values[6];

        inv[7] = raw_values[0] * raw_values[6] * raw_values[11] -
                 raw_values[0] * raw_values[7] * raw_values[10] -
                 raw_values[4] * raw_values[2] * raw_values[11] +
                 raw_values[4] * raw_values[3] * raw_values[10] +
                 raw_values[8] * raw_values[2] * raw_values[7] -
                 raw_values[8] * raw_values[3] * raw_values[6];

        inv[11] = -raw_values[0] * raw_values[5] * raw_values[11] +
                  raw_values[0] * raw_values[7] * raw_values[9] +
                  raw_values[4] * raw_values[1] * raw_values[11] -
                  raw_values[4] * raw_values[3] * raw_values[9] -
                  raw_values[8] * raw_values[1] * raw_values[7] +
                  raw_values[8] * raw_values[3] * raw_values[5];

        inv[15] = raw_values[0] * raw_values[5] * raw_values[10] -
                  raw_values[0] * raw_values[6] * raw_values[9] -
                  raw_values[4] * raw_values[1] * raw_values[10] +
                  raw_values[4] * raw_values[2] * raw_values[9] +
                  raw_values[8] * raw_values[1] * raw_values[6] -
                  raw_values[8] * raw_values[2] * raw_values[5];

        det = raw_values[0] * inv[0] + raw_values[1] * inv[4] + raw_values[2] * inv[8] + raw_values[3] * inv[12];

        assert(det != 0.0f && "Cannot inverse a matrix with 0 determinant");

        det = 1.0f / det;

        for (i = 0; i < 16; i++) {
            result->raw_values[i] = inv[i] * det;
        }

    }

inline void sMat44::print() const {
  std::cout << "  ======== " << std::endl;
  for(int i = 0; i < 4; i++) {
    for(int j = 0; j < 4; j++) {
      std::cout << mat_values[i][j] << " ";
    }
    std::cout << std::endl;
  }
  std::cout << "  ======== " << std::endl;
}
