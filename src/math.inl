
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

inline float dot_prod(const sVector3 v1, const sVector3 v2) {
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline sVector3 cross_prod(const sVector3 v1, const sVector3 v2) {
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

inline void
convert_quaternion_to_matrix(const sQuaternion4 *quat, sMat44 *mat) {
    mat->mat_values[0][0] = 2.0f * (quat->q0 * quat->q0 + quat->q1 * quat->q1) - 1;
    mat->mat_values[1][0] = 2.0f * (quat->q1 * quat->q1 + quat->q0 * quat->q3);
    mat->mat_values[2][0] = 2.0f * (quat->q1 * quat->q3 - quat->q0 * quat->q2);

    mat->mat_values[0][1] = 2.0f * (quat->q1 * quat->q2 - quat->q0 * quat->q3);
    mat->mat_values[1][1] = 2.0f * (quat->q0 * quat->q0 + quat->q2 * quat->q2) - 1;
    mat->mat_values[2][1] = 2.0f * (quat->q2 * quat->q3 + quat->q0 * quat->q1);

    mat->mat_values[0][2] = 2.0f * (quat->q1 * quat->q3 + quat->q0 * quat->q2);
    mat->mat_values[1][2] = 2.0f * (quat->q2 * quat->q3 - quat->q0 * quat->q1);
    mat->mat_values[2][2] = 2.0f * (quat->q0 * quat->q0 + quat->q3 * quat->q3) - 1;
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

    inline sQuaternion4 sQuaternion4::multiply(const sQuaternion4 &quat) const {
      sQuaternion4 result;
      result.w = w * quat.w - dot_prod(vect, quat.vect);
      result.vect = cross_prod(vect, quat.vect).sum(vect.mult(quat.w)).sum(quat.vect.mult(w));

      return result;
    };

    inline sQuaternion4 sQuaternion4::multiply(const float num) const {
      return sQuaternion4{w * num, x * num, y * num, z * num};
    }

    inline sQuaternion4 sQuaternion4::sum(const sQuaternion4 &quat) const {
      return sQuaternion4{w + quat.w, x + quat.x, y + quat.y, z + quat.z};
    }

// MATRIX44 ==========================
//
    void
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

    // Yoinked from a stackoverlof that yoinked from the MESA implmentation
    // of GLU
    // https://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
    // It uses Sarrus' rule
    // TODO: SIMD...?
    void sMat44::invert(sMat44 *result) const {
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
