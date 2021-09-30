//
// Created by jsmar on 20/05/2021.
//
#ifndef __MATH_H_
#define __MATH_H_

#include <string.h>
#include <assert.h>
#include <cmath>

/**
 * Single header Math library with usefull utilities and structures
 * by Juan S. Marquerie
 * */

// TODO: cleanup a bit, its getting a bit messy bro

//// TYPE DEFINTIONS

union sQuaternion4;
union sVector2;
union sVector4;
union sMat44;
void convert_quaternion_to_matrix(const sQuaternion4 *quat, sMat44 *mat);

union sVector2 {
    struct { float x = 0.0f; float y = 0.0f;};
    float raw_values[2];
};

union sVector3 {
    struct { float x = 0.0f; float y = 0.0f; float z = 0.0f; };
    float raw_values[3];

    inline void multiply(const sVector3 &vect) {
      x *= vect.x;
      y *= vect.y;
      z *= vect.z;
    }

    inline sVector3 sum(const sVector3 &vect) const {
      return {x + vect.x, y + vect.y, z + vect.z};
    }

    inline sVector3 subs(const sVector3 &vect) const {
      return {x - vect.x, y - vect.y, z - vect.z};
    }

    inline sVector3 mult(const float num) const {
      return {x * num, y * num, z * num};
    }

    inline sVector3 mult(const sVector3 &vect) const {
        return {x * vect.x, y * vect.y, z * vect.z};
    }

    inline sVector3 invert() const {
      return sVector3{-x, -y, -z};
    };

    inline float dot(const sVector3 vect) const {
        return vect.x * x + vect.y * y + vect.z * z;
    }

    inline sVector3 normalize() const {
      float magnitude = sqrt((x*x)+ (y*y) + (z*z));
      return sVector3{x / magnitude, y / magnitude, z / magnitude};
    }

    inline float magnitude() const {
      return sqrt( (x*x) + (y*y) + (z*z) );
    }

    inline bool is_equal(const sVector3 v) const {
      return v.x == x && v.y == y && v.z == z;
    }

    inline sQuaternion4 get_pure_quaternion() const;
};

union sVector4 {
    struct { float x = 0.0f; float y = 0.0f; float z = 0.0f; float w = 0.0f; };
    float raw_values[4];
};


union sQuaternion4 {
    float raw_values[4];
    struct {
        float q0 = 1.0f;
        float q1 = 0.0f;
        float q2 = 0.0f;
        float q3 = 0.0f;
    };
    struct {
        float w;
        float x;
        float y;
        float z;
    };

    sQuaternion4 inverse() const;

    sQuaternion4 normalize() const;

    inline sQuaternion4 conjugate() const;

    sQuaternion4 multiply(const sQuaternion4 &quat) const;

    sQuaternion4 multiply(const float num) const; 

    sQuaternion4 sum(const sQuaternion4 &quat) const;

    sVector3 get_vector() const {
        return sVector3{x, y, z};
    }
};

union sMat33 {
    float raw_values[9];
    float mat_values[3][3];
    struct {
        float sx1 = 1.0f; float sy1 = 0.0f; float tmp1 = 0.0f;
        float sx2 = 0.0f; float sy2 = 1.0f; float tmp2 = 0.0f;
        float px = 0.0f; float py = 0.0f; float tmp3 = 1.0;
    };

    void set_identity() {
        sx1 = 1.0f;  sy1 = 0.0f;  px = 0.0f;
        sx2 = 0.0f;  sy2 = 1.0f;  py = 0.0f;
        tmp1 = 0.0f; tmp2 = 0.0f; tmp3 = 1.0;
    }

    inline sVector2 multiply(const sVector2   *vect) {
        return sVector2{ (vect->x * sx1) + (vect->y * sy1) + px, (vect->x * sx2) + (vect->y * sy2) + py };
    }

    inline sVector3 multiply(const sVector3   &vect) const {
        sVector3 result {};
        for (int i = 0; i < 3; i++) {
            result.raw_values[i] = (vect.raw_values[0] * mat_values[i][0]) +
                                   (vect.raw_values[1] * mat_values[i][1]) +
                                   (vect.raw_values[2] * mat_values[i][2]);
        }

        return result;
    }


    inline void
    multiply(const sMat33   *B) {
        sMat33 result = {};
        for (int x = 0; x < 3; x++) {
            for (int y = 0; y < 3; y++) {
                float tmp = 0.0f;
                for (int i = 0; i < 3; i++) {
                    tmp += mat_values[x][i] * B->mat_values[i][y];
                }

                result.mat_values[x][y] = tmp;
            }
        }

        memcpy(mat_values, result.mat_values, sizeof(float) * 9);
    }

    inline void
    multiply_to(const sMat33   *B,
                      sMat33   *result) {
        for (int x = 0; x < 3; x++) {
            for (int y = 0; y < 3; y++) {
                float tmp = 0.0f;
                for (int i = 0; i < 3; i++) {
                    tmp += mat_values[x][i] * B->mat_values[i][y];
                }

                result->mat_values[x][y] = tmp;
            }
        }
    }

    /*inline sVector3 multiply(const sVector3   vect) const{
        float x = sx1 * vect.x + (sx2 * vect.y + (sx3 * vect.z + px));
        float y = sy1 * vect.x + (sy2 * vect.y + (sy3 * vect.z + py));
        float z = tmp1 * vect.x + (tmp2 * vect.y + (tmp3 * vect.z + pz));
        return sVector3{x, y, z};
    }*/


    inline void set_position(const sVector2    vec) {
        px = vec.x;
        py = vec.y;
    }

    inline void set_scale(const sVector2   vec) {
        sx1 = vec.x;
        sy2 = vec.y;
    }

    inline void invert(sMat33 *inverse) const {
        // Using sarrus rule
        float determinant = (mat_values[0][0] * (mat_values[1][1] * mat_values[2][2] - mat_values[2][1] * mat_values[1][2])
                            - mat_values[1][0] * (mat_values[0][1] * mat_values[2][2] - mat_values[2][1] * mat_values[0][2])
                            + mat_values[2][0] * (mat_values[0][1] * mat_values[1][2] - mat_values[1][1] * mat_values[0][2]));
        assert(determinant != 0 && "Determinant cannto be 0 on inv_matrix 3");

        determinant = 1.0f / determinant;

        inverse->mat_values[0][0] = + (mat_values[1][1] * mat_values[2][2] - mat_values[2][1] * mat_values[1][2]) * determinant;
        inverse->mat_values[1][0] = - (mat_values[1][0] * mat_values[2][2] - mat_values[2][0] * mat_values[1][2]) * determinant;
        inverse->mat_values[2][0] = + (mat_values[1][0] * mat_values[2][1] - mat_values[2][0] * mat_values[1][1]) * determinant;
        inverse->mat_values[0][1] = - (mat_values[0][1] * mat_values[2][2] - mat_values[2][1] * mat_values[0][2]) * determinant;
        inverse->mat_values[1][1] = + (mat_values[0][0] * mat_values[2][2] - mat_values[2][0] * mat_values[0][2]) * determinant;
        inverse->mat_values[2][1] = - (mat_values[0][0] * mat_values[2][1] - mat_values[2][0] * mat_values[0][1]) * determinant;
        inverse->mat_values[0][2] = + (mat_values[0][1] * mat_values[1][2] - mat_values[1][1] * mat_values[0][2]) * determinant;
        inverse->mat_values[1][2] = - (mat_values[0][0] * mat_values[1][2] - mat_values[1][0] * mat_values[0][2]) * determinant;
        inverse->mat_values[2][2] = + (mat_values[0][0] * mat_values[1][1] - mat_values[1][0] * mat_values[0][1]) * determinant;
      }

};

union sMat44 {
    struct {
        float sx1 = 1.0f; float sy1 = 0.0f; float sz1 = 0.0f; float tmp1 = 0.0f;
        float sx2 = 0.0f; float sy2 = 1.0f; float sz2 = 0.0f; float tmp2 = 0.0f;
        float sx3 = 0.0f; float sy3 = 0.0f; float sz3 = 1.0f; float tmp3 = 0.0f;
        float px = 0.0f; float py = 0.0f; float pz = 0.0f; float tmp4 = 1.0f;
    };
    float raw_values[16];
    float mat_values[4][4];

    void
    set_identity();
    ///
    /// SIMPLE OPERATIONS
    ///

    inline void set_position(const sVector3 &vec);

    inline void add_position(const sVector3 &vec);

    inline void set_scale(const sVector3 vec);


    ///
    /// COMPLEX OPERATIONS
    ///

    // TODO: Wrap loops
    // TODO: Optimize via SIMD
    inline void
    multiply(const sMat44   *B);

    inline void
    rotate(const sQuaternion4 *quat);

    inline void
    scale(const sVector3 vect);

    inline sQuaternion4 multiply(const sQuaternion4 &quat) const;

    inline sVector4 multiply(const sVector4   vect) const;

    inline sVector3 multiply(const sVector3   vect) const;

    inline void transpose_to(sMat44* result) const;

    inline void transpose();

    // Yoinked from a stackoverlof that yoinked from the MESA implmentation
    // of GLU
    // https://stackoverflow.com/questions/1148309/inverting-a-4x4-matrix
    // It uses Sarrus' rule
    // TODO: SIMD...?
    void invert(sMat44 *result) const;

    inline void print() const;
};


#include "math.inl"

#endif // __MATH_H_
