#ifndef MATRIX33_H_
#define MATRIX33_H_

#include <string.h>
#include <assert.h>
#include <cmath>

#include "vector.h"

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
        sVector3 result = {};
        for (int i = 0; i < 3; i++) {
            result.raw_values[i] = (vect.raw_values[0] * mat_values[i][0]) +
                                   (vect.raw_values[1] * mat_values[i][1]) +
                                   (vect.raw_values[2] * mat_values[i][2]);
        }

        return result;
    }


    inline void multiply(const sMat33   *B) {
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

    inline void multiply_to(const sMat33   *B,
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

#endif // MATRIX33_H_
