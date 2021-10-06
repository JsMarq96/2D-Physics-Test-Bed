#ifndef GAUSS_SEIDEL_H_
#define GAUSS_SEIDEL_H_

#include "math.h"

inline void gauss_seidel_iteration(const sDynMatrix  &A,
                                   const sDynMatrix  &b,
                                         sDynMatrix  &x,
                                   const float       min_lambda,
                                   const float       max_lambda) {
    float a_top = 0.0f;

    for(int i = 0; i < x.height; i++) {
        float delta = 0.0f;
    }
}

#endif // GAUSS_SEIDEL_H_
