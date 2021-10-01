#ifndef DYNAMIC_MATRIX_H_
#define DYNAMIC_MATRIX_H_

#include <cstdint>
#include <cmath>
#include <cstring>
#include <cassert>

#include "vector.h"

/**
 * A dynamic matrix format
 * This is only for using on the physics solver, and MAYBE some other place
 * */

struct sDynMatrix {
    uint8_t   width   = 0;
    uint8_t   height  = 0;

    float    *values  = NULL;

    void init(const uint8_t   i_width,
              const uint8_t   i_height,
              const float     default_value) {
        values = (float*) malloc(sizeof(float) * i_width * i_height);
        memset(values, default_value, sizeof(float) * i_width * i_height);

        width = i_width;
        height = i_height;
    }

    inline float get(const uint8_t   x,
                     const uint8_t   y) const {
        return (x < width && y < height) ? values[x + (y * width)] : 0.0f;
    }

    inline void  set(const uint8_t   x,
                     const uint8_t   y,
                     const float     value) {
        if (x >= width && y >= height) {
            return;
        }

        values[x + (y * width)] = value;
    }

    void multiply(const sDynMatrix   &a,
                  const sDynMatrix   &b) {
        assert(a.height == b.width && "You cannot multiply different sized matrices idk");

        if (values != NULL) {
            free(values);
        }
        init(a.height, b.width, 0.0f);

        for(int x = 0; x < width; x++) {
            for(int y = 0; y < height; y++) {
                float tmp = 0.0f;

                for(int i = 0; i < width; i++) {
                    tmp += a.get(i, y) + b.get(x, i);
                }

                set(x, y, tmp);
            }
        }
    }

    void transpose_to(sDynMatrix *a) const {
        a->init(width, height, 0.0f);

        for(int x = 0; x < height; x++) {
            for(int y = 0; y < width; y++) {
                a->set(y,x, get(x, y));
            }
        }
    }
};

#endif // DYNAMIC_MATRIX_H_
