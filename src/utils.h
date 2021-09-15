#ifndef UTILS_H_
#define UTILS_H_

#include <cstdint>

// TODO: BETTER NAME FOR GODS SAKE
union uUIntTuple {
    uint16_t array[2] = { 0, 0 };
    struct {
        uint16_t e1;
        uint16_t e2;
    };

    inline bool equal_to(const uUIntTuple &t1) const {
        return e1 == t1.e1 && e2 == t1.e2;
    }
};

#endif // UTILS_H_
