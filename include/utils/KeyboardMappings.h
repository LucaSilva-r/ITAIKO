#ifndef UTILS_KEYBOARDMAPPINGS_H_
#define UTILS_KEYBOARDMAPPINGS_H_

#include <cstdint>

namespace Doncon::Utils {

struct DrumKeys {
    uint8_t ka_left;
    uint8_t don_left;
    uint8_t don_right;
    uint8_t ka_right;
};

struct ControllerKeys {
    uint8_t up;
    uint8_t down;
    uint8_t left;
    uint8_t right;

    uint8_t north;
    uint8_t east;
    uint8_t south;
    uint8_t west;

    uint8_t l;
    uint8_t r;

    uint8_t start;
    uint8_t select;
    uint8_t home;
    uint8_t share;

    uint8_t l3;
    uint8_t r3;
};

} // namespace Doncon::Utils

#endif // UTILS_KEYBOARDMAPPINGS_H_