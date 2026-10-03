#pragma once
#include <cstdint>
namespace pros {
// Like the real sensor: get_rotation() is clockwise-positive and unbounded.
struct Imu {
    double rotation = 0;
    double get_rotation() const { return rotation; }
    std::int32_t reset(bool = false) { return 1; }
};
}
