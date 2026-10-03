#pragma once
#include <cstdint>
namespace pros {
struct Rotation {
    std::int32_t centidegrees = 0;
    std::int32_t get_position() const { return centidegrees; }
    std::int32_t reset_position() { centidegrees = 0; return 1; }
};
}
