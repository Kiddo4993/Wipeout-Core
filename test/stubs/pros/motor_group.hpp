#pragma once
#include <cstdint>
#include <vector>
namespace pros {
struct MotorGroup {
    std::vector<std::int32_t> history;
    std::int32_t move(std::int32_t voltage) { history.push_back(voltage); return 1; }
};
}
