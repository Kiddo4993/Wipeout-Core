#include "wpcore/pose.hpp"
#include "wpcore/angle.hpp"
#include <cmath>

namespace wpcore {

double Pose::distanceTo(const Pose& other) const {
    return std::hypot(other.x - x, other.y - y);
}

double Pose::angleTo(const Pose& other) const {
    // Normally atan2 takes (y, x). Here it's flipped to atan2(x, y) on
    // purpose: our theta = 0 points along +y, not +x, so the axes atan2
    // needs to reason about are swapped from the textbook version.
    return radToDeg(std::atan2(other.x - x, other.y - y));
}

} // namespace wpcore
