#include "wpcore/angle.hpp"
#include <cmath>

namespace wpcore {

double wrapDeg360(double deg) {
    double wrapped = std::fmod(deg, 360.0);
    if (wrapped < 0) wrapped += 360.0; // fmod can hand back a negative result, nudge it into [0, 360)
    return wrapped;
}

double wrapDeg180(double deg) {
    // Shift by 180 first so the same "mod into [0, 360), then re-center"
    // trick as wrapDeg360 lands us in [-180, 180) instead.
    double wrapped = std::fmod(deg + 180.0, 360.0);
    if (wrapped < 0) wrapped += 360.0;
    return wrapped - 180.0;
}

double angleError(double target, double current) {
    return wrapDeg180(target - current);
}

} // namespace wpcore
