#include "wpcore/exitCondition.hpp"
#include "pros/rtos.hpp"
#include <cmath>

namespace wpcore {

ExitCondition::ExitCondition(double range, int timeoutMs) : range(range), timeoutMs(timeoutMs) {}

bool ExitCondition::getExit(double error) {
    if (range <= 0 || timeoutMs <= 0) return false; // a 0 range/timeout means this tier is turned off

    if (std::fabs(error) > range) {
        windowStart = -1; // stepped back outside the range -- the clock starts over next time we enter it
        return false;
    }

    if (windowStart == -1) windowStart = static_cast<long>(pros::millis());
    return static_cast<long>(pros::millis()) - windowStart >= timeoutMs;
}

void ExitCondition::reset() { windowStart = -1; }

} // namespace wpcore
