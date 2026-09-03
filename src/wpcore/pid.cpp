#include "wpcore/pid.hpp"
#include <cmath>

namespace wpcore {

PID::PID(double kP, double kI, double kD, double windupRange, bool resetOnSignFlip)
    : kP(kP), kI(kI), kD(kD), windupRange(windupRange), resetOnSignFlip(resetOnSignFlip) {}

double PID::update(double error) {
    // Only let the integral grow once we're within windupRange of the
    // target -- otherwise it just sits there piling up while we're still
    // far away and not doing anything useful yet.
    if (windupRange == 0 || std::fabs(error) < windupRange) {
        integral += error;
    } else {
        integral = 0;
    }

    // Error just flipped sign, meaning we sailed past the target. Whatever
    // integral built up on the way there was measuring the wrong thing now
    // -- wipe it so it can't keep shoving us further past.
    if (resetOnSignFlip && hasPreviousError &&
        ((error < 0 && previousError > 0) || (error > 0 && previousError < 0))) {
        integral = 0;
    }

    double derivative = hasPreviousError ? (error - previousError) : 0;

    previousError = error;
    hasPreviousError = true;

    return kP * error + kI * integral + kD * derivative;
}

void PID::reset() {
    integral = 0;
    previousError = 0;
    hasPreviousError = false;
}

} // namespace wpcore
