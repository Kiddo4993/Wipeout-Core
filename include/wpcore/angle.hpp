#pragma once

namespace wpcore {

// Defined ourselves instead of trusting the M_PI macro -- it's not part of
// the C++ standard, and not every toolchain bothers to define it, so better
// to just not depend on it.
constexpr double PI = 3.14159265358979323846;

constexpr double degToRad(double deg) { return deg * PI / 180.0; }
constexpr double radToDeg(double rad) { return rad * 180.0 / PI; }

/// Squashes any angle in degrees down into [0, 360) -- useful any time you
/// want a heading to read the way a compass would (no negative numbers, no
/// "410 degrees").
double wrapDeg360(double deg);

/// Squashes any angle in degrees down into (-180, 180]. This is the one you
/// want when the actual question is "which way do I turn" -- a wrapped
/// result close to 0 means barely any turn, close to +-180 means almost a
/// full about-face, and the sign tells you the direction.
double wrapDeg180(double deg);

/// The question every turn boils down to: standing at `current`, how many
/// degrees do I need to turn to be facing `target`? Answered as the
/// shortest path, in (-180, 180]. Positive means turn clockwise/right,
/// negative means counterclockwise/left -- matching the same
/// clockwise-is-positive convention the IMU uses, so this always agrees
/// with what the heading sensor is telling you.
double angleError(double target, double current);

} // namespace wpcore
