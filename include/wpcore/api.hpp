#pragma once

// Wipeout Core -- our own odometry + PID motion library, built from scratch and
// shaped like LemLib (same config structs, same moveToPoint/turnToHeading
// calls) but with our own math and our own names, so every formula in here
// can actually be explained during judging instead of shrugged at. It also
// carries its own PID-tuning tools (Logger + the telemetry flags on the
// motion params) so dialing in gains means looking at real numbers instead
// of vibes. See README.md for the full math writeup, the tuning guide, and
// the LemLib -> Wipeout Core name mapping.
//
// #include this single header to get the whole public API, same as
// #include "lemlib/api.hpp" did.

#include "wpcore/angle.hpp"
#include "wpcore/pose.hpp"
#include "wpcore/pid.hpp"
#include "wpcore/exitCondition.hpp"
#include "wpcore/trackingWheel.hpp"
#include "wpcore/driveHardware.hpp"
#include "wpcore/odometry.hpp"
#include "wpcore/logger.hpp"
#include "wpcore/robot.hpp"
