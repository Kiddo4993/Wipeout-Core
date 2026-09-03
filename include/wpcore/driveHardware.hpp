#pragma once
#include "pros/motor_group.hpp" // pros::MotorGroup lives here, not in motors.hpp
#include "pros/imu.hpp"
#include "wpcore/trackingWheel.hpp"

namespace wpcore {

/// The measured diameters (inches) of VEX's omni wheel lineup, just here so
/// a DriveConfig reads as `Omniwheel::NEW_275` instead of a bare `2.75`
/// nobody remembers the meaning of six months later. If yours measures
/// differently, use the real number instead -- these are a convenience, not
/// a rule.
namespace Omniwheel {
    constexpr double OLD_275 = 2.75;
    constexpr double OLD_325 = 3.25;
    constexpr double OLD_4 = 4.0;
    constexpr double NEW_275 = 2.75;
    constexpr double NEW_325 = 3.25;
    constexpr double NEW_4 = 4.0;
} // namespace Omniwheel

/// Everything physical about the drivetrain: which motors spin it, and the
/// measurements needed to turn "the motors did X" into "the robot moved Y".
/// This is just hardware description -- the actual driving happens through
/// a `wpcore::Robot`, which you'll usually name `drivetrain`.
struct DriveConfig {
    pros::MotorGroup* leftMotors;
    pros::MotorGroup* rightMotors;
    double trackWidth;      // inches between the left and right wheels
    double wheelDiameter;   // inches, drive wheel diameter
    double rpm;             // drive wheel rpm at full motor speed
    double horizontalDrift; // wheel scrub fudge factor, kept around for
                             // parity with LemLib's constructor -- nothing
                             // in this library's odometry or PID math
                             // actually reads it yet
};

/// The sensors odometry gets its numbers from. Whatever you don't have,
/// leave as nullptr -- every place this struct gets read checks for null
/// first, so a one-tracking-wheel-plus-IMU setup (like ours) works exactly
/// as well as a full four-wheel-plus-IMU setup.
struct OdomConfig {
    TrackingWheel* vertical1 = nullptr;
    TrackingWheel* vertical2 = nullptr;
    TrackingWheel* horizontal1 = nullptr;
    TrackingWheel* horizontal2 = nullptr;
    pros::Imu* imu = nullptr;
};

/// The tuning knobs for one PID axis (lateral or angular), plus how
/// "close enough" gets defined for that axis. See README.md's tuning guide
/// for the order to actually turn these knobs in.
struct PIDGains {
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double windupRange = 0;    // integral anti-windup range (0 = unlimited)
    double smallError = 0;     // settle range, tight tier -- same units as this axis's error
    int smallErrorTimeout = 0; // ms spent inside smallError before we call it settled
    double largeError = 0;     // settle range, loose tier -- a fallback so "close enough" still ends the motion
    int largeErrorTimeout = 0; // ms spent inside largeError before we call it settled
    double slew = 0;           // max |output| change allowed per 10ms tick, 0 = no limit
};

} // namespace wpcore
