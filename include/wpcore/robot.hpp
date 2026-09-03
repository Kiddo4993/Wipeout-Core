#pragma once
#include "wpcore/pose.hpp"
#include "wpcore/pid.hpp"
#include "wpcore/odometry.hpp"
#include "wpcore/driveHardware.hpp"
#include "wpcore/logger.hpp"
#include "pros/rtos.hpp"

namespace wpcore {

/// Options for Robot::moveToPoint(). Declared at namespace scope (not nested
/// in Robot) so its default member initializers can be used as the default
/// argument on moveToPoint() itself -- a nested class's defaults aren't
/// usable there until the enclosing class is complete.
struct MoveToPointParams {
    bool forwards = true;      // drive to the point front-first (false = reverse into it)
    double maxSpeed = 127;     // motor voltage cap, [0, 127]
    double minSpeed = 0;       // motor voltage floor while still moving (fights static friction)
    double earlyExitRange = 0; // if > 0, end the motion as soon as within
                                // this many inches, without waiting on
                                // the settle timers below

    // --- tuning aids, both off by default so a normal autonomous run stays quiet ---
    const char* logFile = nullptr; // e.g. "/usd/lateral_tune.csv" -- logs
                                    // time/error/output for this motion so
                                    // you can graph it afterward instead of
                                    // just watching the robot and guessing
    bool telemetry = false;        // print live distance/heading error and
                                    // motor output to the brain screen while
                                    // this motion runs, so you can watch it
                                    // settle (or not) in real time
};

/// Options for Robot::turnToHeading(). See MoveToPointParams for why this
/// lives at namespace scope instead of nested in Robot.
struct TurnToHeadingParams {
    double maxSpeed = 127;
    double minSpeed = 0;
    const char* logFile = nullptr; // see MoveToPointParams::logFile
    bool telemetry = false;        // see MoveToPointParams::telemetry
};

/// This is the whole robot, glued together: the motors and dimensions in
/// DriveConfig, the sensors in OdomConfig, and the tuning numbers in
/// PIDGains all get handed to one Robot, and everything else (driving
/// somewhere, turning to face something, knowing where you are) happens
/// through it. It's the direct equivalent of LemLib's `lemlib::Chassis
/// chassis` -- just renamed, on purpose, so the object sitting in your code
/// is called `drivetrain` instead of `chassis`.
class Robot {
public:
    Robot(DriveConfig drive, PIDGains lateralGains, PIDGains angularGains, OdomConfig odomCfg);

    /// Resets every sensor to zero and kicks off the background task that
    /// keeps the pose updated. Call this once, from initialize() -- do it
    /// too early (before the IMU's done calibrating) and your heading will
    /// be garbage for the first second or two.
    void calibrate();

    Pose getPose() const;
    void setPose(double x, double y, double theta);
    void setPose(const Pose& pose);

    /// Drives to (x, y). Under the hood it's two PID loops running at once:
    /// distance-to-target feeds the lateral one, angle-to-target feeds the
    /// angular one, and their outputs get blended into left/right motor
    /// voltages every loop. Full derivation is in robot.cpp right above
    /// moveToPointTask -- worth reading once, it's the heart of this whole
    /// library.
    void moveToPoint(double x, double y, int timeout, MoveToPointParams params = {}, bool async = true);

    /// Spins in place to face `theta` degrees (compass heading -- same
    /// convention as the IMU), driven by one PID loop on the shortest-path
    /// angle error.
    void turnToHeading(double theta, int timeout, TurnToHeadingParams params = {}, bool async = true);

    /// Blocks until whatever async motion is running finishes -- either it
    /// settled, or it ran out of time and gave up.
    void waitUntilDone();

    /// Blocks until the robot is `distance` into the current async motion
    /// (inches for moveToPoint, degrees for turnToHeading). Useful for
    /// starting a mechanism partway through a drive instead of waiting for
    /// the whole thing to finish first.
    void waitUntil(double distance);

    /// Straight-up tank drive, no PID involved -- plug your joystick values
    /// in directly. This is what opcontrol should be calling. Range [-127, 127].
    void tank(int left, int right);
    /// Arcade drive: one stick for `throttle` (forward/back), one for `turn`
    /// (left/right), both [-127, 127].
    void arcade(int throttle, int turn);

private:
    DriveConfig drive;
    PIDGains lateralGains;
    PIDGains angularGains;
    Odometry odom;

    PID lateralPID;
    PID angularPID;

    pros::Task* driveTask = nullptr;
    pros::Task* motionTask = nullptr;

    volatile bool motionRunning = false;
    volatile bool stopRequested = false;
    double distanceTravelled = 0;
    mutable pros::Mutex motionMutex;

    void odomLoop(); // background: odom.update() every 10ms

    // Cooperatively stops whatever motion task is currently running and
    // waits (briefly) for it to actually exit before returning, so two
    // motions never fight over the same motors.
    void cancelCurrentMotion();

    void moveToPointTask(double x, double y, int timeout, MoveToPointParams params);
    void turnToHeadingTask(double theta, int timeout, TurnToHeadingParams params);

    static double slewLimit(double target, double previous, double maxDelta);
};

} // namespace wpcore
