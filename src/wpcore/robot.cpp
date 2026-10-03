#include "wpcore/robot.hpp"
#include "wpcore/angle.hpp"
#include "wpcore/exitCondition.hpp"
#include "pros/llemu.hpp"
#include <cmath>
#include <algorithm>

namespace wpcore {

// Inside this distance (inches) moveToPoint stops steering: the angle to the
// target swings wildly that close, and flips 180 degrees on an overshoot.
// Same value LemLib uses.
constexpr double CLOSE_RANGE = 7.5;

Robot::Robot(DriveConfig drive, PIDGains lateralGains, PIDGains angularGains, OdomConfig odomCfg)
    : drive(drive), lateralGains(lateralGains), angularGains(angularGains), odom(odomCfg),
      lateralPID(lateralGains.kP, lateralGains.kI, lateralGains.kD, lateralGains.windupRange),
      angularPID(angularGains.kP, angularGains.kI, angularGains.kD, angularGains.windupRange) {}

void Robot::calibrate() {
    odom.calibrate();
    if (driveTask == nullptr) driveTask = new pros::Task([this] { odomLoop(); });
}

void Robot::odomLoop() {
    while (true) {
        odom.update();
        pros::delay(10);
    }
}

Pose Robot::getPose() const { return odom.getPose(); }
void Robot::setPose(double x, double y, double theta) { odom.setPose(Pose(x, y, theta)); }
void Robot::setPose(const Pose& pose) { odom.setPose(pose); }

double Robot::slewLimit(double target, double previous, double maxDelta) {
    if (maxDelta <= 0) return target; // slew disabled
    double delta = std::clamp(target - previous, -maxDelta, maxDelta);
    return previous + delta;
}

void Robot::cancelCurrentMotion() {
    if (motionTask == nullptr) return;
    // We ask the loop to stop itself on its next iteration rather than
    // force-killing the task with remove(). Killing a task mid-iteration
    // skips whatever cleanup was left below it -- here that's "set the
    // motors to 0" -- so a force-kill can leave the drive running at
    // whatever voltage it last had. Cooperative shutdown means it always
    // gets to stop itself properly.
    stopRequested = true;
    uint32_t waitStart = pros::millis();
    while (motionRunning && pros::millis() - waitStart < 100) pros::delay(5);
    delete motionTask;
    motionTask = nullptr;
    stopRequested = false;
}

void Robot::moveToPoint(double x, double y, int timeout, MoveToPointParams params, bool async) {
    cancelCurrentMotion();
    // Mark the motion as running *before* its task exists. Otherwise a
    // waitUntilDone()/waitUntil() right after an async start can run before the
    // task does, see motionRunning == false, and return immediately.
    motionMutex.take();
    motionRunning = true;
    distanceTravelled = 0;
    motionMutex.give();
    if (async) {
        motionTask = new pros::Task([this, x, y, timeout, params] { moveToPointTask(x, y, timeout, params); });
        return;
    }
    moveToPointTask(x, y, timeout, params);
}

// This is the loop that actually drives moveToPoint, whether it's running
// directly (sync) or inside the pros::Task spawned above (async). It's two
// PID loops running side by side, sharing the same pair of motors:
//
//   1. distance = straight-line distance from where we are to (x, y).
//      That's the lateral PID's error: "how much farther forward".
//   2. angleToTarget = compass bearing from here to (x, y). headingError is
//      the shortest-path turn from where we're currently facing to that
//      bearing -- that's the angular PID's error: "how much do I need to
//      turn to point at it".
//   3. Each PID spits out a voltage, and they get combined the way every
//      tank drive steers: left = lateral + angular, right = lateral -
//      angular. Add the turn correction to one side and subtract it from
//      the other and the robot curves toward the target while still moving
//      forward, instead of turning then driving in two separate steps.
//   4. One extra trick: lateralOut gets multiplied by cos(headingError).
//      If the robot's currently facing 90 degrees away from the point,
//      "drive forward" is pointing 90 degrees away from useful progress --
//      so the further off-heading we are, the less the lateral term is
//      trusted, and the angular term gets to turn us on-target first before
//      we commit to driving.
void Robot::moveToPointTask(double x, double y, int timeout, MoveToPointParams params) {
    lateralPID.reset();
    angularPID.reset();
    ExitCondition smallExit(lateralGains.smallError, lateralGains.smallErrorTimeout);
    ExitCondition largeExit(lateralGains.largeError, lateralGains.largeErrorTimeout);
    Logger* logger = params.logFile ? new Logger(params.logFile) : nullptr;

    motionMutex.take();
    motionRunning = true;
    distanceTravelled = 0;
    motionMutex.give();

    double prevLateralOut = 0;
    double prevAngularOut = 0;
    Pose prevPose = getPose();
    uint32_t start = pros::millis();

    while (pros::millis() - start < static_cast<uint32_t>(timeout) && !stopRequested) {
        Pose pose = getPose();
        double dx = x - pose.x;
        double dy = y - pose.y;
        double distance = std::hypot(dx, dy);

        double angleToTarget = radToDeg(std::atan2(dx, dy));
        if (!params.forwards) angleToTarget = wrapDeg180(angleToTarget + 180.0);
        double headingError = angleError(angleToTarget, pose.theta);

        if (params.earlyExitRange > 0 && distance < params.earlyExitRange) break;
        if (smallExit.getExit(distance) || largeExit.getExit(distance)) break;

        double signedDistance = params.forwards ? distance : -distance;
        double lateralOut = lateralPID.update(signedDistance);
        double angularOut = angularPID.update(headingError);
        if (distance < CLOSE_RANGE) angularOut = 0;

        lateralOut *= std::cos(degToRad(headingError));

        lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);
        angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);
        lateralOut = slewLimit(lateralOut, prevLateralOut, lateralGains.slew);
        angularOut = slewLimit(angularOut, prevAngularOut, angularGains.slew);
        prevLateralOut = lateralOut;
        prevAngularOut = angularOut;

        // Once we're close enough that smallExit would end the motion
        // anyway, stop forcing a minimum speed -- otherwise the robot is
        // fighting itself and can never actually settle.
        if (params.minSpeed > 0 && distance > lateralGains.smallError && std::fabs(lateralOut) < params.minSpeed) {
            lateralOut = std::copysign(params.minSpeed, lateralOut);
        }

        drive.leftMotors->move(lateralOut + angularOut);
        drive.rightMotors->move(lateralOut - angularOut);

        // Tuning aids -- neither one does anything unless you asked for it
        // in params. See the "Tuning PID" section of README.md for how to
        // actually use these to dial in kP/kI/kD instead of guessing.
        if (logger) logger->log(pros::millis() - start, distance, lateralOut);
        if (params.telemetry) {
            pros::lcd::print(4, "dist:%.2f  hdgErr:%.2f", distance, headingError);
            pros::lcd::print(5, "latOut:%.1f  angOut:%.1f", lateralOut, angularOut);
        }

        motionMutex.take();
        distanceTravelled += pose.distanceTo(prevPose);
        motionMutex.give();
        prevPose = pose;

        pros::delay(10);
    }

    drive.leftMotors->move(0);
    drive.rightMotors->move(0);
    delete logger;

    motionMutex.take();
    motionRunning = false;
    motionMutex.give();
}

void Robot::turnToHeading(double theta, int timeout, TurnToHeadingParams params, bool async) {
    cancelCurrentMotion();
    // Same as moveToPoint: mark the motion as running before its task exists.
    motionMutex.take();
    motionRunning = true;
    distanceTravelled = 0;
    motionMutex.give();
    if (async) {
        motionTask = new pros::Task([this, theta, timeout, params] { turnToHeadingTask(theta, timeout, params); });
        return;
    }
    turnToHeadingTask(theta, timeout, params);
}

// Simpler than moveToPoint -- just one PID loop. Error is the shortest-path
// number of degrees from where we're facing to `theta`, and the output goes
// out as equal-and-opposite voltage on each side, which is what makes a
// tank drive spin in place instead of driving forward.
void Robot::turnToHeadingTask(double theta, int timeout, TurnToHeadingParams params) {
    angularPID.reset();
    ExitCondition smallExit(angularGains.smallError, angularGains.smallErrorTimeout);
    ExitCondition largeExit(angularGains.largeError, angularGains.largeErrorTimeout);
    Logger* logger = params.logFile ? new Logger(params.logFile) : nullptr;

    motionMutex.take();
    motionRunning = true;
    distanceTravelled = 0;
    motionMutex.give();

    double prevOut = 0;
    double prevTheta = getPose().theta;
    uint32_t start = pros::millis();

    while (pros::millis() - start < static_cast<uint32_t>(timeout) && !stopRequested) {
        Pose pose = getPose();
        double error = angleError(theta, pose.theta);

        if (smallExit.getExit(error) || largeExit.getExit(error)) break;

        double out = angularPID.update(error);
        out = std::clamp(out, -params.maxSpeed, params.maxSpeed);
        out = slewLimit(out, prevOut, angularGains.slew);
        prevOut = out;

        if (params.minSpeed > 0 && std::fabs(error) > angularGains.smallError && std::fabs(out) < params.minSpeed) {
            out = std::copysign(params.minSpeed, out);
        }

        drive.leftMotors->move(out);
        drive.rightMotors->move(-out);

        if (logger) logger->log(pros::millis() - start, error, out);
        if (params.telemetry) {
            pros::lcd::print(4, "target:%.1f  err:%.2f", theta, error);
            pros::lcd::print(5, "out:%.1f", out);
        }

        // waitUntil() during a turn measures degrees turned, not inches --
        // there's no "distance" in a spin, so this is the closest analog.
        motionMutex.take();
        distanceTravelled += std::fabs(angleError(pose.theta, prevTheta));
        motionMutex.give();
        prevTheta = pose.theta;

        pros::delay(10);
    }

    drive.leftMotors->move(0);
    drive.rightMotors->move(0);
    delete logger;

    motionMutex.take();
    motionRunning = false;
    motionMutex.give();
}

void Robot::waitUntilDone() {
    while (true) {
        motionMutex.take();
        bool running = motionRunning;
        motionMutex.give();
        if (!running) break;
        pros::delay(10);
    }
}

void Robot::waitUntil(double distance) {
    while (true) {
        motionMutex.take();
        bool running = motionRunning;
        double travelled = distanceTravelled;
        motionMutex.give();
        if (!running || travelled >= distance) break;
        pros::delay(10);
    }
}

void Robot::tank(int left, int right) {
    drive.leftMotors->move(std::clamp(left, -127, 127));
    drive.rightMotors->move(std::clamp(right, -127, 127));
}

void Robot::arcade(int throttle, int turn) {
    drive.leftMotors->move(std::clamp(throttle + turn, -127, 127));
    drive.rightMotors->move(std::clamp(throttle - turn, -127, 127));
}

} // namespace wpcore
