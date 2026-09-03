#pragma once
#include "wpcore/pose.hpp"
#include "wpcore/driveHardware.hpp"
#include "pros/rtos.hpp"

namespace wpcore {

/// This is the class that answers "where's the robot right now" without
/// ever looking at what the drive motors are doing -- it only reads
/// sensors. That's on purpose: motors slip, wheels skid, voltage doesn't
/// map cleanly to speed, but a free-spinning tracking wheel just tells you
/// the truth about how far it rolled. Call update() on a fixed loop and
/// Robot will handle that for you, at 10ms, from a background task.
///
/// --- The idea, and the math, for whenever a judge asks "how does this work" ---
///
/// Every tick, two numbers come in: how far each tracking wheel rolled
/// since last tick, and how much the heading changed since last tick (from
/// the IMU). The tricky part is that a tracking wheel almost never sits
/// exactly on the robot's center of rotation -- it's offset by some
/// distance `r`. That means when the robot turns, the wheel picks up motion
/// that has nothing to do with the robot actually going anywhere: spin in
/// place by an angle `dTheta` and a wheel sitting `r` inches off-center
/// still gets dragged through an arc `r * dTheta` inches long, even though
/// the robot's center never moved an inch. If you don't account for that,
/// every turn looks like the robot also drove forward, and your position
/// estimate slowly wanders off from reality.
///
/// So the real job is: given what the wheel just read, and how much we just
/// rotated, back out the part of that reading that was *actual*
/// translation. That's a classic bit of circle geometry -- picture the
/// tracking center sweeping a small arc of radius `wheelDelta/dTheta +
/// offset` through angle `dTheta`; the straight-line (chord) distance that
/// arc covers is:
///
///     local = 2 * sin(dTheta/2) * (wheelDelta/dTheta + offset)   [dTheta != 0]
///     local = wheelDelta                                        [dTheta == 0, nothing to correct for]
///
/// Run that once for the forward/back wheel and once for the strafe wheel
/// and you get how far the robot moved in its *own* frame for that one
/// tick -- "forward 0.3 inches, right 0.02 inches," from the robot's point
/// of view. The last step is turning that into field coordinates, which
/// just means rotating it by whatever the robot's average heading was
/// during the tick (heading at the start, plus half of however much it
/// turned). That average-heading rotation is what lets a robot sweeping
/// through a curve get tracked accurately as a curve, instead of odometry
/// only working for straight lines.
class Odometry {
public:
    explicit Odometry(OdomConfig sensors);

    /// Pulls fresh sensor readings and updates the pose. Call this once per
    /// control loop tick (~10ms) -- Robot's background task already does.
    void update();

    Pose getPose() const;

    /// Tells odometry "actually, we're here" -- used at the start of
    /// autonomous, or any time you know the real position better than the
    /// sensors currently think you do. Also re-anchors the heading math
    /// internally so theta reads correctly on the very next update(), not
    /// eventually.
    void setPose(const Pose& pose);

    /// Zeroes every sensor and resets internal state. Call once, from
    /// initialize().
    void calibrate();

private:
    OdomConfig sensors;
    Pose pose;
    mutable pros::Mutex poseMutex;

    double prevVertical = 0;
    double prevHorizontal = 0;
    double prevHeading = 0;   // last reported heading (continuous, offset applied), degrees
    double headingOffset = 0; // added to raw IMU rotation to get reported heading
};

} // namespace wpcore
