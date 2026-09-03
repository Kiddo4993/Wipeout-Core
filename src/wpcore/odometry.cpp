#include "wpcore/odometry.hpp"
#include "wpcore/angle.hpp"
#include <cmath>

namespace wpcore {

Odometry::Odometry(OdomConfig sensors) : sensors(sensors) {}

void Odometry::calibrate() {
    if (sensors.vertical1) sensors.vertical1->reset();
    if (sensors.vertical2) sensors.vertical2->reset();
    if (sensors.horizontal1) sensors.horizontal1->reset();
    if (sensors.horizontal2) sensors.horizontal2->reset();
    if (sensors.imu) sensors.imu->reset(true); // blocking reset -- give the gyro time to actually settle before we trust it

    prevVertical = 0;
    prevHorizontal = 0;
    prevHeading = 0;
    headingOffset = 0;
}

void Odometry::update() {
    // --- 1. see what the sensors say ---
    double verticalRaw = sensors.vertical1 ? sensors.vertical1->getDistance() : 0;
    double horizontalRaw = sensors.horizontal1 ? sensors.horizontal1->getDistance() : 0;

    // get_rotation() instead of get_heading() because it's continuous -- it
    // can go past 360 or negative instead of wrapping, which matters a lot
    // for computing a clean deltaHeading right as we cross the 0/360 line.
    // One quirk to know about: the V5 IMU reports clockwise spin as
    // *negative* rotation, but Wipeout Core treats clockwise as positive (to
    // match get_heading() and plain compass headings), so the sign gets
    // flipped right here. This is the one line that convention lives in --
    // if headings ever seem to run backwards on the real robot, this is the
    // first place to look.
    double imuRaw = sensors.imu ? -sensors.imu->get_rotation() : 0;
    double reportedHeading = imuRaw + headingOffset;

    double verticalOffset = sensors.vertical1 ? sensors.vertical1->getOffset() : 0;
    double horizontalOffset = sensors.horizontal1 ? sensors.horizontal1->getOffset() : 0;

    // --- 2. how much changed since the last tick ---
    double deltaVertical = verticalRaw - prevVertical;
    double deltaHorizontal = horizontalRaw - prevHorizontal;
    double deltaHeadingDeg = reportedHeading - prevHeading;
    double deltaHeadingRad = degToRad(deltaHeadingDeg);

    // --- 3. local displacement -- the arc-chord correction from the class comment above ---
    double localY, localX;
    if (std::fabs(deltaHeadingRad) < 1e-9) {
        // Barely rotated (or didn't at all) this tick, so there's no
        // rotational contamination to remove -- the raw wheel deltas
        // already *are* the local displacement. Also sidesteps a division
        // by ~0 in the general formula below.
        localY = deltaVertical;
        localX = deltaHorizontal;
    } else {
        localY = 2.0 * std::sin(deltaHeadingRad / 2.0) * (deltaVertical / deltaHeadingRad + verticalOffset);
        localX = 2.0 * std::sin(deltaHeadingRad / 2.0) * (deltaHorizontal / deltaHeadingRad + horizontalOffset);
    }

    // --- 4. rotate that local displacement into field coordinates, using
    //         the average heading during this tick (start-of-tick heading
    //         plus half of whatever we turned) ---
    double avgHeadingRad = degToRad(prevHeading + deltaHeadingDeg / 2.0);
    double globalDX = localX * std::cos(avgHeadingRad) + localY * std::sin(avgHeadingRad);
    double globalDY = localY * std::cos(avgHeadingRad) - localX * std::sin(avgHeadingRad);

    // --- 5. and that's the new pose ---
    poseMutex.take();
    pose.x += globalDX;
    pose.y += globalDY;
    pose.theta = wrapDeg360(reportedHeading);
    poseMutex.give();

    prevVertical = verticalRaw;
    prevHorizontal = horizontalRaw;
    prevHeading = reportedHeading;
}

Pose Odometry::getPose() const {
    poseMutex.take();
    Pose p = pose;
    poseMutex.give();
    return p;
}

void Odometry::setPose(const Pose& newPose) {
    // Re-anchor the heading offset right now, using wherever the raw IMU
    // rotation currently sits, so the *next* update() call reports
    // newPose.theta plus at most one tick's worth of drift -- not whatever
    // the raw IMU total happens to say.
    double imuRaw = sensors.imu ? -sensors.imu->get_rotation() : 0;
    headingOffset = newPose.theta - imuRaw;
    prevHeading = newPose.theta;

    poseMutex.take();
    pose = newPose;
    pose.theta = wrapDeg360(newPose.theta);
    poseMutex.give();
}

} // namespace wpcore
