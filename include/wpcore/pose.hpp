#pragma once

namespace wpcore {

/// Where the robot is: x/y in inches, theta in degrees. Theta follows a
/// compass, not a protractor -- 0 means facing straight down the +y axis,
/// and it counts up clockwise from there. That's a deliberate choice, not
/// an accident: it's exactly what a V5 IMU reports, so odometry theta and
/// imu.get_heading() always mean the same thing and you never have to
/// remember which one needs flipping.
struct Pose {
    double x = 0;
    double y = 0;
    double theta = 0;

    Pose() = default;
    Pose(double x, double y, double theta = 0) : x(x), y(y), theta(theta) {}

    /// How far away (in a straight line, inches) another pose is from this one.
    double distanceTo(const Pose& other) const;

    /// Which way you'd have to face, standing here, to look straight at
    /// `other`. Doesn't care what this pose's own theta is -- it's answering
    /// "where is that thing", not "how far do I need to turn".
    double angleTo(const Pose& other) const;
};

} // namespace wpcore
