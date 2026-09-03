#pragma once
#include "pros/rotation.hpp"

namespace wpcore {

/// A rotation sensor bolted to a free-spinning omni wheel only tells you an
/// angle -- this wraps it so it tells you a distance instead, the way you
/// actually want to think about it.
///
/// The rotation sensor reports centidegrees (1/100th of a degree, so it can
/// give you sub-degree precision as a plain integer). Turning that into how
/// far the wheel's contact patch has rolled is just the arc-length formula
/// from geometry class: angle in radians times radius.
class TrackingWheel {
public:
    /// @param sensor the rotation sensor riding on this wheel's axle
    /// @param wheelDiameter the wheel's diameter, in inches -- measure it,
    ///        don't guess; a couple millimeters off here quietly throws off
    ///        every distance the whole library computes
    /// @param offset how far (inches) this wheel's contact point sits from
    ///        the robot's tracking center -- the point odometry reports x/y
    ///        for. Sign matters here: get it backwards and the robot will
    ///        report drifting sideways in a circle every time it just turns
    ///        in place, which is the tell that this needs flipping.
    /// @param gearRatio (sensor turns) / (wheel turns) -- leave it at 1.0 if
    ///        the sensor sits directly on the tracking wheel's axle with no
    ///        gearing between them
    TrackingWheel(pros::Rotation* sensor, double wheelDiameter, double offset, double gearRatio = 1.0);

    /// Total distance (inches) this wheel has rolled since it was last reset.
    double getDistance() const;

    double getOffset() const;

    void reset();

private:
    pros::Rotation* sensor;
    double wheelDiameter;
    double offset;
    double gearRatio;
};

} // namespace wpcore
