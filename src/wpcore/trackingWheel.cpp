#include "wpcore/trackingWheel.hpp"
#include "wpcore/angle.hpp"

namespace wpcore {

TrackingWheel::TrackingWheel(pros::Rotation* sensor, double wheelDiameter, double offset, double gearRatio)
    : sensor(sensor), wheelDiameter(wheelDiameter), offset(offset), gearRatio(gearRatio) {}

double TrackingWheel::getDistance() const {
    double degrees = sensor->get_position() / 100.0; // the sensor speaks in centidegrees, so /100 first
    double radians = degToRad(degrees);
    return radians * (wheelDiameter / 2.0) * gearRatio; // arc length = angle(rad) * radius
}

double TrackingWheel::getOffset() const { return offset; }

void TrackingWheel::reset() { sensor->reset_position(); }

} // namespace wpcore
