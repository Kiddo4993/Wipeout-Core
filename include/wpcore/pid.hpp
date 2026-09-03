#pragma once

namespace wpcore {

/// A PID controller, the same one you'd find in any textbook, just written
/// out ourselves instead of taken on faith:
///
///     output = kP*error + kI*(running total of error) + kD*(how fast error is changing)
///
/// Each term is answering a different question. kP asks "how wrong am I
/// right now" and reacts proportionally -- bigger error, bigger push. kI
/// asks "how wrong have I *been*", and slowly cranks up the output if a
/// small error just refuses to go away (friction, an uneven wheel, whatever
/// keeps you a hair short of target). kD asks "am I closing in fast", and
/// leans against the other two terms as error shrinks quickly, so the robot
/// eases into the target instead of slamming into it and bouncing back.
///
/// One assumption baked in: update() gets called roughly every 10ms, same
/// as the odometry task. That's what lets kI and kD gains carry over
/// straight from an existing LemLib project without retuning -- LemLib runs
/// on the same assumption, so the numbers mean the same thing here.
class PID {
public:
    /// @param kP proportional gain
    /// @param kI integral gain
    /// @param kD derivative gain
    /// @param windupRange how close to the target (same units as the error
    ///        you're feeding in) the integral term is even allowed to grow.
    ///        0 means no limit. Without this, a robot that starts way off
    ///        target can build up a huge integral term before it ever gets
    ///        close, and that stored-up value then rockets it past the
    ///        target once it finally arrives -- windupRange keeps the
    ///        integral silent until it's actually relevant.
    /// @param resetOnSignFlip if true, the integral snaps back to zero the
    ///        instant the error changes sign. That sign flip means we just
    ///        overshot the target, and whatever integral built up chasing
    ///        it from the other side is now stale -- left alone, it would
    ///        keep pushing the same direction and fight the correction back.
    PID(double kP, double kI, double kD, double windupRange = 0, bool resetOnSignFlip = true);

    /// Hand it the current error, get back what to do about it.
    double update(double error);

    /// Clears the integral total and the "last error" the derivative term
    /// compares against. Call this before starting any new motion --
    /// otherwise it starts the new motion still carrying baggage from
    /// whatever it was doing last.
    void reset();

    double kP, kI, kD;
    double windupRange;
    bool resetOnSignFlip;

private:
    double integral = 0;
    double previousError = 0;
    bool hasPreviousError = false;
};

} // namespace wpcore
