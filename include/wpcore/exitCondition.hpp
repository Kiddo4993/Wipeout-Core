#pragma once

namespace wpcore {

/// Answers one question, over and over: "are we there yet?" A motion counts
/// as settled once its error has stayed inside `range` continuously for
/// `timeoutMs` -- not just touched that range for an instant, since a robot
/// that's still oscillating will dip in and out of a tight range without
/// actually being done. Feed it the current error once per loop tick.
///
/// Robot pairs two of these per motion: a tight range with a short timeout,
/// and a looser range with a longer timeout. Whichever one fires first ends
/// the motion -- that way a motion that nails the target exits fast, but
/// one that gets *close* and just can't quite squeeze out the last bit of
/// error (a common symptom of kP being a touch low) doesn't hang around
/// forever waiting for perfection.
class ExitCondition {
public:
    ExitCondition(double range, int timeoutMs);

    /// Feed it the current absolute error. Returns true once that error has
    /// been sitting inside `range` for `timeoutMs` straight.
    bool getExit(double error);

    void reset();

private:
    double range;
    int timeoutMs;
    long windowStart = -1; // when we most recently entered range; -1 = currently outside it
};

} // namespace wpcore
