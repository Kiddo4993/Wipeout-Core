#pragma once
#include <cstdio>

namespace wpcore {

/// A tiny CSV logger, built for one job: tuning PID gains.
///
/// Watching a robot drive and just eyeballing "yeah that looked kinda
/// wobbly" is how you end up tweaking kD for twenty minutes without actually
/// fixing anything. Point a motion at a file with this instead, pull the SD
/// card after the run, and open the CSV in Sheets or Excel (or paste it into
/// Desmos) -- the overshoot, the oscillation, the settle time are all just
/// sitting there in a graph instead of a memory of "it felt off."
///
/// SD card paths on the V5 brain look like "/usd/whatever.csv". If there's
/// no SD card in the brain, every write silently does nothing instead of
/// crashing your autonomous -- so it's always safe to leave logging turned
/// on, even on a brain with no card in it.
class Logger {
public:
    explicit Logger(const char* path);
    ~Logger();

    /// One row per control-loop tick: how far into the motion we are (ms),
    /// what the error was, and what the controller decided to do about it.
    void log(double timeMs, double error, double output);

private:
    FILE* file = nullptr;
};

} // namespace wpcore
