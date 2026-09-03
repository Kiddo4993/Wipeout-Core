#include "wpcore/logger.hpp"

namespace wpcore {

Logger::Logger(const char* path) {
    file = std::fopen(path, "w");
    // No SD card, bad path, whatever -- fopen just returns nullptr and log()
    // below already checks for that, so there's nothing else to do here.
    if (file) std::fprintf(file, "time_ms,error,output\n");
}

Logger::~Logger() {
    if (file) std::fclose(file);
}

void Logger::log(double timeMs, double error, double output) {
    if (!file) return;
    std::fprintf(file, "%.1f,%.4f,%.4f\n", timeMs, error, output);
}

} // namespace wpcore
