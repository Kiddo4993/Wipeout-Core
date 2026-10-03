// Host-side stand-ins for the PROS RTOS calls Wipeout Core uses (tests only).
#pragma once
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#define TIMEOUT_MAX 0xFFFFFFFFUL
namespace pros {
inline std::uint32_t millis() {
    static const auto start = std::chrono::steady_clock::now();
    return static_cast<std::uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count());
}
inline void delay(std::uint32_t ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }
class Mutex {
    std::mutex m;
public:
    bool take(std::uint32_t = TIMEOUT_MAX) { m.lock(); return true; }
    bool give() { m.unlock(); return true; }
};
// Tasks don't start on their own: tests run them with Task::runPending(), which
// reproduces "the task was created but hasn't been scheduled yet".
class Task {
public:
    template <class F> explicit Task(F&& f) { queue().push_back(std::function<void()>(std::forward<F>(f))); }
    static void runPending() { while (!queue().empty()) { auto f = queue().front(); queue().pop_front(); f(); } }
    static std::deque<std::function<void()>>& queue() { static std::deque<std::function<void()>> q; return q; }
};
} // namespace pros
