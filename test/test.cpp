// Host-side tests for Wipeout Core, built against stub PROS headers.
#include "wpcore/api.hpp"
#include <atomic>
#include <cmath>
#include <cstdio>
#include <thread>

static int failures = 0;
#define CHECK(name, cond) do { if (cond) std::printf("ok - %s\n", name); else { std::printf("not ok - %s\n", name); failures++; } } while (0)
static bool near(double a, double b) { return std::fabs(a - b) < 0.05; }
static std::int32_t centidegreesFor(double inches, double diameter) { return static_cast<std::int32_t>(std::lround(inches / (diameter / 2.0) * 180.0 / M_PI * 100.0)); }

int main() {
    using namespace wpcore;
    {   // heading follows the IMU: clockwise is positive
        pros::Imu imu;
        OdomConfig cfg; cfg.imu = &imu;
        Odometry odom(cfg); odom.calibrate(); odom.setPose(Pose(0, 0, 0));
        imu.rotation = 90; odom.update();
        CHECK("turning clockwise 90 degrees gives a heading of 90", near(odom.getPose().theta, 90));
    }
    {   // after a right turn, driving forward moves +x (east), not -x
        pros::Imu imu; pros::Rotation sensor;
        TrackingWheel wheel(&sensor, 2.0, 0);
        OdomConfig cfg; cfg.imu = &imu; cfg.vertical1 = &wheel;
        Odometry odom(cfg); odom.calibrate(); odom.setPose(Pose(0, 0, 0));
        imu.rotation = 90; odom.update();
        sensor.centidegrees = centidegreesFor(10, 2.0); odom.update();
        Pose p = odom.getPose();
        CHECK("driving 10 in forward after a right turn ends at x=+10, y=0", near(p.x, 10) && near(p.y, 0));
    }
    pros::MotorGroup left, right;
    DriveConfig drive{&left, &right, 10, 2.75, 450, 2};
    PIDGains lateral; lateral.kP = 5;
    PIDGains angular; angular.kP = 2; angular.smallError = 1;
    {   // an async motion counts as running as soon as moveToPoint returns
        Robot robot(drive, lateral, angular, OdomConfig{});
        robot.moveToPoint(10, 10, 30, {}, true);
        std::atomic<bool> returned{false};
        std::thread waiter([&] { robot.waitUntilDone(); returned = true; });
        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        bool returnedEarly = returned;
        pros::Task::runPending();   // let the motion run (it times out after 30 ms) so the waiter finishes
        waiter.join();
        CHECK("waitUntilDone() waits for an async motion that hasn't been scheduled yet", !returnedEarly);
    }
    {   // within close range of the target, stop steering (no spin on overshoot)
        left.history.clear(); right.history.clear();
        Robot robot(drive, lateral, angular, OdomConfig{});
        robot.moveToPoint(0.5, 5, 25, {}, false);
        CHECK("no turning output within 7.5 in of the target", !left.history.empty() && left.history.front() == right.history.front());
    }
    {   // minSpeed shouldn't push the turn when it's already inside the settle range
        left.history.clear(); right.history.clear();
        Robot robot(drive, lateral, angular, OdomConfig{});
        TurnToHeadingParams params; params.minSpeed = 20;
        robot.turnToHeading(0.5, 25, params, false);
        CHECK("turnToHeading doesn't apply minSpeed inside smallError", !left.history.empty() && std::abs(left.history.front()) < 20);
    }
    std::printf("# fail %d\n", failures);
    return failures == 0 ? 0 : 1;
}
