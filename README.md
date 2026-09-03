# Wipeout Core

Our own odometry + PID motion library for the PROS/V5 robot. It's shaped
like LemLib on purpose -- same idea of a hardware config, a sensor config,
and a set of PID gains all handed to one object -- but every formula inside
it is ours, written to be understood, not just imported.

## Why this exists

At some point a judge is going to ask "walk me through how your robot knows
where it is," and "we put lemlib::api.hpp at the top of the file" is not an
answer that goes anywhere. This library exists so that question has a real
answer: here's the sensor math, here's the control loop, here's why it's
built this way, and here's the actual code doing it.

It's not trying to out-engineer LemLib -- LemLib is excellent, and there's
no shame in using it once you understand what it's doing. This is that
understanding, made concrete enough to hand to a judge and to keep tuning
for the rest of the season.

## Layout

```
include/wpcore/     headers -- the public API
src/wpcore/         implementation
example/main.cpp     our actual main.cpp, ported from lemlib -> Wipeout Core
```

## The idea

A robot doing autonomous needs to answer two questions, over and over,
many times a second: **where am I**, and **what do I do about the gap
between where I am and where I want to be**. Odometry answers the first.
PID answers the second. Everything else in this library -- `moveToPoint`,
`turnToHeading`, the settle logic -- is just those two answers glued
together into something you can call from an autonomous routine.

## The math

### Odometry -- knowing where you are

The whole point of odometry is to track position without trusting the drive
motors, because motors lie: wheels slip, voltage doesn't map cleanly to
speed, and a wheel commanded to spin isn't always a wheel that actually
gripped the tile. A free-spinning tracking wheel doesn't have that problem
-- if it turned, it's because the ground under it moved past it that much,
full stop. So [odometry.cpp](src/wpcore/odometry.cpp) never looks at motor
commands at all. It only reads a tracking wheel's rotation sensor and the
IMU, every 10ms, and rebuilds position from just that.

Here's the wrinkle that makes this more interesting than "add up how far
the wheels turned": a tracking wheel almost never sits exactly on the
robot's actual center of rotation. It's offset from it by some distance
`r`. That means whenever the robot *turns*, the wheel picks up motion that
has nothing to do with the robot going anywhere -- spin in place by an
angle `dTheta`, and a wheel sitting `r` inches off-center still gets dragged
through an arc `r * dTheta` inches long, even though the robot's center
never moved at all. Ignore that, and every turn looks like the robot also
drove forward a little, and the position estimate slowly drifts away from
reality over the course of a match.

So each tick, the real job is: take what the wheel just measured, take how
much we just rotated, and back out the part of that measurement that was
*actual* translation. That's a circle-geometry problem -- picture the
tracking center sweeping a small arc through the same angle `dTheta`,
sweeping around a radius of `wheelDelta/dTheta + offset`. The straight-line
distance (the chord) that arc covers is:

```
local = 2 * sin(dTheta/2) * (wheelDelta/dTheta + offset)     dTheta != 0
local = wheelDelta                                           dTheta == 0
```

Do that once for the forward/back wheel and once for the strafing wheel and
you get exactly how far the robot moved *in its own frame* that tick --
"forward this much, sideways that much," from the robot's own point of
view. The last step is turning that into a field-coordinate move, which is
just a rotation by whatever the robot's heading averaged out to during that
tick (heading at the start of the tick, plus half of whatever it turned).
Using the *average* heading instead of the start or end heading is what
makes this accurate through a curve and not just along straight lines --
without it, every curved path would get tracked as a slightly-wrong series
of straight segments.

That rotated displacement just gets added onto the running `(x, y)`, and
`theta` gets set straight from the IMU. Add it all up, tick after tick, and
you get a position estimate that tracks the real robot closely enough to
drive by.

### PID -- deciding what to do about it

Once you know the error -- how far off you are from where you want to be
-- something has to turn that number into a motor voltage.
[pid.cpp](src/wpcore/pid.cpp) is the classic three-term answer:

```
output = kP * error + kI * (running total of error) + kD * (how fast error is changing)
```

Each term is doing a different job:

- **kP** reacts to *right now*. Bigger error, bigger push. On its own,
  a P-only controller almost always settles a little short of the target
  (motor friction and gravity eat some of that proportional push) and tends
  to overshoot and oscillate if you push kP too high trying to fix that.
- **kI** reacts to *history*. If a small error just refuses to close no
  matter how long you wait -- classic sign of friction or an unbalanced
  robot -- the integral term slowly grows until it's strong enough to push
  through it. It's powerful and also the easiest term to make things worse
  with, which is why it's the last one to touch.
- **kD** reacts to *rate of change*. As the error shrinks quickly (robot's
  barreling toward the target), kD leans against the other two terms so the
  robot eases in instead of overshooting and bouncing back. It's the anti-
  overshoot term.

Two extra guards worth knowing, since a judge might ask about them
specifically:

- **Anti-windup** (`windupRange`): the integral term only starts
  accumulating once the error is already within `windupRange`. Without
  this, a robot that starts far from its target builds up a massive
  integral total before it's even close, and that stored-up value then
  fires the robot straight past the target the moment it finally arrives.
- **Sign-flip reset**: the moment the error crosses zero, we just
  overshot -- and the integral built up chasing the target from the other
  side no longer describes reality. Left alone it would keep pushing the
  same direction and fight the very correction trying to bring it back, so
  it gets wiped instead.

One assumption threaded through all of this: `update()` is called roughly
every 10ms, which is also how fast the odometry task runs. That's what lets
gains tuned in LemLib drop straight into Wipeout Core's `PIDGains` and behave
the same way -- LemLib runs on the same 10ms assumption, so the numbers
mean the same thing here.

### Knowing when to stop

A PID loop never mathematically reaches *exactly* zero error -- it gets
close and hovers. [exitCondition.hpp](include/wpcore/exitCondition.hpp)
is the "close enough, for long enough" check: error has to sit inside a
range continuously for a timeout, not just touch it once, since an
oscillating robot will clip through a tight range without actually being
settled. Robot runs two of these per motion -- a tight range with a short
timeout, and a looser range with a longer timeout -- so a clean approach
exits fast, and a slightly-off approach still exits eventually instead of
hanging there forever waiting for perfection that kP alone can't deliver.

### Turning it into motion

[robot.cpp](src/wpcore/robot.cpp) is where odometry and PID actually
drive the robot:

- **`turnToHeading`** is the simple case: one PID loop on the shortest-path
  heading error, output sent as equal-and-opposite voltage to each side so
  the robot spins in place.
- **`moveToPoint`** runs two PID loops at once, sharing the same motors.
  Distance-to-target drives the lateral one; bearing-to-target drives the
  angular one. Their outputs combine the way every tank drive steers --
  `left = lateral + angular`, `right = lateral - angular` -- so adding a
  turn correction to one side and subtracting it from the other curves the
  robot toward the point while it's still driving forward, instead of
  turning first and driving second. One extra trick: the lateral output
  gets scaled by `cos(headingError)`. If the robot's currently facing 90
  degrees away from the target, "drive forward" is pointing 90 degrees away
  from anything useful -- so the more off-heading the robot is, the less
  the lateral term gets trusted, and the angular term gets to aim the robot
  before it commits to driving.

## Tuning PID with this library

This is the part most teams do half-blind -- nudge a gain, run the
autonomous, eyeball whether it "looked better," repeat. Wipeout Core builds in
two ways to actually *see* what a motion is doing instead of guessing:

- **`params.telemetry = true`** prints live numbers to the brain screen
  while the motion runs -- distance/heading error and motor output for
  `moveToPoint`, target/error/output for `turnToHeading`. Good for a quick
  look on the practice field.
- **`params.logFile = "/usd/whatever.csv"`** logs every tick (time, error,
  output) to that file on the SD card. Pull the card after the run, open
  the CSV in Sheets/Excel/Desmos, and graph error against time. Overshoot,
  oscillation, and settle time stop being impressions and start being a
  line you can point at.

Both are off by default, so a normal autonomous run stays quiet. Turn them
on for a dedicated tuning run:

```cpp
// somewhere you can trigger it on demand -- a controller button in
// opcontrol, or a temporary line in autonomous while testing
drivetrain.setPose(0, 0, 0);
drivetrain.moveToPoint(0, 24, 2000, {.logFile = "/usd/lateral_tune.csv", .telemetry = true});
drivetrain.waitUntilDone();
```

### The actual tuning order

Tune lateral and angular separately -- don't try to dial in a `moveToPoint`
run where both are wrong at once, you won't be able to tell which gain is
causing what.

1. **Zero out kI and kD.** Start with just kP.
2. **Raise kP** until the robot drives to the target and just barely starts
   to overshoot and oscillate around it. Then back off by about 10-20%.
   Too low and it'll be sluggish and undershoot; too high and it'll
   oscillate or slam into the target.
3. **Add kD** to kill that oscillation. kD reacts to how fast the error's
   closing, so it eases the robot in as it approaches instead of letting it
   barrel through. Raise it until the overshoot mostly disappears -- too
   much kD and the robot will feel hesitant or jittery, especially if your
   sensors are at all noisy.
4. **Only add kI if there's leftover steady-state error** -- the robot
   consistently stops a fixed distance short (or a fixed few degrees off)
   no matter how long you wait. Add a small amount and watch the log: it
   should slowly close that last bit of error without reintroducing
   oscillation. Most VEX drivetrains barely need any kI for lateral motion;
   turning against friction sometimes wants a little more.
5. **Set `smallError`/`smallErrorTimeout` and `largeError`/
   `largeErrorTimeout`** once the gains feel good. Small/tight range with a
   short timeout for a clean, fast exit; large/loose range with a longer
   timeout as the fallback so a run that gets close but not perfect still
   moves on instead of eating the rest of your 15 seconds.
6. **Read the log.** A good run's error-vs-time graph looks like a smooth
   curve settling toward zero with at most one small overshoot. A jagged,
   oscillating graph means back kP off or add more kD. A graph that flattens
   out above zero and stays there means either add a touch of kI, or your
   `smallError` range is tighter than the robot can actually reach.

## LemLib → Wipeout Core name mapping

| LemLib | Wipeout Core | notes |
|---|---|---|
| `lemlib::Drivetrain` | `wpcore::DriveConfig` | motors + physical dims |
| `lemlib::OdomSensors` | `wpcore::OdomConfig` | tracking wheels + imu |
| `lemlib::ControllerSettings` | `wpcore::PIDGains` | same fields, same order |
| `lemlib::Chassis chassis` | `wpcore::Robot drivetrain` | **the rename** — `chassis.foo()` → `drivetrain.foo()` |
| `lemlib::TrackingWheel` | `wpcore::TrackingWheel` | unchanged |
| `lemlib::Omniwheel::NEW_275` | `wpcore::Omniwheel::NEW_275` | unchanged |
| `chassis.moveToPoint(...)` | `drivetrain.moveToPoint(...)` | same params: `.forwards`, `.maxSpeed`, `.minSpeed`, plus new `.logFile`/`.telemetry` |
| `chassis.turnToHeading(...)` | `drivetrain.turnToHeading(...)` | same signature, plus `.logFile`/`.telemetry` |
| `chassis.waitUntil(...)` / `waitUntilDone()` | same on `drivetrain` | unchanged |
| *(none)* | `wpcore::Logger` | new -- CSV logging for tuning, see above |

`example/main.cpp` is our actual file with every port number, wheel size,
track width, and PID gain carried over unchanged -- only the library calls
changed.

## Installing into your PROS project

1. Copy `include/wpcore/` into your project's `include/` folder, and
   `src/wpcore/` into your project's `src/` folder.
2. PROS's default Makefile globs `src/**/*.cpp` and adds `include/` to the
   compiler's include path automatically -- no Makefile edits needed.
3. Swap `#include "lemlib/api.hpp"` for `#include "wpcore/api.hpp"` in
   `main.cpp` and follow the mapping table above (or just copy
   `example/main.cpp` over it and adjust).
4. LemLib can stay installed alongside Wipeout Core while you verify Wipeout Core
   matches on the practice field -- they don't conflict -- or pull it once
   you're confident.

Your editor may flag `#include "pros/*.hpp"` as unresolved in *this*
folder -- expected, those headers come from the PROS kernel template in
your actual project, not from this library on its own.

## One thing to verify on real hardware

`odometry.cpp` flips the sign of `imu.get_rotation()` to match Wipeout Core's
clockwise-positive convention -- that's a best guess at the V5 IMU's sign
behavior, called out right where it happens in the code. Spin the robot 90°
clockwise by hand after `calibrate()` and check that `drivetrain.getPose().theta`
goes up, not down. If it goes down, flip that one sign.
