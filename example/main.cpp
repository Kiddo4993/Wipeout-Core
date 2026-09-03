// Full port of the team's LemLib main.cpp to Wipeout Core. Every motor port, sensor
// port, dimension, and PID gain is copied straight from that file --
// everywhere the old code called `chassis.something()`, this file calls
// `drivetrain.something()` instead (that rename, plus the lemlib:: -> wpcore::
// type renames, are the only things that changed). See README.md for the
// full mapping table and the math behind moveToPoint/turnToHeading/odometry.
//
// This file is meant to replace main.cpp in the actual PROS project -- drop
// include/wpcore and src/wpcore into that project first (see README.md
// "Installing into your PROS project").
#include "main.h"
#include "wpcore/api.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/rotation.h"
#include "pros/rotation.hpp"

using namespace pros; // IWYU pragma: keep

pros::MotorGroup left_motors({1, 9, 4}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({-3, -5, -10}, pros::MotorGearset::blue);
pros::MotorGroup liftMotor({-11, 20});
// pros::Motor intakeMotors({});
// pros::Motor chainbar({});
pros::Controller master(pros::E_CONTROLLER_MASTER); // create a controller object for the master controller
pros::adi::Pneumatics intake_piston('H', false, true); // create a piston object for the pneumatic piston on port 1
// pros::Rotation chainbar_encoder(17);

// --- drivetrain hardware config ---------------------------------------
// was: lemlib::Drivetrain drivetrain(...)
wpcore::DriveConfig driveConfig{&left_motors, // left motor group
                              &right_motors, // right motor group
                              9.5, // 9.5 inch track width
                              wpcore::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
};

pros::Imu imu(2);
pros::Rotation verticalrotation_sensor(13);
pros::Rotation horizontalrotation_sensor(21);

// was: lemlib::TrackingWheel horizontal_tracking_wheel/vertical_tracking_wheel
wpcore::TrackingWheel horizontal_tracking_wheel(&horizontalrotation_sensor, 2.0, -2);
// vertical tracking wheel
wpcore::TrackingWheel vertical_tracking_wheel(&verticalrotation_sensor, 2.0, 0);

// was: lemlib::OdomSensors sensors(...)
wpcore::OdomConfig odomConfig{&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
};

// was: lemlib::ControllerSettings lateral_controller(...)
wpcore::PIDGains lateralGains{6, // proportional gain (kP)
                            0, // integral gain (kI)
                            15, // derivative gain (kD)
                            0, // anti windup
                            0, // small error range, in inches
                            0, // small error range timeout, in milliseconds
                            0, // large error range, in inches
                            0, // large error range timeout, in milliseconds
                            0 // maximum acceleration (slew)
};

// was: lemlib::ControllerSettings angular_controller(...)
wpcore::PIDGains angularGains{0.5, // proportional gain (kP)
                            0, // integral gain (kI)
                            20, // derivative gain (kD)
                            3, // anti windup
                            1, // small error range, in degrees
                            100, // small error range timeout, in milliseconds
                            3, // large error range, in degrees
                            500, // large error range timeout, in milliseconds
                            0 // maximum acceleration (slew)
};

// was: lemlib::Chassis chassis(...); this is the "instead of chassis it's
// drivetrain" rename -- the object every autonomous/opcontrol call below
// goes through is named `drivetrain`, not `chassis`.
wpcore::Robot drivetrain(driveConfig, // drivetrain hardware config
                       lateralGains, // lateral PID gains
                       angularGains, // angular PID gains
                       odomConfig // odometry sensors
);

// Moves the chainbar to targetPosition (centidegrees, from chainbar_encoder)
// using a PID loop. Runs as its own pros::Task, so calling this function does
// not block the caller -- use it as a macro (e.g. on a button press) to send
// the chainbar to a preset position while the rest of opcontrol keeps running.
// This is a standalone mechanism PID (not part of Wipeout Core -- chainbar isn't a
// tracked axis, just a raw position-hold loop) so it's untouched by the port.
// void chainbarFunction(int targetPosition) {
//     static pros::Task* chainbarTask = nullptr;
//     static volatile bool stopRequested = false;
//     static volatile bool taskFinished = true;

//     if (chainbarTask != nullptr) {
//         stopRequested = true;
//         uint32_t waitStart = pros::millis();
//         while (!taskFinished && pros::millis() - waitStart < 100) {
//             pros::delay(5);
//         }
//         delete chainbarTask;
//         chainbarTask = nullptr;
//     }

//     stopRequested = false;
//     taskFinished = false;

//     chainbarTask = new pros::Task([targetPosition]() {
//         double kP = 8;
//         double kI = 0.0;
//         double kD = 3;

//         double error = 0;
//         double previousError = 0;
//         double integral = 0;
//         double derivative = 0;

//         const double errorThreshold = 1000; // centidegrees (~0.2 degrees) considered "at target"

//         while (!stopRequested) {
//             error = targetPosition - chainbar_encoder.get_position();

//             integral += error;
//             derivative = error - previousError;

//             double output = (kP * error) + (kI * integral) + (kD * derivative);

//             if (output > 70) output = 70;
//             if (output < -70) output = -70;

//             chainbar.move(output);

//             previousError = error;

//             if ((error < 0 ? -error : error) < errorThreshold) break;

//             pros::delay(10);
//         }

//         chainbar.brake();
//         taskFinished = true;
//     });
// }

// initialize function. Runs on program startup
void initialize() {
    intake_piston.set_value(false);
    pros::lcd::initialize(); // initialize brain screen
    // chainbar_encoder.reset_position(); // zero the chainbar encoder wherever it is at power-on
    drivetrain.calibrate(); // was: chassis.calibrate(); -- calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", drivetrain.getPose().x); // x
            pros::lcd::print(1, "Y: %f", drivetrain.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", drivetrain.getPose().theta); // heading
            // pros::lcd::print(3, "Chainbar: %d", chainbar_encoder.get_position());
            pros::delay(100);
        }
    });
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void autoskills() {
    // chainbar.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    // drivetrain.setPose(0, -62, 180);

    // intake_piston.set_value(false);
    // drivetrain.moveToPoint(0,   -58,  500, {.forwards = false}, false);
    // drivetrain.moveToPoint(0,   -64,  500, {.forwards = true, .minSpeed = 60}, false);
    // drivetrain.moveToPoint(0,   -58,  500, {.forwards = false}, false);
    // drivetrain.moveToPoint(0,   -64,  500, {.forwards = true, .minSpeed = 60}, false);
    // drivetrain.moveToPoint(0,   -50,  1000, {.forwards = false}, false);
    // chainbarFunction(-2500);
    // drivetrain.turnToHeading(270, 600, {}, false);
    // drivetrain.moveToPoint(-18.2,   -50,  900, {.forwards = true}, false);
    // chainbarFunction(0);
    // delay(300);
    // intake_piston.set_value(true);

    // delay(300);
    // chainbarFunction(0);
    // drivetrain.moveToPoint(-8,   -51,  900, {.forwards = false}, false);
    // drivetrain.turnToHeading(325, 600, {}, false);
    // drivetrain.moveToPoint(-24,   -27,  1700, {.forwards = true, .maxSpeed = 30}, true);
    // drivetrain.waitUntil(20);
    // intake_piston.set_value(false);
    // drivetrain.waitUntilDone();
    // drivetrain.turnToHeading(0, 600, {}, false);
    // chainbarFunction(-15000);
    // drivetrain.moveToPoint(-24,   -45,  1500, {.forwards = false}, false);
    // chainbarFunction(-108000);
    // delay(2000);
    // intake_piston.set_value(true);
    //  chainbarFunction(-8000);

    //  drivetrain.turnToHeading(25, 600);
    //  drivetrain.moveToPoint(0, 0, 1000, {.forwards = true}, false);
}

// --- PID tuning helpers --------------------------------------------------
// Swap one of these into autonomous() temporarily while dialing in gains.
// Each one drives a fixed, repeatable motion, prints live numbers to the
// brain screen as it runs, and logs every tick to the SD card -- pull the
// card afterward and graph error vs. time instead of just watching the
// robot and guessing whether that last kD change helped. See README.md's
// "Tuning PID with this library" section for the order to actually turn
// kP/kI/kD in.
void tuneLateralPID() {
    drivetrain.setPose(0, 0, 0);
    drivetrain.moveToPoint(0, 24, 3000, {.logFile = "/usd/lateral_tune.csv", .telemetry = true});
    drivetrain.waitUntilDone();
}

void tuneAngularPID() {
    drivetrain.setPose(0, 0, 0);
    drivetrain.turnToHeading(90, 2000, {.logFile = "/usd/angular_tune.csv", .telemetry = true});
    drivetrain.waitUntilDone();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
   // Tuning gains right now? Call tuneLateralPID() or tuneAngularPID()
   // here instead and leave the real routine below alone.
   drivetrain.setPose(0, 0, 0);
   drivetrain.turnToHeading(90, 2000);

    // Right side
    // drivetrain.setPose(-62, 0, 270.0);

    // intake_piston.set_value(false);
    // drivetrain.moveToPoint(-59.5,   0,  400, {.forwards = false}, false);
    // drivetrain.moveToPoint(-68,   0,  400, {.forwards = true, .minSpeed = 100}, false);
    // drivetrain.moveToPoint(-60,   0,  400, {.forwards = false}, false);
    // drivetrain.moveToPoint(-68,   0,  400, {.forwards = true, .minSpeed = 100}, false);
    // drivetrain.moveToPoint(-58,   0,  400, {.forwards = false}, false);
    // drivetrain.moveToPoint(-58.5,   -17,  1200, {.forwards = false}, false);
    // drivetrain.turnToHeading(302, 600, {}, false);
    // drivetrain.moveToPoint(-57.3,   -17.55,  800, {.forwards = false}, false);
    // delay(300);
    // intake_piston.set_value(true);
    // drivetrain.moveToPoint(-59,   -16.5,  1200, {.forwards = true}, false);
    // drivetrain.turnToHeading(42, 600, {}, false);
    // drivetrain.moveToPoint(-61,   -14.5,  800, {.forwards = false}, false);
    // delay(600);
    // intake_piston.set_value(false);
    // delay(500);
    // drivetrain.moveToPoint(-59,   -16.5,  1200, {.forwards = true}, false);
    // drivetrain.turnToHeading(180, 600, {}, false);
    // drivetrain.moveToPoint(-59,   16.5,  1200, {.forwards = false}, false);
    // intake_piston.set_value(true);
    // drivetrain.moveToPoint(-59,   16.5,  1200, {.forwards = false}, false);

    // Left side
    // drivetrain.setPose(0, -62, 180);

    // intake_piston.set_value(false);
    // drivetrain.moveToPoint(0,   -58,  500, {.forwards = false}, false);
    // drivetrain.moveToPoint(0,   -64,  500, {.forwards = true, .minSpeed = 60}, false);
    // drivetrain.moveToPoint(0,   -58,  500, {.forwards = false}, false);
    // drivetrain.moveToPoint(0,   -64,  500, {.forwards = true, .minSpeed = 60}, false);
    // drivetrain.moveToPoint(0,   -49.7,  1000, {.forwards = false}, false);
    // chainbarFunction(-2500);
    // drivetrain.turnToHeading(271, 600, {}, false);
    // drivetrain.moveToPoint(-18.2,   -49.7,  900, {.forwards = true}, false);
    // chainbarFunction(0);
    // delay(300);
    // intake_piston.set_value(true);
    // delay(300);
    // chainbarFunction(0);
    // drivetrain.moveToPoint(-8,   -51,  900, {.forwards = false}, false);
    // drivetrain.turnToHeading(323, 600, {}, false);
    // drivetrain.moveToPoint(-24.5,   -27,  1900, {.forwards = true, .maxSpeed = 30}, true);
    // drivetrain.waitUntil(20.9);
    // intake_piston.set_value(false);
    // drivetrain.waitUntilDone();
    // drivetrain.turnToHeading(0, 600, {}, false);
    // chainbarFunction(-15000);
    // drivetrain.moveToPoint(-26,   -45,  1500, {.forwards = false}, false);
    // chainbarFunction(-108000);
    // delay(2000);
    // intake_piston.set_value(true);
    //  chainbarFunction(-8000);
}

void opcontrol() {
    while (true) {
        int turn = master.get_analog(ANALOG_LEFT_Y);
        int dir = master.get_analog(ANALOG_RIGHT_X);
        left_motors.move(dir - turn);
        right_motors.move(dir + turn);

        // Lift / intake -- independent of chainbar
        if (master.get_digital(DIGITAL_L1)) {
            liftMotor.move(127);
        } else if (master.get_digital(DIGITAL_L2)) {
            liftMotor.move(-127);
        } else {
            liftMotor.move(0);
        }

        // if (master.get_digital(DIGITAL_R2)) {
        //     chainbar.move(75);
        // } else if (master.get_digital(DIGITAL_R1)) {
        //     chainbar.move(-75);
        // } else {
        //     chainbar.brake();
        // }

        if (master.get_digital_new_press(DIGITAL_R1)) {
            intake_piston.toggle(); // toggle piston state
        }
        // if (master.get_digital_new_press(DIGITAL_X)) {
        //     intake_piston.set_value(false);
        //     chainbarFunction(-91427);
        // }
        // if (master.get_digital_new_press(DIGITAL_DOWN)) {
        //     chainbarFunction(-90000);
        //     intake_piston.set_value(true);
        // } else if (master.get_digital_new_press(DIGITAL_UP)) {
        //     intake_piston.set_value(true);
        //     chainbarFunction(-105600);
        // }

        pros::delay(20); // Run for 20ms then update
    }
}
