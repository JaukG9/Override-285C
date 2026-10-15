#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"
#include "robot/odometry.h"
#include "robot/auto-pid.h"
#include "robot/functions.h"

/* ODOMETRY FOR THE DRIVETRAIN */
// sensors
pros::Imu imu(9);
pros::Rotation horizontal_sensor(4);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, 0.206);

// odometry settings (no vertical tracking wheels, so lemlib uses the drive motor encoders)
lemlib::OdomSensors sensors(
    nullptr,                    // vertical tracking wheel 1
    nullptr,                    // vertical tracking wheel 2
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr,                    // horizontal tracking wheel 2
    &imu                        // inertial sensor
);

//lateral PID controller
lemlib::ControllerSettings lateral_controller(6.3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              4.7, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.95, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              18, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// steering curve
lemlib::ExpoDriveCurve steer_curve(10, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
CustomChassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &steer_curve,
                        &steer_curve
);

/* ODOMETRY FOR THE LIFT */
// the lift is run in inches: the rotation sensor reads 0 -> 33 CENTIdegrees (0.33 deg, what
// get_position() returns raw) over the lift's 0 -> 23.6 in. one sensor count is ~0.7in
const double LIFT_MAX_HEIGHT = 23.6;    // inches
const double LIFT_MAX_ANGLE = 0.33;     // degrees
const double LIFT_IN_PER_DEG = LIFT_MAX_HEIGHT / LIFT_MAX_ANGLE;

double target = 0.0;    // inches (same units as get_lift_height())
bool do_lift_PID = true;

// starting gains for 2 green (200rpm) motors. output is rpm and error is inches, so kP = rpm
// per inch of error (full 200rpm from ~3.5in away), kD = rpm per in/s of closing speed (slows it
// down before the target), kI = rpm per in*s (only builds up within 2in of the target).
// kG is a constant rpm to hold against gravity. raise kG if the lift sags, raise kP if it's
// slow, raise kD if it bounces at the target
const double kP = 60.0, kI = 3.0, kD = 6.0, kG = 0.0;

void lift_PID(void*){
    double error = 0.0;
    double prev_error = 0.0;
    double integral = 0.0;
    double derivative = 0.0;
    const int dt = 10;   // ms per loop (faster loop = faster reaction)

    while(true){
        error = target - get_lift_height();

        if(do_lift_PID){
            // time is in seconds so kI and kD are per second, not per 20ms loop
            if(std::abs(error) < 2.0){
                integral += error * dt / 1000.0;
            }else{
                integral = 0;
            }

            // the sensor only has ~33 counts over the whole range, so the raw derivative spikes
            // every time it ticks over. smooth it
            double raw_derivative = (error - prev_error) / (dt / 1000.0);
            derivative += 0.2 * (raw_derivative - derivative);

            double pid = (error * kP) + (integral * kI) + (derivative * kD) + kG;

            // not capped at 200rpm: sent as voltage (60mV per "rpm", PROS limits it to +-12V),
            // which responds faster than move_velocity's internal velocity loop
            lift.move_voltage(pid * 60);
        }else{
            // start fresh when the PID turns back on (no leftover integral or derivative kick)
            integral = 0;
            derivative = 0;
        }

        prev_error = error;
        pros::delay(dt);
    }
}