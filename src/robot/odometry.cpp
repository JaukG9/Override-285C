#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"
#include "robot/odometry.h"
#include "robot/auto-pid.h"

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
lemlib::ControllerSettings lateral_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
// 2.53, 0.101, 14.2

// angular PID controller
lemlib::ControllerSettings angular_controller(5.11, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              11.1, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
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
double target = 0.0;    // degrees (same units as get_rotation())
bool do_lift_PID = true;
const double kP = 0.0, kI = 0.0, kD = 0.0, kG = 0.0;

void lift_PID(void*){
    double error = 0.0;
    double prev_error = 0.0;
    double integral = 0.0;
    double derivative = 0.0;
    const int dt = 20;

    while(true){
        // rotation sensor reads centidegrees, target is in degrees
        double pos = rotation.get_position() / 100.0;
        error = target - pos;

        if(do_lift_PID){
            if(std::abs(error) < 8.8){
                integral += error * dt;
            }else{
                integral = 0;
            }

            derivative = (error - prev_error) / dt;

            double pid = (error * kP) + (integral * kI) + (derivative * kD) + kG;

            // green cartridge maxes out at 200rpm
            if(pid > 200){
                pid = 200;
            }else if(pid < -200){
                pid = -200;
            }

            lift.move_velocity(pid);
        }else{
            // start fresh when the PID turns back on (no leftover integral or derivative kick)
            integral = 0;
        }

        prev_error = error;
        pros::delay(dt);
    }
}