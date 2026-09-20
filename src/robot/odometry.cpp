#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"
#include "robot/odometry.h"

/* ODOMETRY FOR THE DRIVETRAIN */
// sensors
pros::Rotation vertical_sensor(2);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_sensor, 2, 0);

// odometry settings
lemlib::OdomSensors sensors(
    &vertical_tracking_wheel,   // vertical tracking wheel 1
    nullptr,                    // vertical tracking wheel 2
    nullptr,                    // horizontal tracking wheel 1
    nullptr,                    // horizontal tracking wheel 2
    &imu                        // inertial sensor
);

//lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
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
double target = 0.0;
bool do_lift_PID = true;
const double kP = 0.0, kI = 0.0, kD = 0.0, kG = 0.0;

void lift_PID(void*){
    double error = 0.0;
    double prev_error = 0.0;
    double integral = 0.0;
    double derivative = 0.0;
    const int dt = 0.0;

    while(true){
        double pos = potentiometer.get_value();
        error = target - pos;

        if(std::abs(error) < 100){
            integral += error * dt;
        }else{
            integral = 0;
        }

        derivative = (error - prev_error) / dt;
        prev_error = error;

        double pid = (error * kP) + (integral * kI) + (derivative * kD) + kG;

        if(pid > 127){
            pid = 127;
        }else if(pid < -127){
            pid = -127;
        }

        lift.move(pid);
        pros::delay(dt);
    }
}