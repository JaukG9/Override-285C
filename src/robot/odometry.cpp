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

double l_kP = 10, l_kI = 0, l_kD = 3, a_kP = 2, a_kI = 0, a_kD = 10;
void set_drive_controllers(double l_kP, double l_kI, double l_kD, double a_kP, double a_kI, double a_kD){
    lemlib::ControllerSettings lateral_controller(l_kP, l_kI, l_kD, 3, 1, 100, 3, 500, 0);
    lemlib::ControllerSettings angular_controller(a_kP, a_kI, a_kD, 3, 1, 100, 3, 500, 0);
}

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

void auto_PID(){
    bool ang_tuned = false;
    bool lat_tuned = false;

    // angular
    while(!ang_tuned){
        chassis.setPose(0, 0, 0);
        chassis.turnToHeading(90, 1000);

        std::vector<double> angles = {};
        int numLoops = 0;

        while(numLoops < 500){
            angles.push_back(chassis.getPose().theta - 90);
            pros::delay(2);
        }

        int maxFlips = 5;
        int flips = 0;
        int prev = 0;

        bool oscillating = false, overshooting = false, undershooting = false;
        
        for(int i = 1; i < angles.size(); i++){
            double diff = angles[i] - angles[i - 1];
            if(std::abs(diff) < 1e-5) continue;
            int curr = (diff >= 0) ? 1 : -1;
            if(prev != 0 && curr != prev){
                flips++;
                if(flips > maxFlips){
                    oscillating = true;
                }
            }
            prev = curr;

            int overshoots = 0, maxOvershoots = 0;
            int undershoots = 0, maxUndershoots = 0;

            if(curr == 1 && angles[i] > 0){
                overshoots++;
                overshooting = true;
            }else if(overshooting){
                maxOvershoots = overshoots;
                overshoots = 0;
                overshooting = false;
            }

            if(curr == -1 && angles[i] < 0){
                undershoots++;
                undershooting = true;
            }else if(undershooting){
                maxUndershoots = undershoots;
                undershoots = 0;
                undershooting = false;
            }
        }

        if(oscillating){
            a_kD += flips / 10;
        }else if(maxOvershoots){
            a_kP += 1;
        }
    }
}

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
        if(do_lift_PID){
            double pos = rotation.get_position();
            error = target - pos;

            if(std::abs(error) < 880){
                integral += error * dt;
            }else{
                integral = 0;
            }

            derivative = (error - prev_error) / dt;
            prev_error = error;

            double pid = (error * kP) + (integral * kI) + (derivative * kD) + kG;

            if(pid > 600){
                pid = 600;
            }else if(pid < -600){
                pid = -600;
            }

            lift.move_velocity(pid);
            pros::delay(dt);
        }
    }
}