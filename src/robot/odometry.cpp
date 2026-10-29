#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"
#include "robot/odometry.h"
#include "robot/auto-pid.h"
#include "robot/functions.h"

/* ODOMETRY FOR THE DRIVETRAIN */
pros::Imu imu(9);
// pros::Rotation horizontal_sensor(4);
// lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, 1);

// odometry settings
lemlib::OdomSensors sensors(
    nullptr,                    // vertical tracking wheel 1
    nullptr,                    // vertical tracking wheel 2
    nullptr, //&horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr,                    // horizontal tracking wheel 2
    &imu                        // inertial sensor
);

//lateral PID controller
lemlib::ControllerSettings lateral_controller(6.5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.75, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              14.9, // derivative gain (kD)
                                              0, // anti windup
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
// table of degrees and inches
const double LIFT_CAL_DEG[] = {0, 80, 114, 154, 178, 206, 245, 274, 282, 330, 377, 435, 490, 552};
const double LIFT_CAL_IN[]  = {0, 3, 7, 10, 11.5, 13.5, 16, 18.5, 19, 22, 25, 28, 30.5, 32.4};
const int LIFT_CAL_POINTS = sizeof(LIFT_CAL_DEG) / sizeof(LIFT_CAL_DEG[0]);

const double LIFT_MAX_HEIGHT = LIFT_CAL_IN[LIFT_CAL_POINTS - 1];
const double LIFT_TOP_MOTOR_DEG = LIFT_CAL_DEG[LIFT_CAL_POINTS - 1];

// map degrees to inches
double lift_deg_to_height(double deg){
    int i = 1;
    while(i < LIFT_CAL_POINTS - 1 && deg > LIFT_CAL_DEG[i]) i++;
    double d0 = LIFT_CAL_DEG[i - 1], d1 = LIFT_CAL_DEG[i];
    double h0 = LIFT_CAL_IN[i - 1], h1 = LIFT_CAL_IN[i];
    return h0 + (deg - d0) * (h1 - h0) / (d1 - d0);
}

int lift_motor_dir = 1;
double target = 0.0;
bool do_lift_PID = true;

// PID constants (in mV)
const double kP = 3000.0;   // mV per inch of error
const double kI = 5000.0;   // mV per inch * second of leftover error within 1 in.
const double kD = 100.0;    // mV per in/s of lift speed
const double kG = 150.0;    // constant mV to oppose gravity
const double kS = 400.0;    // address friction issues

/**
 * @brief catch lift faults
 * 
 * Check for faults in the lift, disable do_lift_PID, and print to brain
 */
void lift_fault(const char* msg, double height){
    do_lift_PID = false;
    lift.move_voltage(0);
    printf("[lift] %s (height %.2f in, motors %.0f deg)\n", msg, height, get_lift_motor_deg());
    pros::lcd::print(7, "LIFT: %s", msg);
}

/**
 * @brief run lift PID
 * 
 * Use PID constants and math to calculate PID
 */
void lift_PID(void*){
    const int dt = 10;
    const double max_volts = 12000;

    // zero the encoders with the lift at the bottom, in degrees
    lift.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    lift.tare_position_all();
    pros::delay(20);

    // coast mode for when PID is off
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    // values to use in PID calculations
    double integral = 0.0;
    double velocity = 0.0;
    double prev_height = get_lift_height();
    double last_goal = -1;
    int side = 0;
    int push_time = 0;
    double push_start = prev_height;
    double push_dir = 0;

    while(true){
        double height = get_lift_height();

        // smoothly change velocity
        double raw_velocity = (height - prev_height) / (dt / 1000.0);
        velocity += 0.5 * (raw_velocity - velocity);
        prev_height = height;

        if(do_lift_PID){
            // calc PID
            double goal = std::clamp(target, 0.0, LIFT_MAX_HEIGHT);
            double error = goal - height;

            double gravity = (goal > 0.25) ? kG : 0;
            double friction = (std::abs(error) > 0.05) ? kS * std::clamp(error / 0.3, -1.0, 1.0) : 0;

            double output = kP * error - kD * velocity + kI * integral + gravity + friction;

            // reset integral gain on a new target
            if(goal != last_goal){
                integral = 0;
                side = 0;
                last_goal = goal;
            }

            // stop overshoot
            if(std::abs(error) > 0.1){
                int new_side = (error > 0) ? 1 : -1;
                if(side != 0 && new_side != side) integral *= 0.3;
                side = new_side;
            }

            // calc integral
            if(std::abs(error) < 1.0 && std::abs(output) < max_volts){
                integral += error * dt / 1000.0;
            }
            integral = std::clamp(integral, -4000 / kI, 4000 / kI);
            if(goal <= 0.25 && height < 0.5) integral = 0;

            output = std::clamp(output, -max_volts, max_volts);

            // hardstops
            if(height >= LIFT_MAX_HEIGHT + 0.5 && output > 0) output = 0;
            if(height <= -0.5 && output < 0) output = 0;
            if(goal == 0 && height < 1) output = std::min(output, -4000.0);

            // if lift isn't moving >1in in the right way every .5s
            if(std::abs(output) > 6000){
                double dir = (output > 0) ? 1 : -1;
                if(push_time == 0 || dir != push_dir){
                    push_dir = dir;
                    push_start = height;
                    push_time = 0;
                }
                push_time += dt;

                if(push_time >= 500){
                    double moved = (height - push_start) * push_dir;
                    if(moved < -0.5){
                        lift_fault("moved backwards, flip lift_motor_dir", height);
                        output = 0;
                    }else if(moved < 1.0){
                        lift_fault("not moving / sensor not tracking", height);
                        output = 0;
                    }
                    push_time = 0;
                }
            }else{
                push_time = 0;
            }

            if(do_lift_PID) lift.move_voltage(output * lift_motor_dir);
        }else{
            push_time = 0;
        }

        pros::delay(dt);
    }
}