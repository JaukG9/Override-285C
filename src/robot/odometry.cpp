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
//pros::Rotation horizontal_sensor(4);
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_sensor, lemlib::Omniwheel::NEW_2, 1);

// odometry settings (no vertical tracking wheels, so lemlib uses the drive motor encoders)
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
// the lift is run in inches, from the lift motors' built-in encoders (the rotation sensor
// slips / barely turns, so it can't track the lift). the encoders are zeroed when the lift
// task starts, so THE LIFT HAS TO BE ALL THE WAY DOWN WHEN THE PROGRAM STARTS.
//
// the lift doesn't rise the same amount per motor degree all the way up (215 motor deg only
// got it ~9in, not half of 23.6), so height comes from a table of measured points with
// straight lines between them. to add a point: put the lift at some height, measure it with
// a tape measure, read "mot" on the controller, and add both (keep them in order, bottom first).
// more points = more accurate
const double LIFT_CAL_DEG[] = {0, 82, 154, 180, 230, 262, 308, 381, 440, 669};    // motor degrees (the "mot" number)
const double LIFT_CAL_IN[]  = {0, 3, 6.5, 8, 11, 13, 16, 20, 23.5, 24.75};   // measured lift height at that point, inches
const int LIFT_CAL_POINTS = sizeof(LIFT_CAL_DEG) / sizeof(LIFT_CAL_DEG[0]);

const double LIFT_MAX_HEIGHT = LIFT_CAL_IN[LIFT_CAL_POINTS - 1];        // inches
const double LIFT_TOP_MOTOR_DEG = LIFT_CAL_DEG[LIFT_CAL_POINTS - 1];    // motor degrees at the top

// motor degrees -> inches using the table (keeps the slope of the end segments past the ends)
double lift_deg_to_height(double deg){
    int i = 1;
    while(i < LIFT_CAL_POINTS - 1 && deg > LIFT_CAL_DEG[i]) i++;
    double d0 = LIFT_CAL_DEG[i - 1], d1 = LIFT_CAL_DEG[i];
    double h0 = LIFT_CAL_IN[i - 1], h1 = LIFT_CAL_IN[i];
    return h0 + (deg - d0) * (h1 - h0) / (d1 - d0);
}

// +1 or -1: positive voltage on the lift motors has to move it up (flip if it drives the wrong way)
int lift_motor_dir = 1;

double target = 0.0;    // inches (same units as get_lift_height())
bool do_lift_PID = true;

// gains are in millivolts (move_voltage, -12000 to 12000). error is inches:
// kP = mV per inch of error (full 12V from 4in away)
// kI = mV per inch*second of leftover error, only within 1in of the target. it pulls out the
//      last bit of error. it starts over on every new target, since the bands make the force
//      needed to hold the lift different at every height
// kD = mV per in/s of lift speed (brakes it before the target so it doesn't overshoot)
// kG = constant mV to hold the lift up whenever it's off the bottom. the lift is banded, so
//      the bands do most of this: keep it low
// kS = extra mV to get past friction when it's close but not there (fades in over 0.3in)
// - stops short and slowly creeps in: raise kI (or kS a little)
// - overshoots / bounces: raise kD (or lower kI)
// - buzzes / jitters at the target: lower kS, then kD
const double kP = 3000.0, kI = 5000.0, kD = 150.0, kG = 150.0, kS = 400.0;

// stops the lift PID and says why on the brain screen + terminal
void lift_fault(const char* msg, double height){
    do_lift_PID = false;
    lift.move_voltage(0);
    printf("[lift] %s (height %.2f in, motors %.0f deg)\n", msg, height, get_lift_motor_deg());
    pros::lcd::print(7, "LIFT: %s", msg);
}

void lift_PID(void*){
    const int dt = 10;                  // ms per loop (motor encoders update every 10ms)
    const double max_volts = 12000;

    // zero the encoders with the lift at the bottom, in degrees
    lift.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    lift.tare_position_all();
    pros::delay(20);

    if(LIFT_TOP_MOTOR_DEG <= 0){
        // not measured yet: never drive the lift (and leave it free to push up by hand).
        // if anything turns the PID on, turn it back off and stop the motors so they don't
        // keep running whatever they were last told
        pros::lcd::print(7, "LIFT: measure LIFT_TOP_MOTOR_DEG");
        printf("[lift] LIFT_TOP_MOTOR_DEG isn't set, lift PID is off\n");
        do_lift_PID = false;
        while(true){
            if(do_lift_PID){
                do_lift_PID = false;
                lift.move_voltage(0);
            }
            pros::delay(dt);
        }
    }

    // coast so the lift can be moved by hand whenever the PID is off (the PID holds it itself
    // while it's on, so hold mode isn't needed)
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    double integral = 0.0;
    double last_goal = -1;              // to tell when the target changes
    int side = 0;                       // which side of the target it was last clearly on
    double velocity = 0.0;              // in/s, smoothed
    double prev_height = get_lift_height();
    int push_time = 0;                  // ms the PID has been pushing hard without the lift following
    double push_start = prev_height;
    double push_dir = 0;

    while(true){
        double height = get_lift_height();

        // both lift motors unplugged / not responding: never drive the lift blind
        if(!std::isfinite(height)){
            if(do_lift_PID) lift_fault("lift motors not responding", 0);
            pros::delay(dt);
            continue;
        }

        // speed from the measurement (not the error), so changing the target doesn't kick it
        double raw_velocity = (height - prev_height) / (dt / 1000.0);
        velocity += 0.5 * (raw_velocity - velocity);
        prev_height = height;

        if(do_lift_PID){
            double goal = std::clamp(target, 0.0, LIFT_MAX_HEIGHT);
            double error = goal - height;

            // gravity: only when it's meant to be off the bottom (don't push up while resting)
            double gravity = (goal > 0.25) ? kG : 0;
            // friction: a small push toward the target, faded in so it doesn't buzz at the target
            double friction = (std::abs(error) > 0.05) ? kS * std::clamp(error / 0.3, -1.0, 1.0) : 0;

            double output = kP * error - kD * velocity + kI * integral + gravity + friction;

            // new target: start the integral over (the bands pull differently at every height,
            // so what held it at the last height is wrong here)
            if(goal != last_goal){
                integral = 0;
                side = 0;
                last_goal = goal;
            }
            // clearly crossed over the target (past a 0.1in band so sensor jitter doesn't count):
            // whatever built up pushed it past, so drop most of it (stops overshoot / hunting)
            if(std::abs(error) > 0.1){
                int new_side = (error > 0) ? 1 : -1;
                if(side != 0 && new_side != side) integral *= 0.3;
                side = new_side;
            }

            // integrate only close to the target (farther out kP is doing the work) and when
            // not already maxed out (no windup)
            if(std::abs(error) < 1.0 && std::abs(output) < max_volts){
                integral += error * dt / 1000.0;
            }
            // the integral never needs to be more than ~a third of full power
            integral = std::clamp(integral, -4000 / kI, 4000 / kI);
            // resting on the bottom: nothing to hold, let it go
            if(goal <= 0.25 && height < 0.5) integral = 0;

            output = std::clamp(output, -max_volts, max_volts);

            // never drive past the top or below the bottom
            if(height >= LIFT_MAX_HEIGHT + 0.5 && output > 0) output = 0;
            if(height <= -0.5 && output < 0) output = 0;
            if(goal == 0 && height < 1) output = std::min(output, -4000.0);

            // pushing hard: the lift has to move at least 1in the right way every 500ms.
            // if not, the sensor isn't reading the lift right (or it's jammed): stop instead of
            // slamming into the top/bottom
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

