#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"
#include "robot/functions.h"
#include "robot/odometry.h"
#include "robot/autos.h"
#include "robot/auto-pid.h"

/**
 * @brief initialization
 * 
 * Runs initialization code. This occurs as soon as the program is started. 
 * Initializes the brain, chassis, and pneumatics.
 */
void initialize(){
	pros::lcd::initialize();
	// aivision_back.reset();
	// aivision_side.reset();
	chassis.calibrate();
	rotation.reset_position();

	claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	set_claw(true);

	pros::Task run_lift(lift_PID);
}

/**
 * @brief disabled state
 * 
 * Runs while the robot is in the disabled state of the Field Management System
 * or the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled(){}

/**
 * @brief init for competitions
 * 
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD. Runs the autonomous selector
 */
void competition_initialize(){
	pros::Task selector(autonSelector);
}

/**
 * @brief run autons
 * 
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous(){
	// erase screen for autonomous start
	pros::screen::erase();
	do_lift_PID = true;

	// run the selected autonomous routine
	switch(autonomousSelection){
		case 0: skip(); break;
		case 1: left_repeater_route(); break;
		case 2: right_repeater_route(); break;
		case 9: skills(); break;
		default: left_repeater_route();
	}
}

/**
 * @brief driver control
 * 
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol(){
	bool manual_lifting = false, was_claw_closed = true;
	pros::Task inputs(get_inputs);
	do_lift_PID = false;

	while (true) {
		// tank drive
		tank();

		// run the lift
		if(L1 != L2){
			manual_lifting = true;
			lift_manual(L2);
		}else{
			if(manual_lifting){
				manual_lifting = false;
				target = get_lift_height();
			}
			if(!do_lift_PID){
				lift.move_velocity(0);
			}
		}

		// set claw
		bool claw_closed = !R2;
		if(claw_closed != was_claw_closed){
			set_claw(claw_closed);
		}

		// auto lift
		if(claw_closed && !was_claw_closed && target <= 0.25){
			do_lift_PID = true;
			lift_pos(2);
		}
		was_claw_closed = claw_closed;

		pros::delay(20); // 20ms downtime
	}
}