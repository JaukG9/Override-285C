#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"
#include "robot/functions.h"
#include "robot/odometry.h"
#include "robot/autos.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * Initializes the brain, chassis, and pneumatics.
 */
void initialize(){
	pros::lcd::initialize();
	chassis.calibrate();
}

/**
 * Runs while the robot is in the disabled state of the Field Management System
 * or the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled(){}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * Runs the autonomous selector
 */
void competition_initialize(){
	pros::Task selector(autonSelector);
}

/**
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

	// run the selected autonomous routine
	switch(autonomousSelection){
		case 0: ; break;
		case 1: ; break;
		case 2: ; break;
		default: break;
	}
}

/**
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
void opcontrol() {
	while (true) {
		// tank drive
		tank();

		partner.clear();
		partner.print(0, 0, "Battery: %d%%", 24);

		pros::delay(20); // 20ms downtime
	}
}