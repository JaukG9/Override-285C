#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"
#include "robot/functions.h"
#include "robot/odometry.h"
#include "robot/autos.h"
#include "robot/auto-pid.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * Initializes the brain, chassis, and pneumatics.
 */
void initialize(){
	pros::lcd::initialize();
	// aivision_back.reset();
	// aivision_side.reset();
	chassis.calibrate();
	rotation.reset_position();

	claw.set_value(true);
	pros::Task run_lift(lift_PID);
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

	// tune the drive PIDs, angular first since lateral uses it (REMOVE BEFORE COMP)
	//auto_angular_PID();
	//auto_lateral_PID();

	// // LIFT PID TEST
	// do_lift_PID = true;
	// target = 0.08;
	// controller.print(0, 0, "%.3f, %.3f", target, rotation.get_angle());
	// pros::delay(60);

	do_lift_PID = false;
	repeater_route();

	// run the selected autonomous routine
	// switch(autonomousSelection){
	// 	case 0: skip(); break;
	// 	case 1: ; break;
	// 	case 2: ; break;
	// 	case 9: skills(); break;
	// 	default: break;
	// }
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
	// josticks --> driving
	// right arrow --> run the intake & move lift and claw to set height and open claw + a sensor to see and pick up object
	// L2 --> down, L1 --> up
	// R2 hold to open
	// A --> force override prevent tipping algo

	bool intaking = false, manual_lifting = false;
	pros::Task inputs(get_inputs);
	do_lift_PID = false;

	while (true) {
		// tank drive
		tank();

		// if(right_arrow){
		// 	lift_pos(20);
		// 	intaking = true;
		// }
		
		if(intaking && std::abs(get_lift_height() - 20) < 0.5){
			intaking = false;
		}

		if(L1 != L2 && !intaking){
			do_lift_PID = false;
			manual_lifting = true;
			lift_manual(L2); // L1 = up, L2 = down
		}else if(!intaking){
			// just let go of L1/L2: hold the lift where it stopped
			if(manual_lifting){
				manual_lifting = false;
				target = get_lift_height();
				do_lift_PID = true;
			}

			// only stop the lift here if the PID isn't holding it, otherwise they fight
			if(!do_lift_PID){
				lift.move_velocity(0);
			}
		}

		claw.set_value(!(R2 || intaking));
		claw_rotate.set_value(intaking);

		pros::delay(20); // 20ms downtime
	}
}