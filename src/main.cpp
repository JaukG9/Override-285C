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
void initialize(){}

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
void competition_initialize(){}

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
void autonomous(){}

void opcontrol() {
	/**
	 * Without using any external functions from the other files,
	 * try to implement a simple opcontrol program that lets the
	 * driver do the following
	 * 	 a) make it run tank drive
	 * 	 b) make it so that if you press the A button, the lift goes up
	 *		and goes down if you press the B button
	 *	 c) make it so that if you press R2, then it toggles the value
	 *		of a piston --> it will close if you press it then open if
	 *		you press it again, etc BUT NOT IF YOU'RE HOLDING IT
	 */
}
