#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/robot-config.h"
#include "robot/odometry.h"
#include "robot/functions.h"

bool right_arrow, L1, L2, R2, A = false;
void get_inputs(void*){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        right_arrow = true;
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        L1 = true;
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        L2 = true;
    }    
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        R2 = true;
    }
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        A = true;
    }
}

/**
 * @brief Translates controller input to drivetrain movements
 *
 * This function detects vertical movements on the joysticks of the controller, and
 * based on how much each joystick is moved up or down, supplies voltage to the drivetrain
 * motors of the corresponding side. The left side of the drive train is controlled by the
 * left joystick, and the right side of the drive train is controlled by the right joystick.
 * Down reverses the wheels, and up moves them forward. When the joysticks have unequal
 * magnitudes, the robot will turn in the direction of the one with the greater magnitude.
 * The vertical directional buttons on the controller can be used to move the drivetrain
 * at maximum speed in the forward (up arrow) or backwards (down arrow) direction.
 */
void tank(){
    int axisL = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);  // Gets amount forward/backward from left joystick
    int axisR = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); // Gets the turn left/right from right joystick
    
    // Checks if either of the vertical directional buttons are being pressed
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        axisL = 127 * .85;
        axisR = 127 * .85;
    }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        axisL = -127 * .85;
        axisR = -127 * .85;
    }

    chassis.tank(axisL, axisR); // tank drive
}

double get_rotation(){
    return rotation.get_position() / 100;
}

void lift_pos(double height){
    do_lift_PID = true;
    target = std::min(height, 0.0);
}

void lift_manual(bool up){
    do_lift_PID = false;
    if(up){
        lift.move(127);
    }else{
        lift.move(-127);
    }
}