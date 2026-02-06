/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"
#include "pros/adi.hpp"
#include "umbc.h"
#include "umbc/robot.hpp"
#include "okapi\api\control\iterative\iterativePosPidController.hpp"
#include "okapi\impl\util\configurableTimeUtilFactory.hpp"
#include <cstdint>
#include <time.h>
#include <vector>
#include <cmath>

using namespace pros;
using namespace umbc;
using namespace std;

  
#define MOTOR_RED_GEAR_MULTIPLIER       100
#define MOTOR_GREEN_GEAR_MULTIPLIER     200
#define MOTOR_BLUE_GEAR_MULTIPLIER      600
#define MOTOR_REVERSE                   true
#define REVERSED(port)                  -port

#define INTAKE_MOTOR_SPEED              100
#define TARGET_ERROR                    10
 
 
//left and right are relative to the robot's left and right
 
//DOUBLE CHECK MOTORS!!!!!
  
//ports for left motors(blue)
#define LEFT_MOTOR_FRONT        2
#define LEFT_MOTOR_BACK         11
       
  
//ports for right motors(blue)
#define RIGHT_MOTOR_FRONT      -6
#define RIGHT_MOTOR_BACK       -20

//ports for lift motors
#define ARM_MOTOR_RIGHT        -21
#define ARM_MOTOR_LEFT          5

//ports for intake motors
#define INTAKE_MOTOR_LEFT       12
#define INTAKE_MOTOR_RIGHT     -15
#define REST_POSITION          -65     //low goal
#define MID_GOAL_POSITION      -850    //mid goal


#define KP                   3
#define KD                   0
#define KI                   0
#define KBIAS                0


void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    //initialize PID controller for arm
    okapi::ConfigurableTimeUtilFactory global_time_factory;
    okapi::TimeUtil global_time = global_time_factory.create();
    okapi::IterativePosPIDController arm_controller(KP, KI, KD, KBIAS, global_time);
    
    //initialize motors
    std::vector<int8_t> drive_motors{LEFT_MOTOR_FRONT, LEFT_MOTOR_BACK, RIGHT_MOTOR_FRONT, RIGHT_MOTOR_BACK};
    std::vector<int8_t> intake_motors{INTAKE_MOTOR_LEFT, INTAKE_MOTOR_RIGHT};
    std::vector<int8_t> arm_motors{ARM_MOTOR_RIGHT, ARM_MOTOR_LEFT};
    pros::MotorGroup driveGroup (drive_motors);
    pros::MotorGroup intakeGroup (intake_motors);
    pros::MotorGroup armGroup (arm_motors);

    //brakes and gearing
    //DRIVE
    driveGroup.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
    driveGroup.set_gearing(E_MOTOR_GEAR_GREEN);

    //INTAKE
    intakeGroup.set_brake_modes(E_MOTOR_BRAKE_COAST);
    intakeGroup.set_gearing(E_MOTOR_GEAR_BLUE);

    //ARM
    armGroup.set_brake_modes(E_MOTOR_BRAKE_BRAKE);
    armGroup.set_gearing(E_MOTOR_GEAR_RED);

    
    //drive motors
    pros::Motor left_motor_front (LEFT_MOTOR_FRONT);
    pros::Motor left_motor_back (LEFT_MOTOR_BACK);
        
    pros::Motor right_motor_front (RIGHT_MOTOR_FRONT);
    pros::Motor right_motor_back (RIGHT_MOTOR_BACK);

    //intake motors
    pros::Motor intake_motor_left (INTAKE_MOTOR_LEFT);
    pros::Motor intake_motor_right (INTAKE_MOTOR_RIGHT);

    //arm motors
    pros::Motor arm_motor_right (ARM_MOTOR_RIGHT);
    pros::Motor arm_motor_left (ARM_MOTOR_LEFT);

    //motor states
    bool slowSpeedButton = false;

    enum class INTAKE_STATE {INTAKE_OFF, INTAKE_ON, INTAKE_REVERSE};
    INTAKE_STATE intakeState = INTAKE_STATE::INTAKE_OFF;
    
    enum class ARM_STATE {REST, MID_GOAL};  //implement HIGH_GOAL if the bot can reach it
    ARM_STATE cur_state = ARM_STATE::REST;
    int arm_state_selector = 0;

    enum class DRIVE_STATE {SLOW_DRIVE, DEFAULT_DRIVE, FAST_DRIVE};
    DRIVE_STATE driveState = DRIVE_STATE::DEFAULT_DRIVE;
    int speed_state_selector = 1;

    bool score_slow = false;

    while(1) {
        //left joystick (target movement)
        double left_x = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_X);
        double left_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        //controller deadzone
        if(left_x > -15 && left_x < 15){
            left_x = 0;
        }
        if(left_y > -15 && left_y < 15){
            left_y = 0;
        }
        left_x = pow(left_x, 3) / (127 * 127 * 127); //cubing for finer control
        left_y = pow(left_y, 3) / (127 * 127 * 127); //cubing for finer control

        //right joystick (rotation)
        double right_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        right_x = pow(right_x, 3) / (127 * 127 * 127) *0.75; //cubing for finer control
        
        //converting to polar
        double radius = sqrt(left_x * left_x + left_y * left_y);
        double theta  = atan2(left_y, left_x);

        //this will determine the speed of each motor
        double power_front_left  = sin(theta + (M_PI)/4) / cos(M_PI/4);
        double power_front_right = -cos(theta + (M_PI/4)) / cos(M_PI/4);
        double power_back_left   = -cos(theta + (M_PI/4)) / cos(M_PI/4);
        double power_back_right  = sin(theta + (M_PI)/4) / cos(M_PI/4); 
        
        double speed = 0;
        if(radius != 0){
            speed = max({fabs(power_back_left), fabs(power_back_right), fabs(power_front_left), fabs(power_front_right)}) / radius;
        }

        double vel_fl = 0;
        double vel_fr = 0;
        double vel_bl = 0;
        double vel_br = 0;
        if(speed != 0){
            vel_fl = power_front_left;
            vel_fr = power_front_right;
            vel_bl = power_back_left;
            vel_br = power_back_right;
        }
        
        //SPEED CONTROL
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){ //toggle slow speed
            speed_state_selector++;
        }
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){ //toggle slow speed
            speed_state_selector--;
        }

        if(speed_state_selector > 2){
            speed_state_selector = 2;
        }else if (speed_state_selector < 0){
            speed_state_selector = 0;
        }
        driveState = (DRIVE_STATE)speed_state_selector;
        
        //INTAKE CONTROLS
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){ //toggle intake on
            if(intakeState == INTAKE_STATE::INTAKE_ON){
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else{
                intakeState = INTAKE_STATE::INTAKE_ON;
            }
        }
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){ //toggle intake reverse
            if(intakeState == INTAKE_STATE::INTAKE_REVERSE){
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else{
                intakeState = INTAKE_STATE::INTAKE_REVERSE;
            }
        }
    
        //LIFT CONTROLS
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){ //move arm up
            arm_state_selector++;
        }

        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){ //move arm down
            arm_state_selector--;
        }

        if(arm_state_selector > 1){
            arm_state_selector = 1;
        }else if (arm_state_selector < 0){
            arm_state_selector = 0;
        }
        cur_state = (ARM_STATE)arm_state_selector;

        switch (cur_state){
            case ARM_STATE::REST:
                arm_controller.setTarget(REST_POSITION);
                score_slow = false;
                break;
            
            case ARM_STATE::MID_GOAL:
                score_slow = true;
                arm_controller.setTarget(MID_GOAL_POSITION);
                break;
        }
        /*
        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_RIGHT)){ //override arm control
            cur_state = ARM_STATE::OVERRIDE_UP;
        }
        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_LEFT)){ //override arm control
            cur_state = ARM_STATE::OVERRIDE_DOWN;
        }
        */

        //MOVING MOTORS
        //slow speed if toggled on

        switch(driveState){
            case DRIVE_STATE::SLOW_DRIVE:
                vel_fl *= 0.5;
                vel_fr *= 0.5;
                vel_bl *= 0.5;
                vel_br *= 0.5;
                break;
            case DRIVE_STATE::DEFAULT_DRIVE:
                break;
            case DRIVE_STATE::FAST_DRIVE:
                vel_fl *= 1.25;
                vel_fr *= 1.25;
                vel_bl *= 1.25;
                vel_br *= 1.25;
                break;
        }

        //intake motors
        switch(intakeState){
            case INTAKE_STATE::INTAKE_OFF:
                intake_motor_left.move_velocity(0);
                intake_motor_right.move_velocity(0);
                break;
            case INTAKE_STATE::INTAKE_ON:
                intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * -0.85);
                intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * -0.85);
                break;
            case INTAKE_STATE::INTAKE_REVERSE:
                if(score_slow){
                    intake_motor_left.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER * -0.2);
                    intake_motor_right.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER * -0.2);
                }
                else{
                    intake_motor_left.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER * -0.35);
                    intake_motor_right.move_velocity(-MOTOR_BLUE_GEAR_MULTIPLIER * -0.35);
                }
                break;
        }
        
        //drive motors
        left_motor_front.move_velocity((vel_fl + right_x)*MOTOR_GREEN_GEAR_MULTIPLIER*0.7);
        left_motor_back.move_velocity((vel_bl + right_x)*MOTOR_GREEN_GEAR_MULTIPLIER*0.7);
        right_motor_front.move_velocity((vel_fr - right_x)*MOTOR_GREEN_GEAR_MULTIPLIER*0.7);
        right_motor_back.move_velocity((vel_br - right_x)*MOTOR_GREEN_GEAR_MULTIPLIER*0.7);

        //lift motors
        armGroup.move_absolute(arm_controller.getTarget(), -MOTOR_RED_GEAR_MULTIPLIER * arm_controller.getOutput()*0.35);

        //moving both individually rather than as a group
        //arm_motor_right.move_absolute(arm_controller.getTarget(), -MOTOR_RED_GEAR_MULTIPLIER * arm_controller.getOutput()*0.35);
        //arm_motor_left.move_absolute(arm_controller.getTarget(), -MOTOR_RED_GEAR_MULTIPLIER * arm_controller.getOutput()*0.35);
        arm_controller.step((armGroup.get_positions()[0] + armGroup.get_positions()[1])/2);
        

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}