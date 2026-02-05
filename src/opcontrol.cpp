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
 
 
//left and right are relative to the robot's left and right
 
//DOUBLE CHECK MOTORS!!!!!
  
//ports for left motors(blue)
#define LEFT_MOTOR_FRONT    2
#define LEFT_MOTOR_BACK    11
       
  
//ports for right motors(blue)
#define RIGHT_MOTOR_FRONT   -6
#define RIGHT_MOTOR_BACK    -20

//ports for lift motors
#define ARM_MOTOR_RIGHT         -21
#define ARM_MOTOR_LEFT         5

//ports for intake motors
#define INTAKE_MOTOR_LEFT        12
#define INTAKE_MOTOR_RIGHT        -15

#define REST_POSITION       0       //low goal
#define MID_GOAL_POSITION   -100    //mid goal
#define GOAL_2_POSITION     -300    //ask about these two
#define GOAL_3_POSIITON     -800
#define TARGET_ERROR        10

#define KP                  3
#define KD                  0
#define KI                  0
#define KBIAS               0

/*
#define gyro_port1         'A' //port 1 for gyro
#define gyro_port2         'B' //port 2 for gyro
#define gyro_port3         'C' //port 3 for gyro
*/

void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    //initialize PID controller for arm
    okapi::ConfigurableTimeUtilFactory global_time_factory;
    okapi::TimeUtil global_time = global_time_factory.create();
    okapi::IterativePosPIDController arm_controller(KP, KI, KD, KBIAS, global_time);
    
    //initialize motors and sensors
    std::vector<int8_t> drive_motors{LEFT_MOTOR_FRONT, LEFT_MOTOR_BACK, RIGHT_MOTOR_FRONT, RIGHT_MOTOR_BACK};
    std::vector<int8_t> intake_motors{INTAKE_MOTOR_LEFT, INTAKE_MOTOR_RIGHT};
    std::vector<int8_t> arm_motors{ARM_MOTOR_LEFT, ARM_MOTOR_RIGHT};
    pros::MotorGroup driveGroup (drive_motors);
    pros::MotorGroup intakeGroup (intake_motors);
    pros::MotorGroup armGroup (arm_motors);

    //brakes and gearing
    driveGroup.set_brake_modes(E_MOTOR_BRAKE_COAST);
    driveGroup.set_gearing(E_MOTOR_GEAR_BLUE);
    
    //SET BRAKE AND GEAR MODES FOR INTAKE AND LIFT

    
    //drive motors
    pros::Motor left_motor_front (LEFT_MOTOR_FRONT);
    pros::Motor left_motor_back (LEFT_MOTOR_BACK);
        
    pros::Motor right_motor_front (RIGHT_MOTOR_FRONT);
    pros::Motor right_motor_back (RIGHT_MOTOR_BACK);

    //intake motors
    pros::Motor intake_motor_left (INTAKE_MOTOR_LEFT);
    pros::Motor intake_motor_right (INTAKE_MOTOR_RIGHT);

    //motor states
    bool slowSpeedButton = false;

    enum class INTAKE_STATE {INTAKE_OFF, INTAKE_ON, INTAKE_REVERSE};
    INTAKE_STATE intakeState = INTAKE_STATE::INTAKE_OFF;
    
    enum class ARM_STATE {REST, MID_GOAL};  //implement HIGH_GOAL if the bot can reach it
    ARM_STATE cur_state = ARM_STATE::REST;


    enum class DIGITAL_BUTTON_STATE {RELEASED, PRESSED};
    DIGITAL_BUTTON_STATE cur_r1_state = DIGITAL_BUTTON_STATE::RELEASED, prev_r1_state = DIGITAL_BUTTON_STATE::RELEASED;
    DIGITAL_BUTTON_STATE cur_r2_state = DIGITAL_BUTTON_STATE::RELEASED, prev_r2_state = DIGITAL_BUTTON_STATE::RELEASED;
    int state_selector = 0;

    /*******************************************************
    //GYRO STUFF
    pros::ADIAnalogIn gyro_sensor1 (gyro_port1);
    pros::ADIAnalogIn gyro_sensor2 (gyro_port2);
    pros::ADIAnalogIn gyro_sensor3 (gyro_port3);

    gyro_sensor1.calibrate();
    gyro_sensor2.calibrate();
    gyro_sensor3.calibrate();

    double curr_angle = 0;
    ********************************************************/

    while(1) {
        //GRYO STUFF
        /***********************************************************************************************************
        double gyro_val1 = gyro_sensor1.get_value_calibrated_HR(); //using HR cause this needs to be integrated
        double gyro_val2 = gyro_sensor2.get_value_calibrated_HR();
        double gyro_val3 = gyro_sensor3.get_value_calibrated_HR();

        double gyro_avg = (gyro_val1 + gyro_val2 + gyro_val3) / 3;
        
        //avg rot velocity in degrees per second
        double gyro_deg = gyro_avg / 19.5;    
        double test = gyro_val1 / 6.5;      
        
        //TUNE THIS DEADZONE
        if(fabs(gyro_deg) < 0.5){
            gyro_deg = 0;
        }
        

        //curr_angle += gyro_deg * (this->opcontrol_delay_ms);
        curr_angle += test * (this->opcontrol_delay_ms / 1000.0);

        if(curr_angle > 360){
            curr_angle -= 360;
        }
        else if(curr_angle < 0){
            curr_angle += 360;
        }

        double curr_rad = curr_angle * (M_PI / 180);

        pros::lcd::set_text(5, "curr angle: " + to_string(gyro_sensor1.get_value()));
        **************************************************************************************************************/
        //END GYRO STUFF


        //left joystick (target movement)
        double left_x = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_X);
        double left_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        left_x = pow(left_x, 3) / (127 * 127 * 127); //cubing for finer control
        left_y = pow(left_y, 3) / (127 * 127 * 127); //cubing for finer control

        //right joystick (rotation)
        double right_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        right_x = pow(right_x, 3) / (127 * 127 * 127); //cubing for finer control
        
        //converting to polar
        double radius = sqrt(left_x * left_x + left_y * left_y);
        double theta = atan2(left_y, left_x);

        //FOR FIELD RELATIVE
        //theta -= curr_rad; 

        //front right and back left motors
        //double power_one = -cos(theta + (M_PI/4)) / cos(M_PI/4);

        //front left and back right motors
        //double power_two = sin(theta + (M_PI)/4) / cos(M_PI/4);

        //this will determine the speed of each motor
        double power_front_left = sin(theta + (M_PI)/4) / cos(M_PI/4);
        double power_front_right = -cos(theta + (M_PI/4)) / cos(M_PI/4);
        double power_back_left = -cos(theta + (M_PI/4)) / cos(M_PI/4);
        double power_back_right = sin(theta + (M_PI)/4) / cos(M_PI/4); 
        
        double speed = 0;
        if(radius != 0){
            speed = max({fabs(power_back_left), fabs(power_back_right), fabs(power_front_left), fabs(power_front_right)}) / radius;
            //speed2 = max(fabs(power_front_left), fabs(power_front_right));
            //speed = max(speed1, speed2) / radius;
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
        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_A)){ //toggle slow speed
            slowSpeedButton = !slowSpeedButton;
        }

        //INTAKE CONTROLS
        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_L1)){ //toggle intake on
            if(intakeState == INTAKE_STATE::INTAKE_ON){
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else{
                intakeState = INTAKE_STATE::INTAKE_ON;
            }
        }
        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_L2)){ //toggle intake reverse
            if(intakeState == INTAKE_STATE::INTAKE_REVERSE){
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else{
                intakeState = INTAKE_STATE::INTAKE_REVERSE;
            }
        }

        //LIFT CONTROLS
        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_R1)){ //move arm up
            cur_r1_state = DIGITAL_BUTTON_STATE::PRESSED;
        }else{
            cur_r1_state = DIGITAL_BUTTON_STATE::RELEASED;
        }

        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_R2)){ //move arm down
            cur_r2_state = DIGITAL_BUTTON_STATE::PRESSED;
        }else{
            cur_r2_state = DIGITAL_BUTTON_STATE::RELEASED;
        }

        if((cur_r1_state == DIGITAL_BUTTON_STATE::PRESSED) && (prev_r1_state == DIGITAL_BUTTON_STATE::RELEASED)){
            state_selector++;
        }
         if((cur_r2_state == DIGITAL_BUTTON_STATE::PRESSED) && (prev_r2_state == DIGITAL_BUTTON_STATE::RELEASED)){
            state_selector--;
        }

        if(state_selector > 1){
            state_selector = 1;
        }else if (state_selector < 0){
            state_selector = 0;
        }
        cur_state = (ARM_STATE)state_selector;

        switch (cur_state){
            case ARM_STATE::REST:
                arm_controller.setTarget(REST_POSITION);
                break;
            
            case ARM_STATE::MID_GOAL:
                arm_controller.setTarget(MID_GOAL_POSITION);
                break;
        }

        //MOVING MOTORS

        //slow speed if toggled on
        if(slowSpeedButton){
            vel_fl *= 0.5;
            vel_fr *= 0.5;
            vel_bl *= 0.5;
            vel_br *= 0.5;
        }

        //intake motors
        //TEST FIRST DONT BREAK THE MOTORS PLEASE PLEASEEEE
        switch(intakeState){
            case INTAKE_STATE::INTAKE_OFF:
                intake_motor_left.move_velocity(0);
                intake_motor_right.move_velocity(0);
                break;
            case INTAKE_STATE::INTAKE_ON:
                intake_motor_left.move_velocity(200);
                intake_motor_right.move_velocity(200);
                break;
            case INTAKE_STATE::INTAKE_REVERSE:
                intake_motor_left.move_velocity(-200);
                intake_motor_right.move_velocity(-200);
                break;
        }

        //drive motors
        left_motor_front.move_velocity((vel_fl + right_x)*MOTOR_BLUE_GEAR_MULTIPLIER);
        left_motor_back.move_velocity((vel_bl + right_x)*MOTOR_BLUE_GEAR_MULTIPLIER);
        right_motor_front.move_velocity((vel_fr - right_x)*MOTOR_BLUE_GEAR_MULTIPLIER);
        right_motor_back.move_velocity((vel_br - right_x)*MOTOR_BLUE_GEAR_MULTIPLIER);

        //error system to prevent arm motors from moving while within a close enough range of the target
        if(abs((arm_controller.getTarget() - ((armGroup.get_positions()[0] + armGroup.get_positions()[1])/2))) > TARGET_ERROR){
            armGroup.move_absolute(arm_controller.getTarget(), -MOTOR_RED_GEAR_MULTIPLIER * arm_controller.getOutput());
        }else{
            armGroup.move_velocity(0);
        }
        //feeds back process varaible (average of motor positions)
        arm_controller.step((armGroup.get_positions()[0] + armGroup.get_positions()[1])/2);

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}