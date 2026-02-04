/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "umbc.h"

#include <cstdint>
#include "okapi\api\control\iterative\iterativePosPidController.hpp"

using namespace pros;
using namespace umbc;
using namespace std;
using namespace okapi;

#define MOTOR_BLUE_GEAR_MULTIPLIER      600
#define MOTOR_RED_GEAR_MULTIPLIER       50

#define INTAKE_MOTOR_L        12
#define INTAKE_MOTOR_R        -15   //I think one of them has to be reversed
#define INTAKE_MOTOR_SPEED    100

#define MOTOR_FRONT_L       2
#define MOTOR_FRONT_R       -6
#define MOTOR_BACK_L        11
#define MOTOR_BACK_R        -20

#define ARM_MOTOR_R         -21
#define ARM_MOTOR_L         5

#define REST_POSITION       0
#define GOAL_1_POSITION     -750
//#define GOAL_2_POSITION     -300
//#define GOAL_3_POSIITON     -800


#define KP                  3
#define KD                  0
#define KI                  0
#define KBIAS               0


ARM_STATE cur_state = ARM_STATE::REST;
DIGITAL_BUTTON_STATE prev_button_state_r1 = DIGITAL_BUTTON_STATE::BUTTON_RELEASED;
DIGITAL_BUTTON_STATE cur_button_state_r1 = DIGITAL_BUTTON_STATE::BUTTON_RELEASED;

DIGITAL_BUTTON_STATE prev_button_state_r2 = DIGITAL_BUTTON_STATE::BUTTON_RELEASED;
DIGITAL_BUTTON_STATE cur_button_state_r2 = DIGITAL_BUTTON_STATE::BUTTON_RELEASED;

void umbc::Robot::opcontrol() {
    
    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    IterativePosPIDController arm_controler = IterativePosPIDController({KP, KI, KD, KBIAS}, global_time);
    
    // initialize motors and sensors
    const vector<int8_t> arm_motors = {ARM_MOTOR_L, ARM_MOTOR_R};
    pros::Motor_Group arm_group = Motor_Group(arm_motors);

    const vector<int8_t> intake_motors = {INTAKE_MOTOR_L, INTAKE_MOTOR_R};
    pros::Motor_Group intake_group = Motor_Group(intake_motors);

    //arm_group.set_brake_modes(pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_BRAKE);

    pros::Motor m1 = Motor(MOTOR_FRONT_L);
    pros::Motor m2 = Motor(MOTOR_FRONT_R);
    pros::Motor m3 = Motor(MOTOR_BACK_L);
    pros::Motor m4 = Motor(MOTOR_BACK_R);

    pros::Motor m5 = Motor(ARM_MOTOR_R);
    pros::Motor m6 = Motor(ARM_MOTOR_L);

    //set current and prev arm state
    cur_button_state_r1 = prev_button_state_r1;
    cur_button_state_r2 = prev_button_state_r2;
    int state_selector = 0; //inc. or dec. variable the user changes to operate the arm state machine 
    

    while(1) {

        float analog_left_x = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_X);
        float analog_left_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        float analog_right_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        
        float S = 0;
        S = sqrt(pow(analog_left_x,2) + pow(analog_left_y,2)); //I think this would calculate distance...idk

        float T = 0;
        if(!(S == 0)){
            T = atan2(analog_left_y, analog_left_x);
        }

        float R = 0;
        R = analog_right_x;

        
        float P_1 = 0;
        P_1 = -(cos(T + (M_PI/4))/cos(M_PI/4));

        float P_2 = 0;
        P_2 = -(cos(T + (3*M_PI/4))/cos(M_PI/4));

        float P_lf = 0;
        float P_rf = 0;
        float P_lb = 0;
        float P_rb = 0;
        P_lf = P_2;
        P_rf = P_1;
        P_lb = P_1;
        P_rb = P_2;


        float M_lf = 0;
        float M_rf = 0;
        float M_lb = 0;
        float M_rb = 0;
        float largest_stick = 0;
        
        if(S != 0){
            largest_stick = abs((std::max(std::max(P_lf, P_rf), std::max(P_lb, P_rb))))/S;
        }

        if((largest_stick != 0)){
            M_lf = (P_lf/(largest_stick))*(1-abs(R))+R;
            M_rf = (P_rf/(largest_stick))*(1-abs(R))-R;
            M_lb = (P_lb/(largest_stick))*(1-abs(R))+R;
            M_rb = (P_rb/(largest_stick))*(1-abs(R))-R;
        }

        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_R1)) //move arm up
        {
            cur_button_state_r1 = DIGITAL_BUTTON_STATE::BUTTON_PRESSED;
        }else{
            cur_button_state_r1 = DIGITAL_BUTTON_STATE::BUTTON_RELEASED;
        }
        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_R2)) //move arm down
        {
            cur_button_state_r2 = DIGITAL_BUTTON_STATE::BUTTON_PRESSED;
        }else{
            cur_button_state_r2 = DIGITAL_BUTTON_STATE::BUTTON_RELEASED;
        }

        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_L1)) //intake game objects
        {  
            intake_group.move_velocity(INTAKE_MOTOR_SPEED);
        }else{
            intake_group.move_velocity(0);
        }

        if(controller_master->get_digital(E_CONTROLLER_DIGITAL_L2)) //output game objects
        {  
            intake_group.move_velocity(-INTAKE_MOTOR_SPEED);
        }else{
            intake_group.move_velocity(0);
        }

        if((cur_button_state_r1 == DIGITAL_BUTTON_STATE::BUTTON_PRESSED) && (prev_button_state_r1 == DIGITAL_BUTTON_STATE::BUTTON_RELEASED)){
            state_selector++;
        }
        if((cur_button_state_r2 == DIGITAL_BUTTON_STATE::BUTTON_PRESSED) && (prev_button_state_r2 == DIGITAL_BUTTON_STATE::BUTTON_RELEASED)){
            state_selector--;
        }

        if(state_selector > 1){
            state_selector = 1;
        }else if (state_selector < 0){
            state_selector = 0;
        }
        cur_state = (ARM_STATE)state_selector;

        switch (cur_state){
            case REST:
                arm_controler.setTarget(REST_POSITION);
                break;
            
            case GOAL_1:
                arm_controler.setTarget(GOAL_1_POSITION);
                break;
        }
                    
        arm_group.move_absolute(arm_controler.getTarget(), -MOTOR_RED_GEAR_MULTIPLIER * arm_controler.getOutput());

        m1.move_velocity((M_lf+R) * MOTOR_BLUE_GEAR_MULTIPLIER * .01);
        m2.move_velocity((M_rf-R) * MOTOR_BLUE_GEAR_MULTIPLIER * .01);
        m3.move_velocity((M_lb+R) * MOTOR_BLUE_GEAR_MULTIPLIER * .01);
        m4.move_velocity((M_rb-R) * MOTOR_BLUE_GEAR_MULTIPLIER * .01);
        
        pros::lcd::set_text(1, (std::to_string(m5.get_position()))); //right motor
        pros::lcd::set_text(2, (std::to_string(m6.get_position()))); //left motor

        arm_controler.step((arm_group.get_positions()[0] + arm_group.get_positions()[1])/2);
        
        prev_button_state_r1 = cur_button_state_r1;
        prev_button_state_r2 = cur_button_state_r2;

        pros::lcd::set_text(3, (std::to_string(arm_controler.getTarget())));
        pros::lcd::set_text(4, (std::to_string(arm_controler.getOutput())));

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}