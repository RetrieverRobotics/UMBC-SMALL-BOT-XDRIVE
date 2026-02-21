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
#define TARGET_ERROR                    5
#define dt                              10
 
/*EXPERIMENTAL INTAKE MOTOR CONTROLLER TO READJUST MOTOR SPEED IN CASE OF STALLING*/
class IntakeMotorController{
    private:
        vector<double> motor_position_history = {};
        MotorGroup *intake_group;

        double current_threshold = 0;
        double min_position_change = 0;

        bool is_intake_stalling(){
            //motor is stalling if the differnce between the most recent position and last position is below a certain number
            if(((motor_position_history[motor_position_history.size()-2] - motor_position_history[motor_position_history.size()-1]) < min_position_change)
            && (((intake_group->get_current_draws()[0] + intake_group->get_current_draws()[1])/2) > current_threshold)){
                return true;
            }
            return false;
        }
        
        void getAdjustedOutputVoltage(double &motor_output_l, double &motor_output_r){
            //not mathematically sound, fix this later
            motor_output_l = MOTOR_BLUE_GEAR_MULTIPLIER * (((intake_group->get_current_draws()[0] + intake_group->get_current_draws()[1])/2) - current_threshold);
            motor_output_r = MOTOR_BLUE_GEAR_MULTIPLIER * (((intake_group->get_current_draws()[0] + intake_group->get_current_draws()[1])/2) - current_threshold);
        }
    
    public:
        IntakeMotorController(double currentThreshold, double minPositionChange){
            current_threshold = current_threshold;
            min_position_change = minPositionChange;
        }

        void updateIntakeController(MotorGroup &intakegroup){
            intake_group = &intakegroup;
            motor_position_history.push_back((intake_group->get_positions()[0]+intake_group->get_positions()[1])/2);
        }
};


class PIDController {
    private:
        double kp;
        double ki;
        double kd;
        double kBias;

        double error = 0;
        double prev_error = 0;
        double changeError = 0;
        double totalError = 0;

    public:
        PIDController(double p, double i, double d, double bias){
            kp = p;
            ki = i;
            kd = d;
            kBias = bias;
        }

        void reset(){
            error = 0;
            prev_error = 0;
            changeError = 0;
            totalError = 0;
        }

        double calculatePID(double target, double current){
            error = target - current;
            changeError = (error - prev_error)/dt;
            totalError += error * dt;
            prev_error = error;

            //capping totalError to prevent possible overshooting
            if(totalError > 500) totalError = 500;
            if(totalError < -500) totalError = -500;

            double pidCalc = kp * error + ki * totalError + kd * changeError + kBias;
            double pidCalc_scaled = pidCalc * 12000 / 100;
            if(pidCalc_scaled > 12000) pidCalc_scaled = 12000;
            if(pidCalc_scaled < -12000) pidCalc_scaled = -12000;

            return pidCalc_scaled;
        }
}; 


//left and right are relative to the robot's left and right
 
//DOUBLE CHECK MOTORS!!!!!
  
//ports for left drive motors (green)
#define LEFT_MOTOR_FRONT        2
#define LEFT_MOTOR_BACK         11
       
  
//ports for right drive motors (green)
#define RIGHT_MOTOR_FRONT      -7
#define RIGHT_MOTOR_BACK       -20

//ports for lift motors (red)
#define ARM_MOTOR_RIGHT        -21
#define ARM_MOTOR_LEFT          5

//ports for intake motors (blue)
#define INTAKE_MOTOR_LEFT                   12
#define INTAKE_MOTOR_RIGHT                  -15

#define INTAKE_SINGLE_OUT_TIMMER       320

#define RESET_BUTTON           1
#define REST_POSITION           20    //low goal
#define MID_GOAL_POSITION      -915    //mid goal


#define KP                   0.1
#define KD                   0.65
#define KI                   0.01
#define KBIAS                -15        //for gravity
double leftArmMotorZero = 0;
double rightArmMotorZero = 0;
double armTarget = 0;

void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    //initialize PID controller for arm
    PIDController leftArmPID(KP, KI, KD, KBIAS);
    PIDController rightArmPID(KP, KI, KD, KBIAS);
    
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
    armGroup.set_brake_modes(E_MOTOR_BRAKE_HOLD);
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
    leftArmMotorZero = arm_motor_left.get_position();
    rightArmMotorZero = arm_motor_right.get_position();

    //motor states
    bool slowSpeedButton = false;

    enum class INTAKE_STATE{
        INTAKE_OFF,
        INTAKE_ON,
        INTAKE_REVERSE
    };
    INTAKE_STATE intakeState = INTAKE_STATE::INTAKE_OFF;
    double time = 0;
    double timmer_limit = INTAKE_SINGLE_OUT_TIMMER;
    double intake_position_hold_r = 0;
    double intake_position_hold_l = 0;

    bool timed_outake = false;
    bool allow_timed_outake = false;
    bool break_timmer = false;
    
    enum class ARM_STATE{ //implement HIGH_GOAL if the bot can reach it
        REST,
        MID_GOAL,
        OVERRIDE
    };
    ARM_STATE arm_state = ARM_STATE::REST;
    int arm_state_selector = 0;

    enum class DRIVE_STATE{
        SLOW_DRIVE,
        DEFAULT_DRIVE,
        FAST_DRIVE
    };
    DRIVE_STATE driveState = DRIVE_STATE::DEFAULT_DRIVE;
    int speed_state_selector = 1;

    enum class SCORE_SPEED{SLOW, DEFAULT, FAST};
    int score_speed_selector = 1;

    //Limit switch
    ADIDigitalIn reset_button (RESET_BUTTON);
    
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
        left_y = pow(left_y, 3) / (127 * 127 * 127);

        //right joystick (rotation)
        double right_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        right_x = pow(right_x, 3) / (127 * 127 * 127) *0.75; //cubing for finer control
        
        //converting to polar
        double radius = sqrt(left_x * left_x + left_y * left_y);
        double theta  = atan2(left_y, left_x);

        //this will determine the speed of each motor
        double power_front_left = sin(theta + (M_PI)/4) / cos(M_PI/4);
        double power_front_right = -cos(theta + (M_PI/4)) / cos(M_PI/4);
        double power_back_left = -cos(theta + (M_PI/4)) / cos(M_PI/4);
        double power_back_right = sin(theta + (M_PI)/4) / cos(M_PI/4); 
        
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
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){ //toggle intake on
            if(intakeState == INTAKE_STATE::INTAKE_ON){
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else{
                intakeState = INTAKE_STATE::INTAKE_ON;
            }
        }
        
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_X)){
            time = pros::millis(); //reset time so measure with
            allow_timed_outake = true; //latch to prevent outake from working on start up
        }
        timed_outake = ((pros::millis() - time) < timmer_limit);
        break_timmer = ((pros::millis() - time) < timmer_limit + 50);
        

        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R2))
        {
            if(intakeState == INTAKE_STATE::INTAKE_REVERSE){
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else{
                intakeState = INTAKE_STATE::INTAKE_REVERSE;
            }
        }

        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){ //changes score speed, mainly for skills
            score_speed_selector++;
        }
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)){ //changes score speed, mainly for skills
            score_speed_selector--;
        }

        if(score_speed_selector > 2){
            score_speed_selector = 2;
        }else if (score_speed_selector < 0){
            score_speed_selector = 0;
        }

        SCORE_SPEED score_speed = (SCORE_SPEED)score_speed_selector;

        //RETURN TO DEFAULT
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
            score_speed_selector = (int)SCORE_SPEED::DEFAULT;
            speed_state_selector = (int)DRIVE_STATE::DEFAULT_DRIVE;
            arm_state_selector = (int)ARM_STATE::REST;
        }
    
        //LIFT CONTROLS
        //override switch
        if(reset_button.get_new_press()){
            arm_motor_left.tare_position();
            arm_motor_right.tare_position();
            leftArmMotorZero = arm_motor_left.get_position();
            rightArmMotorZero = arm_motor_right.get_position();
            arm_state_selector = (int)ARM_STATE::REST;
            arm_state = ARM_STATE::REST;
        }

        if(arm_state != ARM_STATE::OVERRIDE){
            //move arm up
            if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
                arm_state_selector++;
            }

            //move arm down
            if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
                arm_state_selector--;
            }

            if(arm_state_selector > 1){
                arm_state_selector = 1;
            }else if (arm_state_selector < 0){
                arm_state_selector = 0;
            }
            arm_state = (ARM_STATE)arm_state_selector;
        }

            
        switch (arm_state){
            case ARM_STATE::REST:
                armTarget = REST_POSITION;
                break;
                
            case ARM_STATE::MID_GOAL:
                armTarget = MID_GOAL_POSITION;
                break;
            default:
                break;
        }
        
                

        //MOVING MOTORS
        //drive speed states
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
                if(score_speed == SCORE_SPEED::SLOW){
                    intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.1);
                    intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.1);
                }
                else if(score_speed == SCORE_SPEED::DEFAULT){
                    intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.2);
                    intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.2);
                }
                else{
                    intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.85);
                    intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.85);
                }
                break;
        }
        
        //drive motors
        left_motor_front.move_velocity((vel_fl + right_x)*MOTOR_GREEN_GEAR_MULTIPLIER*0.7);
        left_motor_back.move_velocity((vel_bl + right_x)*MOTOR_GREEN_GEAR_MULTIPLIER*0.7);
        right_motor_front.move_velocity((vel_fr - right_x)*MOTOR_GREEN_GEAR_MULTIPLIER*0.7);
        right_motor_back.move_velocity((vel_br - right_x)*MOTOR_GREEN_GEAR_MULTIPLIER*0.7);

        //lift motors
        double leftArmVolt = 0;
        double rightArmVolt = 0;

        //if arms are not within target error && arms are not in override, calc PID
        if(!(fabs(arm_motor_left.get_position() - leftArmMotorZero - armTarget) < TARGET_ERROR &&
           fabs(arm_motor_right.get_position() - rightArmMotorZero - armTarget) < TARGET_ERROR) && arm_state != ARM_STATE::OVERRIDE){
            leftArmVolt = leftArmPID.calculatePID(armTarget, arm_motor_left.get_position() - leftArmMotorZero);
            rightArmVolt = rightArmPID.calculatePID(armTarget, arm_motor_right.get_position() - rightArmMotorZero);
        }
        else{
            leftArmPID.reset();
            rightArmPID.reset();
        }
        
        if(arm_state == ARM_STATE::OVERRIDE){
            if(controller_master->get_digital(E_CONTROLLER_DIGITAL_L1)){
                arm_motor_left.move_velocity(MOTOR_RED_GEAR_MULTIPLIER * -0.25);
                arm_motor_right.move_velocity(MOTOR_RED_GEAR_MULTIPLIER * -0.25);
            }
            else if(controller_master->get_digital(E_CONTROLLER_DIGITAL_L2)){
                arm_motor_left.move_velocity(MOTOR_RED_GEAR_MULTIPLIER * 0.25);
                arm_motor_right.move_velocity(MOTOR_RED_GEAR_MULTIPLIER * 0.25);
            }
            else{
                arm_motor_left.move_velocity(0);
                arm_motor_right.move_velocity(0);
            }
        }
        else{
            arm_motor_left.move_voltage(leftArmVolt);
            arm_motor_right.move_voltage(rightArmVolt);
        }
        
        if(timed_outake && allow_timed_outake){ //drives intake when timmer is active (only works here for some reason DON'T MOVE)
            intake_position_hold_l = intake_motor_left.get_position();
            intake_position_hold_r = intake_motor_right.get_position();
            intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.85);
            intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.85);
        }
        if(break_timmer && allow_timed_outake && !timed_outake){ //locks motors into last recorded position to shoot ball at the end
            intake_motor_left.move_absolute(intake_position_hold_l, 100);
            intake_motor_right.move_absolute(intake_position_hold_r, 100);
        }

        pros::lcd::set_text(2, std::to_string(timmer_limit));
        
        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}