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
#include <cstdint>
#include <time.h>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace pros;
using namespace umbc;
using namespace std;

/* I swear I'm going to have a real stern talking to to whoever the fuck put 
magic numbers everywhere */
  
constexpr int MOTOR_RED_GEAR_MULTIPLIER    = 100;
constexpr int MOTOR_GREEN_GEAR_MULTIPLIER  = 200;
constexpr int MOTOR_BLUE_GEAR_MULTIPLIER   = 600;
constexpr bool MOTOR_REVERSE                = true;

constexpr int INTAKE_MOTOR_SPEED = 100;
constexpr int TARGET_ERROR = 5;
//constexpr int dt = 10; avoid hardcoded value for dt; this would only work in a perfect scenario
 
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

        double calculatePID(double target, double current, double time_step){
            error = target - current;
            changeError = (error - prev_error)/time_step;
            totalError += error * time_step;
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
constexpr int LEFT_MOTOR_FRONT = 2;
constexpr int LEFT_MOTOR_BACK = 11;
       
  
//ports for right drive motors (green)
constexpr int RIGHT_MOTOR_FRONT = -7;
constexpr int RIGHT_MOTOR_BACK  = -20;

//ports for lift motors (red)
constexpr int ARM_MOTOR_RIGHT = -21;
constexpr int ARM_MOTOR_LEFT = 5;

//ports for intake motors (blue)
constexpr int INTAKE_MOTOR_LEFT = 12;
constexpr int INTAKE_MOTOR_RIGHT  = -13;

constexpr int INTAKE_SINGLE_OUT_TIMER = 320;

constexpr int REST_POSITION  = 20;    //low goal
constexpr int MID_GOAL_POSITION = -915;   //mid goal


constexpr double KP = 0.1;
constexpr double KD = 0.65;
constexpr double KI = 0.01;
constexpr int KBIAS = -15;      //for gravity
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
    uint32_t time = 0; //previously was a double, however pros::millis() returns a 32 bit integer
    double timer_limit = INTAKE_SINGLE_OUT_TIMER;
    double intake_position_hold_r = 0;
    double intake_position_hold_l = 0;

    bool timed_outake = false;
    bool allow_timed_outake = false;
    bool break_timer = false;
    bool timed_outake_started = false; //Needs debugging where called 

    enum class ARM_STATE{ //implement HIGH_GOAL if the bot can reach it
        REST,
        MID_GOAL,
        OVERRIDE
    };
    ARM_STATE arm_state = ARM_STATE::REST;
    ARM_STATE prev_arm_state = arm_state; //Prev arm state for PID purposes
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


    constexpr double INV_127_CUBED = 1.0 / 2048383.0; //substitutes 127 * 127 * 127, whatever that means (magic number)
    const double PI_OVER_4 = M_PI/4;
    const double INV_COS_45 = 1.0 / cos(PI_OVER_4); //Save on repeated  calculations in the loop when determining motor speed

    while(1) {

        uint32_t loop_start = pros::millis();
        //left joystick (target movement)
        double raw_left_x = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_X);
        double raw_left_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

        //controller deadzone
        double left_x = (std::abs(raw_left_x) < 15) ? 0 : (raw_left_x * raw_left_x * raw_left_x) * INV_127_CUBED ;
        double left_y = (std::abs(raw_left_y) < 15) ? 0 : (raw_left_y * raw_left_y * raw_left_y) * INV_127_CUBED ;

        //right joystick (rotation)
        double raw_right_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        double right_x = (raw_right_x * raw_right_x * raw_right_x) * INV_127_CUBED * 0.75; //cubing for finer control; I believe 0.75 is a sensitivity cap
        
        //converting to polar
        //double radius = sqrt(left_x * left_x + left_y * left_y);
        //double theta  = atan2(left_y, left_x);

        //this will determine the speed of each motor
        //double power_front_left = sin(theta + PI_OVER_4) * INV_COS_45;
        //double power_front_right = -cos(theta + PI_OVER_4) * INV_COS_45;
        //double power_back_left = -cos(theta + PI_OVER_4) * INV_COS_45;
        //double power_back_right = sin(theta + PI_OVER_4) * INV_COS_45; 

        //Combine translation before normalization
        //double raw_fl = power_front_left * radius + right_x;
        //double raw_fr = power_front_right * radius - right_x;
        //double raw_bl = power_back_left * radius + right_x;
        //double raw_br = power_back_right * radius - right_x;
        
    
        //double vel_fl = raw_fl / scale;
        //double vel_fr = raw_fr / scale;
        //double vel_bl = raw_bl / scale;
        //double vel_br = raw_br / scale;

        //Believe it or not, the previous polar calculations just simplify to this
        double vel_fl =  left_y + left_x + right_x;
        double vel_fr = left_y - left_x - right_x;
        double vel_bl = left_y - left_x + right_x;
        double vel_br = left_y + left_x - right_x;

        //Normalization step; makes sure all motors move the same speed
        double maxPower = max({fabs(vel_fl), fabs(vel_fr), fabs(vel_bl), fabs(vel_br), 1.0});
        vel_fl /= maxPower;
        vel_fr /= maxPower;
        vel_bl /= maxPower;
        vel_br /= maxPower;

        
        //SPEED CONTROL
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){ //toggle slow speed
            speed_state_selector++;
        }
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){ //toggle slow speed
            speed_state_selector--;
        }

        //Caps the speed state to the 2nd speed (I believe) and makes sure the lowest state is 0
        speed_state_selector = std::clamp(speed_state_selector, 0, 2);

        driveState = (DRIVE_STATE)speed_state_selector;
        
        //INTAKE CONTROLS
        //Toggle forward intake
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){ //toggle intake
            if(intakeState == INTAKE_STATE::INTAKE_ON){
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else{
                intakeState = INTAKE_STATE::INTAKE_ON;
            }

        }

        // Toggle reverse Intake
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R2))
        {
            if(intakeState == INTAKE_STATE::INTAKE_REVERSE){
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else{
                intakeState = INTAKE_STATE::INTAKE_REVERSE;
            }
        }
        
        uint32_t now = pros::millis();
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_X)){
            time = now; //reset time so measure with
            allow_timed_outake = true; //latch to prevent outake from working on start up
            timed_outake_started = false;
        }
        timed_outake = ((now - time) < timer_limit);
        break_timer = ((now - time) < timer_limit + 50);
        

        

        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){ //changes score speed, mainly for skills
            score_speed_selector++;
        }
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)){ //changes score speed, mainly for skills
            score_speed_selector--;
        }

        score_speed_selector = std::clamp(score_speed_selector, 0, 2);
        SCORE_SPEED score_speed = (SCORE_SPEED)score_speed_selector;

        //RETURN TO DEFAULT
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
            score_speed_selector = (int)SCORE_SPEED::DEFAULT;
            speed_state_selector = (int)DRIVE_STATE::DEFAULT_DRIVE;
            arm_state_selector = (int)ARM_STATE::REST;
        }
    
        //LIFT CONTROLS
        //override switch
        if(controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
            if(arm_state == ARM_STATE::OVERRIDE){
                arm_motor_left.tare_position();
                arm_motor_right.tare_position();
                leftArmMotorZero = arm_motor_left.get_position();
                rightArmMotorZero = arm_motor_right.get_position();
                arm_state_selector = (int)ARM_STATE::REST;
                arm_state = ARM_STATE::REST;
            }
            else{
                arm_state_selector = (int)ARM_STATE::OVERRIDE;
                arm_state = ARM_STATE::OVERRIDE;
            }
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

            arm_state_selector = std::clamp(arm_state_selector, 0, 1);
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


        double drive_rpm_scale; //This is the same as making the bot go mach fuck or abysmally slow, just accounts for normalization now instead of using 0.7
        switch(driveState){
            case DRIVE_STATE::SLOW_DRIVE:    
                drive_rpm_scale = 0.4; break;
            case DRIVE_STATE::DEFAULT_DRIVE:
                drive_rpm_scale = 0.7; break;
            case DRIVE_STATE::FAST_DRIVE:
                drive_rpm_scale = 1.0; break;
        }
        
        //drive motors
        left_motor_front.move_velocity(vel_fl * MOTOR_GREEN_GEAR_MULTIPLIER * drive_rpm_scale);
        left_motor_back.move_velocity(vel_bl * MOTOR_GREEN_GEAR_MULTIPLIER* drive_rpm_scale);
        right_motor_front.move_velocity(vel_fr * MOTOR_GREEN_GEAR_MULTIPLIER* drive_rpm_scale);
        right_motor_back.move_velocity(vel_br * MOTOR_GREEN_GEAR_MULTIPLIER* drive_rpm_scale);

        //lift motors
        double leftArmVolt = 0;
        double rightArmVolt = 0;

        //if arms are not within target error && arms are not in override, calc PID
        if (arm_state != prev_arm_state){
            leftArmPID.reset();
            rightArmPID.reset();
            prev_arm_state = arm_state;
        }

        if(!(fabs(arm_motor_left.get_position() - leftArmMotorZero - armTarget) < TARGET_ERROR &&
           fabs(arm_motor_right.get_position() - rightArmMotorZero - armTarget) < TARGET_ERROR) && arm_state != ARM_STATE::OVERRIDE){

            double time_step = pros::millis() - loop_start + this->opcontrol_delay_ms + 1; //calculates dt for pid; hardcoded dt assumed loop ran perfectly at 10ms; the + 1 protects against 0 division
            leftArmVolt = leftArmPID.calculatePID(armTarget, arm_motor_left.get_position() - leftArmMotorZero, time_step);
            rightArmVolt = rightArmPID.calculatePID(armTarget, arm_motor_right.get_position() - rightArmMotorZero, time_step);
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
        
        //This if block needs extensive debugging/testing
        if(timed_outake && allow_timed_outake){
            if(!timed_outake_started){ // capture position only on the first iteration
                intake_position_hold_l = intake_motor_left.get_position();
                intake_position_hold_r = intake_motor_right.get_position();
                timed_outake_started = true;
            }

            intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.85);
            intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.85);
        }

        if(break_timer && allow_timed_outake && !timed_outake){ //locks motors into last recorded position to shoot ball at the end
            intake_motor_left.move_absolute(intake_position_hold_l, 100);
            intake_motor_right.move_absolute(intake_position_hold_r, 100);
        }

        pros::lcd::set_text(2, std::to_string(timer_limit));
        
        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}
