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
#define LEFT_MOTOR_FRONT    11
#define LEFT_MOTOR_BACK     3
       
  
//ports for right motors(blue)
#define RIGHT_MOTOR_FRONT   -1
#define RIGHT_MOTOR_BACK    -6

#define gyro_port1         'A' //port 1 for gyro
#define gyro_port2         'B' //port 2 for gyro
#define gyro_port3         'C' //port 3 for gyro

void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    // initialize motors and sensors
    
    std::vector<int8_t> motors{LEFT_MOTOR_FRONT, LEFT_MOTOR_BACK, RIGHT_MOTOR_FRONT, RIGHT_MOTOR_BACK};
    pros::MotorGroup totalMotors (motors);

    totalMotors.set_brake_modes(E_MOTOR_BRAKE_COAST);
    totalMotors.set_gearing(E_MOTOR_GEAR_BLUE);
    

    pros::Motor left_motor_front (LEFT_MOTOR_FRONT);
    pros::Motor left_motor_back (LEFT_MOTOR_BACK);
        
    pros::Motor right_motor_front (RIGHT_MOTOR_FRONT);
    pros::Motor right_motor_back (RIGHT_MOTOR_BACK);


    //GYRO STUFF
    pros::ADIAnalogIn gyro_sensor1 (gyro_port1);
    pros::ADIAnalogIn gyro_sensor2 (gyro_port2);
    pros::ADIAnalogIn gyro_sensor3 (gyro_port3);

    gyro_sensor1.calibrate();
    gyro_sensor2.calibrate();
    gyro_sensor3.calibrate();

    double curr_angle = 0;

    while(1) {
        //GRYO STUFF
        double gyro_val1 = gyro_sensor1.get_value_calibrated_HR(); //using HR cause this needs to be integrated
        double gyro_val2 = gyro_sensor2.get_value_calibrated_HR();
        double gyro_val3 = gyro_sensor3.get_value_calibrated_HR();

        double gyro_avg = (gyro_val1 + gyro_val2 + gyro_val3) / 3;
        
        //avg rot velocity in degrees per second
        double gyro_deg = gyro_avg / 19.5;          
        
        //TUNE THIS DEADZONE
        if(fabs(gyro_deg) < 0.5){
            gyro_deg = 0;
        }

        curr_angle += gyro_deg * (this->opcontrol_delay_ms);

        if(curr_angle > 360){
            curr_angle -= 360;
        }
        else if(curr_angle < 0){
            curr_angle += 360;
        }

        double curr_rad = curr_angle * (M_PI / 180);

        //left joystick (target movement)
        double left_x = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_X);
        double left_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        //right joystick (rotation)
        double right_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        
        //converting to polar
        double r = sqrt(left_x * left_x + left_y * left_y);
        double theta = atan2(left_y, left_x);

        //FOR FIELD RELATIVE
        theta -= curr_rad; 

        //front right and back left motors
        double power_one = -cos(theta + (M_PI/4)) / cos(M_PI/4);

        //front left and back right motors
        double power_two = sin(theta + (M_PI)/4) / cos(M_PI/4);

        double power_fl = power_two;
        double power_fr = power_one;
        double power_bl = power_one;
        double power_br = power_two; 
        
        double speed1 = 0;
        double speed2 = 0;
        double speed = 0;
        if(r != 0){
            speed1 = max(fabs(power_bl), fabs(power_br));
            speed2 = max(fabs(power_fl), fabs(power_fr));
            speed = max(speed1, speed2) / r;
        }

        double vel_fl = 0;
        double vel_fr = 0;
        double vel_bl = 0;
        double vel_br = 0;
        if(speed != 0){
            vel_fl = power_fl;
            vel_fr = power_fr;
            vel_bl = power_bl;
            vel_br = power_br;
        }

        

        left_motor_front.move_velocity((vel_fl * (1 - abs(right_x)) + right_x)*MOTOR_BLUE_GEAR_MULTIPLIER*0.25);
        left_motor_back.move_velocity((vel_bl * (1 - abs(right_x)) + right_x)*MOTOR_BLUE_GEAR_MULTIPLIER*0.25);
        right_motor_front.move_velocity((vel_fr * (1 - abs(right_x)) - right_x)*MOTOR_BLUE_GEAR_MULTIPLIER*0.25);
        right_motor_back.move_velocity((vel_br * (1 - abs(right_x)) - right_x)*MOTOR_BLUE_GEAR_MULTIPLIER*0.25);

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}