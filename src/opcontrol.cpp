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
#include <fstream>
#include <ctime>

using namespace pros;
using namespace umbc;
using namespace std;

#define MOTOR_RED_GEAR_MULTIPLIER 100
#define MOTOR_GREEN_GEAR_MULTIPLIER 200
#define MOTOR_BLUE_GEAR_MULTIPLIER 600
#define MOTOR_REVERSE true
#define REVERSED(port) -port

#define INTAKE_MOTOR_SPEED 100
#define TARGET_ERROR 10

// left and right are relative to the robot's left and right

// DOUBLE CHECK MOTORS!!!!!

// ports for left drive motors (green)
#define LEFT_MOTOR_FRONT 2
#define LEFT_MOTOR_BACK 11

// ports for right drive motors (green)
#define RIGHT_MOTOR_FRONT -7
#define RIGHT_MOTOR_BACK -20

// ports for lift motors (red)
#define ARM_MOTOR_RIGHT -21
#define ARM_MOTOR_LEFT 5

// ports for intake motors (blue)
#define INTAKE_MOTOR_LEFT 12
#define INTAKE_MOTOR_RIGHT -15
#define REST_POSITION -65      // low goal
#define MID_GOAL_POSITION -850 // mid goal

/*
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

        double calculatePID(double target, double current){
            error = target - current;
            changeError = error - prev_error;
            totalError += error;
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
*/

#define KP 3
#define KD 0
#define KI 0
#define KBIAS 0

// These are to stringify macro names
#define STR(x) #x
#define XSTR(x) STR(x)

// initialize motors
std::vector<int8_t> drive_motors{LEFT_MOTOR_FRONT, LEFT_MOTOR_BACK, RIGHT_MOTOR_FRONT, RIGHT_MOTOR_BACK};
std::vector<int8_t> intake_motors{INTAKE_MOTOR_LEFT, INTAKE_MOTOR_RIGHT};
std::vector<int8_t> arm_motors{ARM_MOTOR_LEFT, ARM_MOTOR_RIGHT};

pros::MotorGroup driveGroup(drive_motors);
pros::MotorGroup intakeGroup(intake_motors);
pros::MotorGroup armGroup(arm_motors);

void umbc::Robot::opcontrol()
{

    // nice names for controllers (do not edit)
    umbc::Controller *controller_master = this->controller_master;
    umbc::Controller *controller_partner = this->controller_partner;

    // initialize PID controller for arm

    okapi::ConfigurableTimeUtilFactory global_time_factory;
    okapi::TimeUtil global_time = global_time_factory.create();
    okapi::IterativePosPIDController arm_controller(KP, KI, KD, KBIAS, global_time);

    // initialize motors
    std::vector<int8_t> drive_motors{LEFT_MOTOR_FRONT, LEFT_MOTOR_BACK, RIGHT_MOTOR_FRONT, RIGHT_MOTOR_BACK};
    std::vector<int8_t> intake_motors{INTAKE_MOTOR_LEFT, INTAKE_MOTOR_RIGHT};
    std::vector<int8_t> arm_motors{ARM_MOTOR_RIGHT, ARM_MOTOR_LEFT};
    pros::MotorGroup driveGroup(drive_motors);
    pros::MotorGroup intakeGroup(intake_motors);
    pros::MotorGroup armGroup(arm_motors);

    // brakes and gearing
    // DRIVE
    driveGroup.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    driveGroup.set_gearing(E_MOTOR_GEAR_GREEN);

    // INTAKE
    intakeGroup.set_brake_mode(E_MOTOR_BRAKE_COAST);
    intakeGroup.set_gearing(E_MOTOR_GEAR_BLUE);

    // ARM
    armGroup.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
    armGroup.set_gearing(E_MOTOR_GEAR_RED);

    // drive motors
    pros::Motor left_motor_front(LEFT_MOTOR_FRONT);
    pros::Motor left_motor_back(LEFT_MOTOR_BACK);

    pros::Motor right_motor_front(RIGHT_MOTOR_FRONT);
    pros::Motor right_motor_back(RIGHT_MOTOR_BACK);

    // intake motors
    pros::Motor intake_motor_left(INTAKE_MOTOR_LEFT);
    pros::Motor intake_motor_right(INTAKE_MOTOR_RIGHT);

    // arm motors
    pros::Motor arm_motor_right(ARM_MOTOR_RIGHT);
    pros::Motor arm_motor_left(ARM_MOTOR_LEFT);

    // motor states
    bool slowSpeedButton = false;

    enum class INTAKE_STATE
    {
        INTAKE_OFF,
        INTAKE_ON,
        INTAKE_REVERSE
    };
    INTAKE_STATE intakeState = INTAKE_STATE::INTAKE_OFF;

    enum class ARM_STATE
    {
        REST,
        MID_GOAL
    }; // implement HIGH_GOAL if the bot can reach it
    ARM_STATE cur_arm_state = ARM_STATE::REST;
    int arm_state_selector = 0;

    enum class DRIVE_STATE
    {
        SLOW_DRIVE,
        DEFAULT_DRIVE,
        FAST_DRIVE
    };
    DRIVE_STATE driveState = DRIVE_STATE::DEFAULT_DRIVE;
    int speed_state_selector = 1;

    enum class SCORE_SPEED
    {
        SLOW,
        DEFAULT,
        FAST
    };
    int score_speed_selector = 1;

    while (1)
    {
        // left joystick (target movement)
        double left_x = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_X);
        double left_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        // controller deadzone
        if (left_x > -15 && left_x < 15)
        {
            left_x = 0;
        }
        if (left_y > -15 && left_y < 15)
        {
            left_y = 0;
        }
        left_x = pow(left_x, 3) / (127 * 127 * 127); // cubing for finer control
        left_y = pow(left_y, 3) / (127 * 127 * 127);

        // right joystick (rotation)
        double right_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);
        right_x = pow(right_x, 3) / (127 * 127 * 127) * 0.75; // cubing for finer control

        // converting to polar
        double radius = sqrt(left_x * left_x + left_y * left_y);
        double theta = atan2(left_y, left_x);

        // this will determine the speed of each motor
        double power_front_left = sin(theta + (M_PI) / 4) / cos(M_PI / 4);
        double power_front_right = -cos(theta + (M_PI / 4)) / cos(M_PI / 4);
        double power_back_left = -cos(theta + (M_PI / 4)) / cos(M_PI / 4);
        double power_back_right = sin(theta + (M_PI) / 4) / cos(M_PI / 4);

        double speed = 0;
        if (radius != 0)
        {
            speed = max({fabs(power_back_left), fabs(power_back_right), fabs(power_front_left), fabs(power_front_right)}) / radius;
        }

        double vel_fl = 0;
        double vel_fr = 0;
        double vel_bl = 0;
        double vel_br = 0;
        if (speed != 0)
        {
            vel_fl = power_front_left;
            vel_fr = power_front_right;
            vel_bl = power_back_left;
            vel_br = power_back_right;
        }

        // SPEED CONTROL
        if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_UP))
        { // toggle slow speed
            speed_state_selector++;
        }
        if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN))
        { // toggle slow speed
            speed_state_selector--;
        }

        if (speed_state_selector > 2)
        {
            speed_state_selector = 2;
        }
        else if (speed_state_selector < 0)
        {
            speed_state_selector = 0;
        }
        driveState = (DRIVE_STATE)speed_state_selector;

        // INTAKE CONTROLS
        if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R1))
        { // toggle intake on
            if (intakeState == INTAKE_STATE::INTAKE_ON)
            {
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else
            {
                intakeState = INTAKE_STATE::INTAKE_ON;
            }
        }
        if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R2))
        { // toggle intake reverse
            if (intakeState == INTAKE_STATE::INTAKE_REVERSE)
            {
                intakeState = INTAKE_STATE::INTAKE_OFF;
            }
            else
            {
                intakeState = INTAKE_STATE::INTAKE_REVERSE;
            }
        }

        if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT))
        { // changes score speed, mainly for skills
            score_speed_selector++;
        }
        if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT))
        { // changes score speed, mainly for skills
            score_speed_selector--;
        }

        if (score_speed_selector > 2)
        {
            score_speed_selector = 2;
        }
        else if (score_speed_selector < 0)
        {
            score_speed_selector = 0;
        }

        SCORE_SPEED score_speed = (SCORE_SPEED)score_speed_selector;

        // LIFT CONTROLS
        // for overide, add toggle button, then just switch code for L1 and L2

        if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_L1))
        { // move arm up
            arm_state_selector++;
        }

        if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_L2))
        { // move arm down
            arm_state_selector--;
        }

        if (arm_state_selector > 1)
        {
            arm_state_selector = 1;
        }
        else if (arm_state_selector < 0)
        {
            arm_state_selector = 0;
        }
        cur_arm_state = (ARM_STATE)arm_state_selector;

        switch (cur_arm_state)
        {
        case ARM_STATE::REST:
            arm_controller.setTarget(REST_POSITION);
            break;

        case ARM_STATE::MID_GOAL:
            arm_controller.setTarget(MID_GOAL_POSITION);
            break;
        }

        // MOVING MOTORS
        // slow speed if toggled on
        switch (driveState)
        {
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

        // intake motors
        switch (intakeState)
        {
        case INTAKE_STATE::INTAKE_OFF:
            intake_motor_left.move_velocity(0);
            intake_motor_right.move_velocity(0);
            break;
        case INTAKE_STATE::INTAKE_ON:
            intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * -0.85);
            intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * -0.85);
            break;
        case INTAKE_STATE::INTAKE_REVERSE:
            if (score_speed == SCORE_SPEED::SLOW)
            {
                intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.1);
                intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.1);
            }
            else if (score_speed == SCORE_SPEED::DEFAULT)
            {
                intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.2);
                intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.2);
            }
            else
            {
                intake_motor_left.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.85);
                intake_motor_right.move_velocity(MOTOR_BLUE_GEAR_MULTIPLIER * 0.85);
            }
            break;
        }

        // drive motors
        left_motor_front.move_velocity((vel_fl + right_x) * MOTOR_GREEN_GEAR_MULTIPLIER * 0.7);
        left_motor_back.move_velocity((vel_bl + right_x) * MOTOR_GREEN_GEAR_MULTIPLIER * 0.7);
        right_motor_front.move_velocity((vel_fr - right_x) * MOTOR_GREEN_GEAR_MULTIPLIER * 0.7);
        right_motor_back.move_velocity((vel_br - right_x) * MOTOR_GREEN_GEAR_MULTIPLIER * 0.7);

        // lift motors

        // double leftArmVolt = leftArmPID.calculatePID(armTarget, arm_motor_left.get_position() - leftArmMotorZero);
        // double rightArmVolt = rightArmPID.calculatePID(armTarget, arm_motor_right.get_position() - rightArmMotorZero);

        // arm_motor_left.move_voltage(leftArmVolt);
        // arm_motor_right.move_voltage(rightArmVolt);

        armGroup.move_absolute(arm_controller.getTarget(), -MOTOR_RED_GEAR_MULTIPLIER * arm_controller.getOutput() * 0.35);

        arm_controller.step((armGroup.get_position(0) + armGroup.get_position(1)));

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}

//------------------------------------------------------------------------------------------------

// Assuming these are defined in your global scope or main.h
// extern pros::MotorGroup driveGroup;

void drive_doctor(std::ostream &writestream)
{
    // Get timestamp (Number of seconds since Pros Initialized)
    uint32_t now = pros::millis();

    // 1. Temperature Check
    // In PROS 4, we iterate through the group or use get_temperatures()
    std::vector<double> temps = driveGroup.get_temperature_all();
    for (size_t i = 0; i < temps.size(); ++i)
    {
        if (temps[i] > 55.0)
        { // 55C is kinda hot idk
            writestream << "[" << now << "] WARNING: Motor on Index " << i
                        << " is hot: " << temps[i] << "C" << std::endl;
        }
    }

    // 2. Current Check
    std::vector<int32_t> currents = driveGroup.get_current_draw_all();
    for (size_t i = 0; i < currents.size(); ++i)
    {
        if (currents[i] > 2000)
        { // Example: > 2000mA (2A) draw
            writestream << "[" << now << "] ALERT: Motor " << i
                        << " over-current: " << currents[i] << "mA" << std::endl;
        }
    }

    // 3. Efficiency Check (Custom calculation or standard telemetry)
    // but we can log power/velocity ratios or use get_efficiency().
    std::vector<double> efficiencies = driveGroup.get_efficiency_all();
    for (size_t i = 0; i < efficiencies.size(); ++i)
    {
        writestream << "[" << now << "] Motor " << i << " Efficiency: " << efficiencies[i] << "%" << std::endl;
    }
}

void doctor()
{
    // Open the file in Append mode so we don't wipe previous runs
    std::ofstream telemetry_file("/usd/doctors_diagnosis.txt", std::ios::app);

    // If the file isn't open
    if (!telemetry_file.is_open())
    {
        pros::lcd::set_text(1, "SD Card Error: File not opened");
        return;
    }

    telemetry_file << "--- DOCTOR DAVEY'S SESSION STARTED ---" << std::endl;

    while (true)
    {
        drive_doctor(telemetry_file);
        // intake_doctor(telemetry_file);
        // arm_doctor(telemetry_file);

        // ESSENTIAL: Flush data to the SD card so it saves if the robot is turned off
        telemetry_file.flush();

        // Delay to prevent CPU hogging and SD card wear
        pros::delay(500);
    }
}

// How to start it in initialize() or autonomous()
void start_diagnostics()
{
    pros::Task telemetry_task(doctor);
}