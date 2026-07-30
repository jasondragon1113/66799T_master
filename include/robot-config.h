#pragma once
#include "Template/api.h"

using namespace pros;

extern Drive chassis;

extern IMU inertial;
extern Rotation fwd_tracker;
extern Rotation sideways_tracker;

extern Motor leftFront;
extern Motor leftMiddle;
extern Motor leftBack;
extern Motor rightFront;
extern Motor rightMiddle;
extern Motor rightBack;

extern MotorGroup leftMotors;
extern MotorGroup rightMotors;

extern Motor intake;
extern Motor cascade1;
extern Motor cascade2;
extern Motor arm;

extern adi::DigitalOut claw;
extern adi::DigitalOut toggle;
extern adi::DigitalIn cascade_limit;

extern Distance distance_sensorL;
extern Distance distance_sensorR;
extern Rotation arm_rotation;

extern lemlib::Drivetrain drivetrain;
extern lemlib::OdomSensors sensors;
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis_lemlib;

void default_constants();
void init();

// Your motors, sensors, etc. should go here.  Below are examples
// inline pros::adi::DigitalIn limit_switch('A');