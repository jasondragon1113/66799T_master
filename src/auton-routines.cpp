#include "main.h"

void left(){
default_constants();
chassis.drive_max_voltage = 127;
chassis.drive_distance(20);
chassis.turn_to_angle(90);
chassis.drive_stop(MotorBrake::brake);

}

void right(){
default_constants();
}

void sawp(){
default_constants();
}