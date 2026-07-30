#include "main.h"

const int CASCADE_SCORE_VELOCITY = 100;   // move_absolute() speed for score(), out of 200 rpm
const int CASCADE_SCORE_SETTLE_DEG = 20;  // max cascade error to be considered "arrived"
const int CASCADE_SCORE_TIMEOUT_MS = 3000; // give up waiting and move on after this long

void score(ScoringLevel level, ArmPosition arm_pos){
  int cascade_target = (int)level;

  arm_set_position(arm_pos);
  cascade1.move_absolute(cascade_target, CASCADE_SCORE_VELOCITY);
  cascade2.move_absolute(cascade_target, CASCADE_SCORE_VELOCITY);

  int waited_ms = 0;
  while(!arm_settled || fabs(cascade1.get_position() - cascade_target) > CASCADE_SCORE_SETTLE_DEG){
    delay(10);
    waited_ms += 10;
    if(waited_ms > CASCADE_SCORE_TIMEOUT_MS) break;
  }
}

void left(){
default_constants();
toggle.set_value(false);
chassis.drive_with_voltage(-67,-67);
delay(500);
score(ScoringLevel::LEVEL_1, ArmPosition::DOWN);
chassis.drive_distance(15.75,307,false);
chassis.drive_distance(8,284,false);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
delay(200);
claw.set_value(false);
delay(50);
chassis.drive_stop(MotorBrake::brake);
chassis.drive_distance(-11.5);
score(ScoringLevel::LEVEL_0, ArmPosition::DOWN);
chassis.turn_to_angle(245.5);
chassis.drive_max_voltage = 107;
chassis.drive_distance(18.5);
chassis.turn_to_angle(220);
// chassis.drive_distance(5);
chassis.drive_max_voltage = 87;
chassis.drive_distance(3,200,false);
claw.set_value(true);
chassis.drive_distance(-20,280,false);
chassis.drive_stop(MotorBrake::brake);

}

void right(){
default_constants();

}

void sawp(){
default_constants();
}