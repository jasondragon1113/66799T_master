#include "main.h"

ArmPosition arm_target = ArmPosition::DOWN;

// Small placeholder values, in ARM output-shaft degrees (not motor degrees --
// see ARM_GEAR_RATIO below). Tune once the real positions have been tested.
int ARM_POS_1_DEG =1120;
int ARM_POS_2_DEG = 720;

// Motor turns per arm turn, e.g. a 12t motor pinion driving a 36t arm gear is
// 3.0. PLACEHOLDER -- count the teeth on both gears and correct this, since
// there's no rotation sensor on the arm output shaft to measure it directly
// (everything is derived from the motor's own internal encoder).
double ARM_GEAR_RATIO = 3.0;

// move_absolute()'s internal V5 position controller has no exposed gains in
// this PROS version, so the arm runs its own PID (see PID.h/PID.cpp, same
// class the drive/turn PID uses) instead -- tune these directly.
float ARM_KP = 0.5;
float ARM_KI = 0.0;
float ARM_KD = 0.0;
float ARM_STARTI = 0; // max error (motor degrees) before the I term starts accumulating

const int ARM_MAX_VOLTAGE = 100; // out of 127, clamps the PID output

// Max error (motor degrees) to be considered "arrived" -- see arm_settled below.
float ARM_SETTLE_ERROR_DEG = 20;

// True once the arm is within ARM_SETTLE_ERROR_DEG of arm_target. Lets other
// code (e.g. Drive::control_arcade) wait for the arm to actually get there
// before doing something that depends on it, instead of guessing a delay.
bool arm_settled = false;

void arm_set_position(ArmPosition pos){
  arm_target = pos;
}

// Converts a target ARM output-shaft angle to the motor-shaft angle
// the arm PID actually drives to.
int arm_target_motor_degrees(ArmPosition pos){
  int arm_deg;
  switch(pos){
    case ArmPosition::POS_1: arm_deg = ARM_POS_1_DEG; break;
    case ArmPosition::POS_2: arm_deg = ARM_POS_2_DEG; break;
    default:                 arm_deg = 0;             break;
  }
  return (int)(arm_deg * ARM_GEAR_RATIO);
}

void arm_task(){
  // Default brake mode is COAST, which would let the arm sag under gravity
  // between PID updates instead of holding position.
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  arm.tare_position(); // wherever the arm physically is at program start becomes 0
  arm_target = ArmPosition::DOWN;

  PID armPID(0, ARM_KP, ARM_KI, ARM_KD, ARM_STARTI);

  while(true){
    float error = arm_target_motor_degrees(arm_target) - arm.get_position();
    float output = armPID.compute(error);

    if(output > ARM_MAX_VOLTAGE) output = ARM_MAX_VOLTAGE;
    if(output < -ARM_MAX_VOLTAGE) output = -ARM_MAX_VOLTAGE;

    arm.move(output);
    arm_settled = fabs(error) < ARM_SETTLE_ERROR_DEG;
    delay(10);
  }
}

void start_arm_task(){
  static Task arm_bg_task(arm_task);
}
