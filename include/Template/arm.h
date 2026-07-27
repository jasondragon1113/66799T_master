#pragma once
#include "main.h"
using namespace pros;

// Arm preset positions, driven by a background task via move_absolute().
// B -> DOWN, A -> POS_1, X -> POS_2 (see Drive::control_arcade in drive.cpp).
enum class ArmPosition {
    DOWN = 0,
    POS_1 = 1,
    POS_2 = 2
};

extern ArmPosition arm_target;

// Placeholder degrees, in ARM output-shaft degrees -- small values on purpose,
// tune once real positions are tested.
extern int ARM_POS_1_DEG;
extern int ARM_POS_2_DEG;

// Motor turns per arm turn (external gear reduction). PLACEHOLDER -- verify
// by counting teeth, see arm.cpp.
extern double ARM_GEAR_RATIO;

// Arm position PID gains -- tune these to adjust how the arm reaches and
// holds its target angle. See arm_task() in arm.cpp.
extern float ARM_KP;
extern float ARM_KI;
extern float ARM_KD;
extern float ARM_STARTI;

// Max error (motor degrees) to be considered "arrived".
extern float ARM_SETTLE_ERROR_DEG;

// True once the arm is within ARM_SETTLE_ERROR_DEG of arm_target -- poll
// this to wait for the arm to actually reach its target.
extern bool arm_settled;

void arm_set_position(ArmPosition pos);
void arm_task();
void start_arm_task();
