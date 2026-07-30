#include "main.h"

ArmPosition arm_target = ArmPosition::DOWN;

// Target angles in ARM degrees, straight from arm_rotation (port 21).
//
// To re-measure any of these:
//   1. Build and run. The arm holds wherever it starts, which becomes 0.
//   2. Move the arm by hand (or with the buttons) to the position you want.
//   3. Read "arm_angle" on the vexdash Graph panel and put that number here.
// Or drag ARM_*_DEG on the dashboard Config panel and watch the arm move --
// the values write back live, so you can find them without rebuilding.
//
// DOWN is 0, and 0 is now a FIXED physical position -- the arm's bottom hard
// stop -- not "wherever the arm was at boot". See ARM_ZERO_ANGLE_DEG below.
float ARM_DOWN_DEG = 0;
float ARM_POS_1_DEG = 283;
float ARM_POS_2_DEG = 160;   // LEFT sequence's final position, after the cascade is back at 0
float ARM_POS_3_DEG = 265;  // LEFT sequence's raised position, before coming back to POS_2

// Soft travel limits in arm degrees. Targets are clamped here so a bad preset
// stalls the motor against nothing instead of slamming the hard stop.
// Set ARM_MAX_DEG to the arm's real measured travel.
float ARM_MIN_DEG = 0;
float ARM_MAX_DEG = 287;

// The rotation sensor must count UP when the motor drives the arm UP. If the
// arm runs away from its target instead of settling on it, flip this first --
// it's the usual cause, not the gains.
bool ARM_ROTATION_REVERSED = false;

// The arm runs its own PID (see PID.h/PID.cpp, same class the drive/turn PID
// uses) against the rotation sensor -- tune these directly.
//
// kP/kD are 3x the old values. Error used to be motor degrees; it's now arm
// degrees, which is ~3x smaller for the same physical error (old
// ARM_GEAR_RATIO), so the gains are scaled up to keep the same volts-per-inch
// of actual arm movement as before. Re-tune from here.
float ARM_KP = 2;
// KI only turns on within ARM_STARTI of target -- lets the I term slowly
// build enough torque to grind through friction/stiction on the last few
// degrees (where KP*error alone is too weak to move the arm), without
// winding up during a large move.
float ARM_KI = 0.2;
float ARM_KD = 0.5;
float ARM_STARTI = 10; // max error (arm degrees) before the I term starts accumulating

const int ARM_MAX_VOLTAGE = 97; // out of 127, clamps the PID output
const int ARM_DOWN_MAX_VOLTAGE = 57; // out of 127, clamps output while descending so the arm goes down slower

// Max error (arm degrees) to be considered "arrived" -- see arm_settled below.
// The old 20 motor degrees was ~6.7 arm degrees; this is a bit tighter. Loosen
// it if preset sequences start hitting their PRESET_STEP_TIMEOUT_MS.
float ARM_SETTLE_ERROR_DEG = 5;

// True once the arm is within ARM_SETTLE_ERROR_DEG of arm_target. Lets other
// code (e.g. Drive::control_arcade) wait for the arm to actually get there
// before doing something that depends on it, instead of guessing a delay.
bool arm_settled = false;

bool arm_sensor_ok = false;

float tele_arm_angle = 0;
float tele_arm_target = 0;
float tele_arm_error = 0;
float tele_arm_output = 0;

// Last known-good angle, so a momentary sensor dropout doesn't read as 0 (which
// would look like a huge error and slam the arm).
static float arm_last_good_deg = 0;

void arm_set_position(ArmPosition pos){
  arm_target = pos;
}

// Rotation::get_position() returns centidegrees, and PROS_ERR when the sensor
// isn't reporting. Updates arm_sensor_ok as a side effect.
float arm_get_position_deg(){
  std::int32_t centideg = arm_rotation.get_position();

  if(centideg == PROS_ERR){
    arm_sensor_ok = false;
    return arm_last_good_deg;
  }

  arm_sensor_ok = true;
  arm_last_good_deg = centideg / 100.0;
  return arm_last_good_deg;
}

float arm_target_degrees(ArmPosition pos){
  float arm_deg;
  switch(pos){
    case ArmPosition::DOWN:  arm_deg = ARM_DOWN_DEG;  break;
    case ArmPosition::POS_1: arm_deg = ARM_POS_1_DEG; break;
    case ArmPosition::POS_2: arm_deg = ARM_POS_2_DEG; break;
    case ArmPosition::POS_3: arm_deg = ARM_POS_3_DEG; break;
    default:                 arm_deg = ARM_DOWN_DEG;  break;
  }
  return clamp(arm_deg, ARM_MIN_DEG, ARM_MAX_DEG);
}

void arm_task(){
  // Default brake mode is COAST, which would let the arm sag under gravity
  // between PID updates instead of holding position.
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  arm_rotation.set_reversed(ARM_ROTATION_REVERSED);
  arm_rotation.set_data_rate(5); // ms, so the 10ms loop always has a fresh sample
  arm_rotation.reset_position(); // wherever the arm physically is at program start becomes 0

  // The motor encoder no longer drives the PID, but the MOTORS tab of the V5
  // dashboard shows it, so keep it zeroed at the same moment.
  arm.tare_position();

  arm_target = ArmPosition::DOWN;

  PID armPID(0, ARM_KP, ARM_KI, ARM_KD, ARM_STARTI);

  while(true){
    float target = arm_target_degrees(arm_target);
    float position = arm_get_position_deg();
    float error = target - position;

    tele_arm_angle = position;
    tele_arm_target = target;
    tele_arm_error = error;

    // With no trustworthy position there's no safe direction to drive, so stop
    // and let the brake mode hold the arm where it is.
    if(!arm_sensor_ok){
      arm.move(0);
      tele_arm_output = 0;
      arm_settled = false;
      delay(10);
      continue;
    }

    float output = armPID.compute(error);

    int max_voltage = output < 0 ? ARM_DOWN_MAX_VOLTAGE : ARM_MAX_VOLTAGE;
    output = clamp(output, (float)-max_voltage, (float)max_voltage);

    arm.move(output);
    tele_arm_output = output;
    arm_settled = fabs(error) < ARM_SETTLE_ERROR_DEG;
    delay(10);
  }
}

void start_arm_task(){
  static Task arm_bg_task(arm_task);
}
