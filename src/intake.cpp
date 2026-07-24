#include "main.h"

// uint32_t clear_start_time = pros::millis();
// uint32_t clear_time = 0;
// uint32_t clog_time = 0;
// uint32_t clog_start_time = pros::millis();
// uint32_t unjam_attempt_time = 0;
// uint32_t unjam_attempt_start_time = pros::millis();
// uint32_t rest_time = 0;
// uint32_t rest_start_time = pros::millis();

int intake_status(){
  int intake_stats_count = 0;

  while(true){
    // if(intake.get_torque() > 0.5){
    //   mid_jam_time += 10;
    // }
    // else{
    //   mid_jam_time = 0;
    // }

    // switch(intake_state){
    //   case IntakeTask::STOP: // stop intake
    //     brake_with_mode(intake, MotorBrake::brake);
    //     brake_with_mode(outtake, MotorBrake::brake);
    //     break;

    //   case IntakeTask::INTAKE: // start intake
    //     intake.move(127);
    //     outtake.move(-127);
    //     break;

    //   case IntakeTask::REVERSE: // rev intake
    //     intake.move(-127);
    //     outtake.move(-127);
    //     break;
    // }

    delay(10);
  }
  return 0;
}