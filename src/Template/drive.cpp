#include "main.h"

Drive::Drive(DriveStyle drive_style, MotorGroup& left_motors, MotorGroup& right_motors, IMU& inertial, 
             float wheel_diameter, float motor_gear_ratio, float gyro_scale, 
             Rotation& fwd_tracker, float fwd_tracker_diameter, float fwd_tracker_dist, 
             Rotation& sideways_tracker, float sideways_tracker_diameter, float sideways_tracker_dist):
    drive_style(drive_style),
    DriveL(left_motors), 
    DriveR(right_motors), 
    Gyro(inertial), 

    wheel_diameter(wheel_diameter), 
    wheel_ratio(motor_gear_ratio), 
    gyro_scale(gyro_scale), 
    drive_in_to_deg_ratio(M_PI*wheel_diameter*wheel_ratio/360.0), // 360 is correct while 36000 isn't for some reason. 

    Fwd_tracker(fwd_tracker), 
    ForwardTracker_diameter(fwd_tracker_diameter), 
    ForwardTracker_center_distance(fwd_tracker_dist), 
    ForwardTracker_in_to_deg_ratio(M_PI*fwd_tracker_diameter/36000.0), // 36000 because using PROS API motor.get_position() returns centidegrees

    Sideways_tracker(sideways_tracker), 
    SidewaysTracker_diameter(sideways_tracker_diameter), 
    SidewaysTracker_center_distance(sideways_tracker_dist), 
    SidewaysTracker_in_to_deg_ratio(M_PI*sideways_tracker_diameter/36000.0), // 36000 because using PROS API motor.get_position() returns centidegrees

    master(CONTROLLER_MASTER)
{
    odom.set_physical_distances(ForwardTracker_center_distance, SidewaysTracker_center_distance);
}

/**
 * Drives each side of the chassis at the specified voltage.
 * 
 * @param left_voltage Voltage out of 127.
 * @param right_voltage Voltage out of 127.
 */
void Drive::drive_with_voltage(int left_voltage, int right_voltage){
    DriveL.move(left_voltage);
    DriveR.move(right_voltage);
}

/**
 * Resets default drive constants.
 * Driving includes drive_distance(), drive_to_point(), and
 * holonomic_drive_to_point().
 * 
 * @param drive_max_voltage Max voltage out of 127.
 * @param drive_kp Proportional constant.
 * @param drive_ki Integral constant.
 * @param drive_kd Derivative constant.
 * @param drive_starti Minimum distance in inches for integral to begin
 * @param drive_min_voltage Min voltage out of 127.
 */
void Drive::set_drive_constants(float drive_max_voltage, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float drive_min_voltage){ // ADDED MIN_VOLTAGE!!!!!
  this->drive_max_voltage = drive_max_voltage;
  this->drive_kp = drive_kp;
  this->drive_ki = drive_ki;
  this->drive_kd = drive_kd;
  this->drive_starti = drive_starti;
  this->drive_min_voltage = drive_min_voltage;
}

void Drive::set_drive_motion_chain_constants(float motion_chain_drive_min_voltage, float motion_chain_drive_early_exit_range){
  this->motion_chain_drive_min_voltage = motion_chain_drive_min_voltage;
  this->motion_chain_drive_early_exit_range = motion_chain_drive_early_exit_range;
}

void Drive::set_turn_motion_chain_constants(float motion_chain_turn_min_voltage, float motion_chain_turn_early_exit_range){
  this->motion_chain_turn_min_voltage = motion_chain_turn_min_voltage;
  this->motion_chain_turn_early_exit_range = motion_chain_turn_early_exit_range;
}

/**
 * Resets default turn constants.
 * Turning includes turn_to_angle() and turn_to_point().
 * 
 * @param turn_max_voltage Max voltage out of 127.
 * @param turn_kp Proportional constant.
 * @param turn_ki Integral constant.
 * @param turn_kd Derivative constant.
 * @param turn_starti Minimum angle in degrees for integral to begin.
 */
void Drive::set_turn_constants(float turn_max_voltage, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  this->turn_max_voltage = turn_max_voltage;
  this->turn_kp = turn_kp;
  this->turn_ki = turn_ki;
  this->turn_kd = turn_kd;
  this->turn_starti = turn_starti;
} 

/**
 * Resets default heading constants.
 * Heading control keeps the robot facing the right direction
 * and is part of drive_distance() and drive_to_point().
 * 
 * @param heading_max_voltage Max voltage out of 127.
 * @param heading_kp Proportional constant.
 * @param heading_ki Integral constant.
 * @param heading_kd Derivative constant.
 * @param heading_starti Minimum angle in degrees for integral to begin.
 */
void Drive::set_heading_constants(float heading_max_voltage, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  this->heading_max_voltage = heading_max_voltage;
  this->heading_kp = heading_kp;
  this->heading_ki = heading_ki;
  this->heading_kd = heading_kd;
  this->heading_starti = heading_starti;
}

/**
 * Resets default swing constants.
 * Swing control holds one side of the drive still and turns with the other.
 * Only left_swing_to_angle() and right_swing_to_angle() use these constants.
 * 
 * @param swing_max_voltage Max voltage out of 127.
 * @param swing_kp Proportional constant.
 * @param swing_ki Integral constant.
 * @param swing_kd Derivative constant.
 * @param swing_starti Minimum angle in degrees for integral to begin.
 */
void Drive::set_swing_constants(float swing_max_voltage, float swing_kp, float swing_ki, float swing_kd, float swing_starti){
  this->swing_max_voltage = swing_max_voltage;
  this->swing_kp = swing_kp;
  this->swing_ki = swing_ki;
  this->swing_kd = swing_kd;
  this->swing_starti = swing_starti;
} 

/**
 * Resets default turn constants.
 * Turning includes turn_to_angle() and turn_to_point().
 * 
 * @param wall_max_voltage Max voltage out of 127.
 * @param wall_kp Proportional constant.
 * @param wall_ki Integral constant.
 * @param wall_kd Derivative constant.
 * @param wall_starti Minimum distance in inches for integral to begin.
 */
void Drive::set_wall_constants(float wall_max_voltage, float wall_kp, float wall_ki, float wall_kd, float wall_starti){
  this->wall_max_voltage = wall_max_voltage;
  this->wall_kp = wall_kp;
  this->wall_ki = wall_ki;
  this->wall_kd = wall_kd;
  this->wall_starti = wall_starti;
} 

/**
 * Resets default turn exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param turn_settle_error Error to be considered settled in degrees.
 * @param turn_settle_time Time to be considered settled in milliseconds.
 * @param turn_timeout Time before quitting and move on in milliseconds.
 */
void Drive::set_turn_exit_conditions(float turn_settle_error, float turn_settle_time, float turn_timeout){
  this->turn_settle_error = turn_settle_error;
  this->turn_settle_time = turn_settle_time;
  this->turn_timeout = turn_timeout;
}

/**
 * Resets default drive exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param drive_settle_error Error to be considered settled in inches.
 * @param drive_settle_time Time to be considered settled in milliseconds.
 * @param drive_timeout Time before quitting and move on in milliseconds.
 */
void Drive::set_drive_exit_conditions(float drive_settle_error, float drive_settle_time, float drive_timeout){
  this->drive_settle_error = drive_settle_error;
  this->drive_settle_time = drive_settle_time;
  this->drive_timeout = drive_timeout;
}

/**
 * Resets default swing exit conditions.
 * The robot exits when error is less than settle_error for a duration of settle_time, 
 * or if the function has gone on for longer than timeout.
 * 
 * @param swing_settle_error Error to be considered settled in degrees.
 * @param swing_settle_time Time to be considered settled in milliseconds.
 * @param swing_timeout Time before quitting and move on in milliseconds.
 */
void Drive::set_swing_exit_conditions(float swing_settle_error, float swing_settle_time, float swing_timeout){
  this->swing_settle_error = swing_settle_error;
  this->swing_settle_time = swing_settle_time;
  this->swing_timeout = swing_timeout;
}

/**
 * Gives the drive's absolute heading with Gyro correction.
 * 
 * @return Gyro scale-corrected heading in the range [0, 360).
 */
float Drive::get_absolute_heading(){ 
  return( reduce_0_to_360(Gyro.get_heading() *360.0/gyro_scale ) ); 
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Left position in inches.
 */
float Drive::get_left_position_in(){
  return( DriveL.get_position() *drive_in_to_deg_ratio ); 
}

/**
 * Gets the motor group's position and converts to inches.
 * 
 * @return Right position in inches.
 */

float Drive::get_right_position_in(){
  return( DriveR.get_position() *drive_in_to_deg_ratio );
}

/**
 * Stops both sides of the drive with the desired mode.
 * 
 * @param mode hold, brake, or stop
 */

void Drive::drive_stop(MotorBrake mode){
  MotorBrake old_mode = DriveL.get_brake_mode();

    DriveL.set_brake_mode_all(mode);
    DriveR.set_brake_mode_all(mode);
    // chassis.drive_with_voltage(0, 0);
    DriveL.brake();
    DriveR.brake();

    // DriveL.set_brake_mode_all(old_mode);
    // DriveR.set_brake_mode_all(old_mode);
}

/**
 * Turns the robot to a field-centric angle.
 * Optimizes direction, so it turns whichever way is closer to the 
 * current heading of the robot.
 * 
 * @param angle Desired angle in degrees.
 * @param extra_angle_deg Additional angle to add to the desired angle.
 */

void Drive::turn_to_angle(float angle, bool motion_chaining){
  turn_to_angle(angle, 0, 0, motion_chaining);
}

void Drive::turn_to_angle(float angle, float extra_angle_deg, bool motion_chaining){
  turn_to_angle(angle, extra_angle_deg, 0, motion_chaining);
}

void Drive::turn_to_angle(float angle, float extra_angle_deg, float extra_drive_voltage, bool motion_chaining){
  angle += extra_angle_deg;
  tele_turn_target = angle; // vexdash telemetry
  PID turnPID(reduce_negative_180_to_180(angle - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while( !turnPID.is_settled() ){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    tele_turn_error = error; // vexdash telemetry

    if(motion_chaining && fabs(error) < motion_chain_turn_early_exit_range){
      break;
    }

    float output = turnPID.compute(error);
    tele_turn_output = output; // vexdash telemetry
    output = clamp(output, -turn_max_voltage, turn_max_voltage);

    if(motion_chaining){
      output = clamp_min_voltage(output, motion_chain_turn_min_voltage);
    }

    drive_with_voltage(output + extra_drive_voltage, -output + extra_drive_voltage);
    delay(10);
  }
}

/**
 * Drives the robot a given distance with a given heading.
 * Drive distance does not optimize for direction, so it won't try
 * to drive at the opposite heading from the one given to get there faster.
 * You can control the heading, but if you choose not to, it will drive with the
 * heading it's currently facing. It uses the average of the left and right
 * motor groups to calculate distance driven.
 * 
 * Passing a heading that differs from the one the robot is currently facing
 * makes it arc into that heading while it drives, which is how you get a
 * curved move instead of a turn-then-straight. Combine it with
 * motion_chaining to exit early (with the drive still powered at
 * motion_chain_drive_min_voltage) and blend into the next movement.
 *
 * @param distance Desired distance in inches.
 * @param heading Desired heading in degrees. Defaults to the current heading.
 * @param motion_chaining Exit early once within motion_chain_drive_early_exit_range.
 * @param extra_drive_voltage Voltage added to both sides, out of 127.
 */

void Drive::drive_distance(float distance){
  drive_distance(distance, get_absolute_heading(), false, 0);
}

void Drive::drive_distance(float distance, bool motion_chaining){
  drive_distance(distance, get_absolute_heading(), motion_chaining, 0);
}

void Drive::drive_distance(float distance, float heading, bool motion_chaining){
  drive_distance(distance, heading, motion_chaining, 0);
}

void Drive::drive_distance(float distance, float heading, bool motion_chaining, float extra_drive_voltage){
  tele_drive_target = distance; // vexdash telemetry
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;
  while(drivePID.is_settled() == false){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    drive_error = distance+start_average_position-average_position;
    if(motion_chaining && fabs(drive_error) < motion_chain_drive_early_exit_range){
      break;
    }

    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    tele_drive_output = drive_output; // vexdash telemetry
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    if(motion_chaining){
      drive_output = clamp_min_voltage(drive_output, motion_chain_drive_min_voltage);
    }

    float left_voltage = drive_output+heading_output + extra_drive_voltage;
    float right_voltage = drive_output-heading_output + extra_drive_voltage;
    drive_with_voltage(left_voltage, right_voltage);

    // TEMP DEBUG -- remove once the slow/no-turn issue is diagnosed.
    printf("heading: %.1f, heading_err: %.1f, drive_out: %.1f, heading_out: %.1f, L: %.1f, R: %.1f\n",
           get_absolute_heading(), heading_error, drive_output, heading_output, left_voltage, right_voltage);

    delay(10);
  }
}

/**
 * Turns to a given angle with only one side of the drivetrain.
 * Like turn_to_angle(), is optimized for turning the shorter
 * direction.
 * 
 * @param angle Desired angle in degrees.
 */

void Drive::swing_to_angle(float angle, bool move_left, bool motion_chaining){
  PID swingPID(reduce_negative_180_to_180(angle - get_absolute_heading()), swing_kp, swing_ki, swing_kd, swing_starti, swing_settle_error, swing_settle_time, swing_timeout);
  while(swingPID.is_settled() == false){
    float error = reduce_negative_180_to_180(angle - get_absolute_heading());
    if(motion_chaining && fabs(error) < motion_chain_turn_early_exit_range){
      break;
    }

    float output = swingPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);

    if(motion_chaining){
      output = clamp_min_voltage(output, motion_chain_turn_min_voltage);
    }

    if(move_left){
      DriveL.move(output);
      brake_with_mode_group(DriveR, MotorBrake::hold);
    }
    else{
      DriveR.move(-output);
      brake_with_mode_group(DriveL, MotorBrake::hold);
    }
    
    delay(10);
  }
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::get_ForwardTracker_position(){
  if(drive_style == DriveStyle::ZERO_TRACKER || 
     drive_style == DriveStyle::TANK_ONE_SIDEWAYS_ROTATION){
    // return DriveR.get_position() * drive_in_to_deg_ratio;
    return get_right_position_in();
  }
  
  return Fwd_tracker.get_position()*ForwardTracker_in_to_deg_ratio;
}

/**
 * Depending on the drive style, gets the tracker's position.
 * 
 * @return The tracker position.
 */

float Drive::get_SidewaysTracker_position(){
  if(drive_style == DriveStyle::ZERO_TRACKER || 
     drive_style == DriveStyle::TANK_ONE_FORWARD_ROTATION){
    return 0;
  }
  
  return Sideways_tracker.get_position()*SidewaysTracker_in_to_deg_ratio;
}

void Drive::wall_distance(WallSide direction, float distance, float heading, float wall_dis_target, float _drive_min_voltage){
  PID drivePID(distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(reduce_negative_180_to_180(heading - get_absolute_heading()), heading_kp, heading_ki, heading_kd, heading_starti, turn_settle_error, turn_settle_time, turn_timeout);
  PID wall_PID(wall_dis_target, wall_kp, wall_ki, wall_kd, wall_starti);
  float start_average_position = (get_left_position_in()+get_right_position_in())/2.0;
  float average_position = start_average_position;

  drivePID.settle_time=10; //

  int rev_constant=1;
  if(distance<0) rev_constant =-1;
  // Wait for heading to settle too, not just distance -- otherwise the move
  // can end right as the wall correction is mid-swing, leaving the robot
  // still turned away from the target heading.
  while(!drivePID.is_settled() || !headingPID.is_settled()){
    average_position = (get_left_position_in()+get_right_position_in())/2.0;
    drive_error = distance+start_average_position-average_position;
    float heading_error = reduce_negative_180_to_180(heading - get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);
    float heading_output = headingPID.compute(heading_error);

    float wall_distance_error = 0;
    if(direction == WallSide::LEFT){
      wall_distance_error = wall_dis_target - distance_sensorL.get();
    } else {
      wall_distance_error = wall_dis_target - distance_sensorR.get();
    }

    // if(fabs(wall_distance_error)>200) wall_distance_error=0;
    float wall_dist_output = wall_PID.compute(wall_distance_error);

    if(direction == WallSide::RIGHT){
      wall_dist_output = -wall_dist_output;
    }

    drive_output = clamp(drive_output, -drive_max_voltage, drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);
    wall_dist_output = clamp(wall_dist_output, -wall_max_voltage, wall_max_voltage);

    clamp_min_voltage(drive_output, _drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output+wall_dist_output*rev_constant), right_voltage_scaling(drive_output, heading_output+wall_dist_output*rev_constant));
    // drive_with_voltage(drive_output+heading_output+wall_dist_output*rev_constant, drive_output-heading_output-wall_dist_output*rev_constant);
    delay(10);
  }
  // drive_settle_time=150;
  // drive_max_voltage=drive_min_voltage;
}

/**
 * Background task for updating the odometry.
 */

void Drive::position_track(){
  while(1){
    odom.update_position(get_ForwardTracker_position(), get_SidewaysTracker_position(), get_absolute_heading());
    delay(5);
  }
}

/**
 * Resets the robot's heading.
 * For example, at the beginning of auton, if your robot starts at
 * 45 degrees, so set_heading(45) and the robot will know which way 
 * it's facing.
 * 
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_heading(float orientation_deg){
  Gyro.set_heading(orientation_deg*gyro_scale/360.0);
}

/**
 * MUST BE CALLED TO START THE ODOMETRY TASK. 
 * 
 * Resets the robot's coordinates and heading.
 * This is for odom-using robots to specify where the bot is at the beginning
 * of the match.
 * 
 * @param X_position Robot's x in inches.
 * @param Y_position Robot's y in inches.
 * @param orientation_deg Desired heading in degrees.
 */

void Drive::set_coordinates(float X_position, float Y_position, float orientation_deg){
  odom.set_position(X_position, Y_position, orientation_deg, get_ForwardTracker_position(), get_SidewaysTracker_position());
  set_heading(orientation_deg);

  if (odom_task != nullptr) { // is this if() even necessary
    odom_task->suspend();    // stop task
    delete odom_task;      // free memory
  }
  odom_task = new Task(position_track_task);

  chassis_lemlib.setPose(X_position, Y_position, to_rad(orientation_deg));
}

/**
 * Gets the robot's x.
 * 
 * @return The robot's x position in inches.
 */

float Drive::get_X_position(){
  return(odom.X_position);
}

/**
 * Gets the robot's y.
 * 
 * @return The robot's y position in inches.
 */

float Drive::get_Y_position(){
  return(odom.Y_position);
}

/**
 * Drives to a specified point on the field.
 * Uses the double-PID method, with one for driving and one for heading correction.
 * The drive error is the euclidean distance to the desired point, and the heading error
 * is the turn correction from the current heading to the desired point. Uses optimizations
 * like driving backwards whenever possible and scaling the drive output with the cosine
 * of the angle to the point.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 */

void Drive::drive_to_point(float X_position, float Y_position){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_point(X_position, Y_position, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_point(float X_position, float Y_position, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  PID drivePID(hypot(X_position-get_X_position(),Y_position-get_Y_position()), drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  float start_angle_deg = to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()));
  PID headingPID(start_angle_deg-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = false;
  bool prev_line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
  // pros::screen::print(TEXT_MEDIUM, 8, "drive_error: %.2f", drive_error);
  // pros::screen::print(TEXT_MEDIUM, 10, "drivePID.is_settled(): %d", drivePID.is_settled());
  while(!drivePID.is_settled()){
    line_settled = is_line_settled(X_position, Y_position, start_angle_deg, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    drive_error = hypot(X_position-get_X_position(),Y_position-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading());
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);
    
    if (drive_error<drive_settle_error) { heading_output = 0; }

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    delay(10);

    // pros::screen::print(TEXT_MEDIUM, 8, "drive_error: %.2f", drive_error);
    // pros::screen::print(TEXT_MEDIUM, 9, "drive_output clamped: %.2f", drive_output);
  }
}

/**
 * Drives to a specified point and orientation on the field.
 * Uses a boomerang controller. The carrot point is back from the target
 * by the same distance as the robot's distance to the target, times the lead. The
 * robot always tries to go to the carrot, which is constantly moving, and the
 * robot eventually gets into position. The heading correction is optimized to only
 * try to reach the correct angle when drive error is low, and the robot will drive 
 * backwards to reach a pose if it's faster. .5 is a reasonable value for the lead. 
 * The setback parameter is used to glide into position more effectively. It is
 * the distance back from the target that the robot tries to drive to first.
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param angle Desired orientation in degrees.
 * @param lead Constant scale factor that determines how far away the carrot point is. 
 * @param setback Distance in inches from target by which the carrot is always pushed back.
 * @param drive_min_voltage Minimum voltage on the drive, used for chaining movements.
 */

void Drive::drive_to_pose(float X_position, float Y_position, float angle){
  drive_to_pose(X_position, Y_position, angle, boomerang_lead, boomerang_setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}


void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout){
  drive_to_pose(X_position, Y_position, angle, lead, setback, drive_min_voltage, drive_max_voltage, heading_max_voltage, drive_settle_error, drive_settle_time, drive_timeout, drive_kp, drive_ki, drive_kd, drive_starti, heading_kp, heading_ki, heading_kd, heading_starti);
}

void Drive::drive_to_pose(float X_position, float Y_position, float angle, float lead, float setback, float drive_min_voltage, float drive_max_voltage, float heading_max_voltage, float drive_settle_error, float drive_settle_time, float drive_timeout, float drive_kp, float drive_ki, float drive_kd, float drive_starti, float heading_kp, float heading_ki, float heading_kd, float heading_starti){
  float target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());
  PID drivePID(target_distance, drive_kp, drive_ki, drive_kd, drive_starti, drive_settle_error, drive_settle_time, drive_timeout);
  PID headingPID(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position()))-get_absolute_heading(), heading_kp, heading_ki, heading_kd, heading_starti);
  bool line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool prev_line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
  bool crossed_center_line = false;
  bool center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
  bool prev_center_line_side = center_line_side;
  while(!drivePID.is_settled()){
    line_settled = is_line_settled(X_position, Y_position, angle, get_X_position(), get_Y_position());
    if(line_settled && !prev_line_settled){ break; }
    prev_line_settled = line_settled;

    center_line_side = is_line_settled(X_position, Y_position, angle+90, get_X_position(), get_Y_position());
    if(center_line_side != prev_center_line_side){
      crossed_center_line = true;
    }

    target_distance = hypot(X_position-get_X_position(),Y_position-get_Y_position());

    float carrot_X = X_position - sin(to_rad(angle)) * (lead * target_distance + setback);
    float carrot_Y = Y_position - cos(to_rad(angle)) * (lead * target_distance + setback);

    drive_error = hypot(carrot_X-get_X_position(),carrot_Y-get_Y_position());
    float heading_error = reduce_negative_180_to_180(to_deg(atan2(carrot_X-get_X_position(),carrot_Y-get_Y_position()))-get_absolute_heading());

    if (drive_error<drive_settle_error || crossed_center_line || drive_error < setback) { 
      heading_error = reduce_negative_180_to_180(angle-get_absolute_heading()); 
      drive_error = target_distance;
    }
    
    float drive_output = drivePID.compute(drive_error);

    float heading_scale_factor = cos(to_rad(heading_error));
    drive_output*=heading_scale_factor;
    heading_error = reduce_negative_90_to_90(heading_error);
    float heading_output = headingPID.compute(heading_error);

    drive_output = clamp(drive_output, -fabs(heading_scale_factor)*drive_max_voltage, fabs(heading_scale_factor)*drive_max_voltage);
    heading_output = clamp(heading_output, -heading_max_voltage, heading_max_voltage);

    drive_output = clamp_min_voltage(drive_output, drive_min_voltage);

    drive_with_voltage(left_voltage_scaling(drive_output, heading_output), right_voltage_scaling(drive_output, heading_output));
    delay(10);
  }
}

/**
 * Turns to a specified point on the field.
 * Functions similarly to turn_to_angle() except with a point. The
 * extra_angle_deg parameter turns the robot extra relative to the 
 * desired target. For example, if you want the back of your robot
 * to point at (36, 42), you would run turn_to_point(36, 42, 180).
 * 
 * @param X_position Desired x position in inches.
 * @param Y_position Desired y position in inches.
 * @param extra_angle_deg Angle turned past the desired heading in degrees.
 */

void Drive::turn_to_point(float X_position, float Y_position){
  turn_to_point(X_position, Y_position, 0, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout){
  turn_to_point(X_position, Y_position, extra_angle_deg, turn_max_voltage, turn_settle_error, turn_settle_time, turn_timeout, turn_kp, turn_ki, turn_kd, turn_starti);
}

void Drive::turn_to_point(float X_position, float Y_position, float extra_angle_deg, float turn_max_voltage, float turn_settle_error, float turn_settle_time, float turn_timeout, float turn_kp, float turn_ki, float turn_kd, float turn_starti){
  PID turnPID(reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading()), turn_kp, turn_ki, turn_kd, turn_starti, turn_settle_error, turn_settle_time, turn_timeout);
  while(turnPID.is_settled() == false){
    float error = reduce_negative_180_to_180(to_deg(atan2(X_position-get_X_position(),Y_position-get_Y_position())) - get_absolute_heading() + extra_angle_deg);
    float output = turnPID.compute(error);
    output = clamp(output, -turn_max_voltage, turn_max_voltage);
    drive_with_voltage(output, -output);
    delay(10);
  }
}



// The cascade extend limit (CASCADE_EXTEND_LIMIT_DEG) and the "arrived"
// tolerance (CASCADE_SETTLE_ERROR_DEG) now live in Template/cascade.h, so the
// buttons below and the cascade controller share one copy of each number
// instead of drifting apart.
// 中文：cascade 的行程上限與「到位」容差搬到 Template/cascade.h，讓下面的按鍵跟
// cascade 控制器共用同一份數字，不會兩邊各改各的。

// RIGHT/LEFT button cascade targets (paired with the arm going to POS_1/POS_2
// respectively). Change these values to retarget.
int CASCADE_PRESET_DEG = 545;       // RIGHT -> ArmPosition::POS_1, first cascade move
int CASCADE_PRESET_2_DEG = 595;     // LEFT  -> ArmPosition::POS_3, first cascade move
int CASCADE_RIGHT_FINAL_DEG = 250;  // RIGHT -> cascade's 2nd move, once the arm settles
int CASCADE_LEFT_FINAL_DEG = 0;     // LEFT  -> cascade's 2nd move, once the arm reaches POS_3

// How long a preset sequence waits on one step before giving up and moving on.
const int PRESET_STEP_TIMEOUT_MS = 3000;
// arm_settled is recomputed by arm_task() every 10ms, so it stays stale (true
// for the *previous* target) briefly after arm_set_position() -- wait this long
// before trusting it.
const int ARM_SETTLE_LATENCY_MS = 30;

// True while a RIGHT/LEFT preset sequence owns the cascade. Cleared as soon as
// the driver takes manual control with L1/L2, which also tells a running
// sequence task to abort instead of fighting the driver. File scope (not local
// to control_arcade) so those tasks can see it.
static bool cascade_preset_active = false;

// Bumped every time something new takes over the arm: another preset press, or
// DOWN. A running sequence task holds the id it started with and gives up as
// soon as it's been superseded, so a later button press can't be undone by an
// older sequence still working through its steps.
static int preset_sequence_id = 0;

// A sequence keeps going only while it still owns the cascade (driver hasn't
// grabbed L1/L2) and hasn't been superseded by a newer press.
static bool preset_still_owns(int seq_id){
  return cascade_preset_active && seq_id == preset_sequence_id;
}

// Give up on the whole preset sequence and hand the cascade back to the driver.
// The buzz on the controller is the only way the driver finds out -- without it
// the robot just quietly stops halfway through and looks broken.
// 中文：整串預設動作放棄，把 cascade 還給駕駛。遙控器震一下是駕駛唯一會知道的方式
// ——不震的話，機器人只是安靜地停在半路，看起來像壞掉。
static void preset_abort(){
  cascade_preset_active = false;
  // Park the cascade where it actually is. We just proved it cannot reach the
  // target it was given, so leaving that target in place would have the PID
  // leaning on it forever -- and once kP is tuned up, "leaning forever" means a
  // stalled motor pulling stall current until something gives.
  // 中文：把目標設回它現在的位置。剛剛已經證明它到不了原本的目標，還留著那個目標的話
  // PID 會一直死推——kP 調高之後，「一直死推」就是馬達堵轉、電流一路吃到燒東西。
  cascade_set_target(cascade_get_position_deg());
  // Drive::master is private and this is a free function, so buzz the master
  // controller through the plain C API -- same controller, no class changes.
  // 中文：Drive 裡的 master 是 private、這裡又是自由函式，所以改用 C 版 API 讓同一
  // 支遙控器震動，不用去動類別。
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "-");
}

// Wait for the cascade to reach target_deg. Returns false if the sequence was
// cancelled OR if the cascade did not get there in time, so the caller stops.
//
// Timing out is NOT "close enough, carry on". The next step of the LEFT
// sequence lowers the arm to POS_2, which is only safe once the cascade is
// actually retracted -- running it against a cascade stuck halfway is how the
// arm and the cascade hit each other. So a timeout aborts the sequence and
// buzzes the controller instead of pressing on.
// 中文：等 cascade 到 target_deg。被取消或「等不到」都回 false，呼叫端就會停手。
// 逾時不等於「差不多了、繼續吧」：LEFT 序列的下一步是把手臂放到 POS_2，那一步只有
// 在 cascade 真的收回來之後才安全，對著卡在半路的 cascade 放手臂就是兩個機構互撞。
// 所以逾時＝中止整串動作＋遙控器震一下，不硬做下去。
static bool cascade_wait_settled(int seq_id, int target_deg){
  int waited_ms = 0;
  while(fabs(cascade_get_position_deg() - target_deg) > CASCADE_SETTLE_ERROR_DEG){
    if(!preset_still_owns(seq_id)) return false;
    delay(10);
    waited_ms += 10;
    if(waited_ms > PRESET_STEP_TIMEOUT_MS){
      preset_abort();
      return false;
    }
  }
  return preset_still_owns(seq_id);
}

// Wait for the arm to reach whatever target was last set with
// arm_set_position(). Same contract as cascade_wait_settled(), timeout included:
// in the LEFT sequence the step after this one retracts the cascade to 0, and
// doing that while the arm is stuck halfway up is the same two mechanisms
// hitting each other from the other direction. So a timeout aborts too.
// 中文：等手臂到位，規則跟 cascade_wait_settled() 一樣，逾時也一樣。LEFT 序列的下一步
// 是把 cascade 收回 0，手臂卡在半路時做那一步，就是同樣兩個機構從另一邊互撞——所以
// 等不到手臂也要中止。
static bool arm_wait_settled(int seq_id){
  delay(ARM_SETTLE_LATENCY_MS);
  int waited_ms = 0;
  while(!arm_settled){
    if(!preset_still_owns(seq_id)) return false;
    delay(10);
    waited_ms += 10;
    if(waited_ms > PRESET_STEP_TIMEOUT_MS){
      preset_abort();
      return false;
    }
  }
  return preset_still_owns(seq_id);
}

/**
 * Controls a chassis with left stick throttle and right stick turning.
 * Default deadband is 5.
 */

void Drive::control_arcade(){
  double throttle = 0;
  double turn = 0;
  bool bt_y=false , last_bt_y=false, bumper_bt_y=false;
  bool bt_a=false , last_bt_a=false, bumper_bt_a=false;
  bool bt_b=false , last_bt_b=false, bumper_bt_b=false;
  bool bt_Right=false , last_bt_Right=false, bumper_bt_Right=false;
  bool bt_R2=false , last_bt_R2=false, bumper_bt_R2=false;
  bool bt_L2=false , last_bt_L2=false, bumper_bt_L2=false;
  bool bt_x=false , last_bt_x=false, bumper_bt_x=false;
  bool bt_up=false , last_bt_up=false, bumper_bt_up=false;
  bool bt_down=false , last_bt_down=false, bumper_bt_down=false;
  bool bt_Left=false , last_bt_Left=false;
  bool outtake_wait = false;
  cascade_preset_active = false;
  // Task in_fxn(intake_status);
  chassis.drive_stop(MotorBrake::coast);

  // Cascade starts at position 0 and is never allowed to go below it.
  cascade1.tare_position();
  cascade2.tare_position();
  cascade_notify_tare();

  // Hand the cascade over to its PID controller for driver control. It holds
  // position 0 (right here, where we just tared) until a button says otherwise.
  // Autonomous turns this back off so its move_absolute() moves still work.
  // 中文：把 cascade 交給它的 PID 控制器（遙控期間才開）。它會先撐在剛剛歸零的
  // 位置 0，直到有按鍵叫它去別的地方。自走會把它關掉，所以自走照舊。
  cascade_set_target(0);
  cascade_control_set_enabled(true);

  while(1){
  throttle = master.get_analog(ANALOG_LEFT_Y);
  turn = master.get_analog(ANALOG_RIGHT_X);
  
  
  //deadband
  if(fabs(throttle)<5){
    throttle = 0;
  }
  if(fabs(turn)<5){
    turn = 0;
  }

  DriveL.move(throttle + turn);
  DriveR.move(throttle - turn);

  // Brake mode is deliberately sticky when both sticks are back at center:
  // letting go of turn should hold the heading you just turned to (stays
  // BRAKE), while letting go of throttle should let the chassis roll to a
  // stop (stays COAST) -- whichever axis was last active wins and persists
  // until the other axis takes over.
  if(fabs(turn)>5){
    chassis.DriveL.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.DriveR.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
  }
  else if(fabs(throttle)>5){
    chassis.DriveL.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    chassis.DriveR.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  }
  
    //set bt
      bt_a = master.get_digital(DIGITAL_A);
    if(!bt_a and last_bt_a){
      bumper_bt_a = !bumper_bt_a;
      }
    last_bt_a = bt_a;
    bt_b = master.get_digital(DIGITAL_B);
    if(!bt_b and last_bt_b){
      bumper_bt_b = !bumper_bt_b;
      }
    last_bt_b = bt_b;
    // bt_Right = master.get_digital(DIGITAL_RIGHT);
    // if(!bt_Right and last_bt_Right){
    //   bumper_bt_Right = !bumper_bt_Right;
    //   }
    // bt_R2 = master.get_digital(DIGITAL_R2);
    // if(!bt_R2 and last_bt_R2){
    //   bumper_bt_R2 = !bumper_bt_R2;
    //   }
    // bt_L2 = master.get_digital(DIGITAL_L2);
    // if(!bt_L2 and last_bt_L2){
    //   bumper_bt_L2 = !bumper_bt_L2;
    //   }
    bt_x = master.get_digital(DIGITAL_X);
    if(!bt_x and last_bt_x){
      bumper_bt_x = !bumper_bt_x;
      }
    last_bt_x = bt_x;
    // bt_up = master.get_digital(DIGITAL_UP);
    // if(!bt_up and last_bt_up){
    //   bumper_bt_up = !bumper_bt_up;
    //   }
    // bt_down = master.get_digital(DIGITAL_DOWN);
    // if(!bt_down and last_bt_down){
    //   bumper_bt_down = !bumper_bt_down;
    //   }

    //bt
    // claw.set_value(bt_a);
    // last_bt_a = bt_a;

    claw.set_value(bumper_bt_b);
    toggle.set_value(bumper_bt_x);

    // Cascade limit switch: this one reads 1 when pressed, 0 when not
    // pressed. Every time it's triggered, re-zero both cascade encoders
    // so the physical hard stop is always "0 degrees" -- this corrects any
    // encoder drift picked up over the match, and the existing L2 (retract)
    // check below (cascade1.get_position() <= 0) will now also stop the
    // motors right at the switch instead of relying on drifted encoder math.
    if(cascade_limit.get_value() == 1){
      cascade1.tare_position();
      cascade2.tare_position();
      // The controller's target is in the same (just re-zeroed) frame, so
      // nothing needs re-aiming -- only the PID's memory of the previous error
      // has to be cleared, or the position jump reads as a fake spike.
      // 中文：控制器的目標跟編碼器是同一套座標，歸零後不用重設目標；只要把 PID
      // 對「上一次誤差」的記憶清掉，位置突跳才不會被當成真的誤差。
      cascade_notify_tare();
    }

    //intake
    if(master.get_digital(DIGITAL_R1)){
      intake.move(127);
    }
    else if(master.get_digital(DIGITAL_R2)){
      intake.move(-127);
    }
    else if (master.get_digital(DIGITAL_L1)){
      // L1 = jog up, unchanged: same button, same voltage, same limit check.
      // cascade_jog() just routes it through the controller (which stands
      // aside while jogging) instead of writing to the motors here.
      // 中文：L1＝往上點動，行為沒變（同一顆按鍵、同樣的電壓、同樣的上限判斷）。
      // 只是改成走 cascade_jog()，點動期間控制器會讓位，不是在這裡直接寫馬達。
      cascade_preset_active = false;
      if(cascade1.get_position() >= CASCADE_EXTEND_LIMIT_DEG || cascade2.get_position() >= CASCADE_EXTEND_LIMIT_DEG){
        cascade_jog(0);
      }
      else{
        cascade_jog(100);
      }
    }
    else if(master.get_digital(DIGITAL_L2)){
      // L2 = jog down, unchanged in the same way as L1 above.
      // 中文：L2＝往下點動，同上，行為沒變。
      cascade_preset_active = false;
      if(cascade1.get_position() <= 0 || cascade2.get_position() <= 0){
        cascade_jog(0);
      }
      else{
        cascade_jog(-97);
      }
    }
    else{
      intake.move(0);
      // Nobody is jogging: hand the cascade back to the PID. If a preset
      // sequence is running it holds that sequence's target; otherwise it holds
      // wherever the driver let go of L1/L2 -- which is the one thing that IS
      // new here. It used to be sent 0 and sag down under its own weight.
      // 中文：沒人在點動，就把 cascade 交還給 PID。有預設動作在跑就撐在那個目標，
      // 否則就停在駕駛放開 L1/L2 的那一格——這是唯一真正變新的地方：以前放開後是
      // 送 0，機構會自己往下沉。
      cascade_jog_stop();
    }

    bt_Right = master.get_digital(DIGITAL_RIGHT);
    if(bt_Right and !last_bt_Right){
      cascade_preset_active = true;
      int right_seq_id = ++preset_sequence_id;
      // Same target height as before -- it is now handed to the cascade PID
      // instead of the motors' built-in move_absolute().
      // 中文：目標高度沿用原本的數字，只是改成交給 cascade 的 PID，不再用馬達內建
      // 的 move_absolute()。
      cascade_set_target(CASCADE_PRESET_DEG);
      arm_set_position(ArmPosition::POS_1);
      claw.set_value(false);
      // Runs on its own task so waiting for the arm doesn't block the rest
      // of control_arcade() (drive, intake, other buttons).
      pros::Task([right_seq_id]{
        if(!arm_wait_settled(right_seq_id)) return;
        cascade_set_target(CASCADE_RIGHT_FINAL_DEG);
      });
    }
    last_bt_Right = bt_Right;

    bt_down = master.get_digital(DIGITAL_DOWN);
    if(bt_down and !last_bt_down){
      // Cancels any preset sequence still stepping through its moves, so it
      // can't send the arm back up after this. The cascade keeps holding
      // wherever it is; only the arm comes down.
      ++preset_sequence_id;
      arm_set_position(ArmPosition::DOWN);
    }
    last_bt_down = bt_down;

    bt_Left = master.get_digital(DIGITAL_LEFT);
    if(bt_Left and !last_bt_Left){
      cascade_preset_active = true;
      int left_seq_id = ++preset_sequence_id;
      cascade_set_target(CASCADE_PRESET_2_DEG);
      // Run on its own task so the waits between steps don't block the rest
      // of control_arcade() (drive, intake, other buttons). Each wait bails
      // out if the driver takes the cascade back over with L1/L2, or presses
      // DOWN / another preset.
      pros::Task([left_seq_id]{
        // 1. cascade is already heading to CASCADE_PRESET_2_DEG; give it 100ms
        //    to start moving, then 2. raise the arm to POS_3.
        pros::delay(100);
        if(!preset_still_owns(left_seq_id)) return;
        arm_set_position(ArmPosition::POS_3);
        if(!arm_wait_settled(left_seq_id)) return;

        // 3. retract the cascade all the way back to CASCADE_LEFT_FINAL_DEG.
        cascade_set_target(CASCADE_LEFT_FINAL_DEG);
        if(!cascade_wait_settled(left_seq_id, CASCADE_LEFT_FINAL_DEG)) return;

        // 4. rotate the arm down to POS_2, cascade holds where it is -- the
        //    cascade PID keeps holding that target for as long as nobody sets
        //    a new one or grabs L1/L2.
        arm_set_position(ArmPosition::POS_2);
      });
    }
    last_bt_Left = bt_Left;
  }
}

// /**
//  * Controls a chassis with left stick throttle and strafe, and right stick turning.
//  * Default deadband is 5.
//  */

// void Drive::control_holonomic(){
//   float throttle = deadband(controller(primary).Axis3.value(), 5);
//   float turn = deadband(controller(primary).Axis1.value(), 5);
//   float strafe = deadband(controller(primary).Axis4.value(), 5);
//   DriveLF.move(fwd, to_volt(throttle+turn+strafe), volt);
//   DriveRF.move(fwd, to_volt(throttle-turn-strafe), volt);
//   DriveLB.move(fwd, to_volt(throttle+turn-strafe), volt);
//   DriveRB.move(fwd, to_volt(throttle-turn+strafe), volt);
// }

// /**
//  * Controls a chassis with left stick left drive and right stick right drive.
//  * Default deadband is 5.
//  */

// void Drive::control_tank(){
//   float leftthrottle = deadband(controller(primary).Axis3.value(), 5);
//   float rightthrottle = deadband(controller(primary).Axis2.value(), 5);
//   DriveL.move(fwd, to_volt(leftthrottle), volt);
//   DriveR.move(fwd, to_volt(rightthrottle), volt);
// }

/**
 * Tracking task to run in the background.
 */

int Drive::position_track_task(){
  chassis.position_track();
  return(0);
}