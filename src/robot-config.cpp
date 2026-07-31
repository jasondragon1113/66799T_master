#include "main.h"

IMU inertial(17);
// PLACEHOLDER TRACKERS -- the robot has neither of them. Their port numbers are
// made up and they DO collide with real devices (port 2 = distance_sensorL,
// port 1 = rightMiddle). That is known and harmless: the drive is configured as
// DriveStyle::ZERO_TRACKER below, so Drive never reads either object, and a
// Rotation constructor does not reconfigure a port that already holds a motor or
// a distance sensor -- it just never returns valid data, which nobody asks for.
// They are also deliberately left out of the vexdash device map in main.cpp.
// Do NOT "fix" the ports to free numbers: give them real unused ports and the
// next person will believe the robot has trackers. Delete them instead, if ever.
// 中文：這兩顆是假的 tracker，車上根本沒有。埠號是隨手填的，而且真的跟現有裝置撞到
// （埠 2＝distance_sensorL、埠 1＝rightMiddle）。這是已知且無害的：底盤下面設定成
// DriveStyle::ZERO_TRACKER，Drive 從頭到尾不會讀這兩個物件；而 Rotation 的建構也不
// 會把已經插著馬達／距離感測器的埠改掉，它只是永遠讀不到有效值——反正沒人讀。
// main.cpp 的 vexdash 孔位圖也故意不宣告它們。
// 不要把埠號「修」成沒人用的號碼：改成合法空埠，下一個人就會以為車上真的有 tracker。
// 真要處理就是整個刪掉。
Rotation fwd_tracker(2);       // fake: no forward tracker on this robot 中文：假的，沒有前向 tracker
Rotation sideways_tracker(1);  // fake: no sideways tracker on this robot 中文：假的，沒有側向 tracker

// 66799T Worlds
// negative port number means reversed (there is no separate "reversed" constructor argument)
Motor leftFront(-4, MotorGears::blue);
Motor leftMiddle(6, MotorGears::blue);
Motor leftBack(16, MotorGears::green);
Motor rightFront(15, MotorGears::blue);
Motor rightMiddle(-1, MotorGears::blue);
Motor rightBack(-8, MotorGears::green);

MotorGroup leftMotors({leftFront.get_port(), leftMiddle.get_port(), leftBack.get_port()});
MotorGroup rightMotors({rightFront.get_port(), rightMiddle.get_port(), rightBack.get_port()});

Motor intake(-5, MotorGears::green);
Motor cascade1(7, MotorGears::green);
Motor cascade2(-3, MotorGears::green);
Motor arm(18, MotorGears::green);

adi::DigitalOut claw('A');
adi::DigitalOut toggle('C');
adi::DigitalIn cascade_limit('D');

Distance distance_sensorL(2);
Distance distance_sensorR(20);
Rotation arm_rotation(21);



Drive chassis(

    // Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
    // You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

    // Drive Style (see drive.h for option list): 
    Drive::DriveStyle::ZERO_TRACKER,
    
    // Left Motors:
    leftMotors,

    // Right Motors:
    rightMotors,

    // Your inertial sensor:
    inertial,

    // Input your wheel diameter. (4" omnis are actually closer to 4.125"):
    2.75,

    // External ratio, must be in decimal, in the format of input teeth/output teeth.
    // If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
    // If the motor drives the wheel directly, this value is 1:
    // 48/36 is INTEGER division: both are ints, so this argument is literally 1
    // -- not 1.333. That is not a bug to fix, it is the number every autonomous
    // distance on this robot was tuned against. (And it explains the old TODO:
    // 36/48 is integer division too and evaluates to 0, which zeroes the odom
    // update entirely -- x/y stop moving, not "increment too slowly".) Writing
    // 48.0/36.0 would change the ratio to 1.333 and every auton drive_distance()
    // would overshoot by a third, so leave it alone unless you are prepared to
    // re-tune the whole auton.
    // 中文：48/36 是「整數除法」，兩邊都是 int，所以這個參數實際上就是 1，不是
    // 1.333。這不是待修的 bug，是全部自走距離都照著它調出來的既有值。（順帶解答舊
    // 的 TODO：36/48 同樣是整數除法、結果是 0，odom 會完全不動，不是「跑太慢」。）
    // 寫成 48.0/36.0 會讓比值變成 1.333，每一段自走 drive_distance() 都會多跑三分
    // 之一——除非你準備把整套自走重調，否則不要動它。
    48/36,

    // Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
    // For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
    362.5,

    // If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
    // If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
    fwd_tracker,

    // Input the Forward Tracker diameter (reverse it to make the direction switch)
    // For a tank drive using odom without a forward tracker, this value is useless and does not affect anything:
    2,

    // Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
    // For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
    // This distance is in inches:
    5.25,

    // Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
    sideways_tracker,

    // Sideways tracker diameter (reverse it to make the direction switch):
    0,

    // Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
    0
);


void default_constants(){
    // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI(, minVoltage)).
    chassis.set_drive_constants(127, 7, 0, 12.5, 0, 0);
    chassis.set_heading_constants(64, 1.5, 0, 8, 0); //chassis.set_heading_constants(64, 0.4, 0, 20, 0);      chassis.set_heading_constants(64, 1, 0, 2, 0);
    chassis.set_turn_constants(107, 3.2, .10583, 17.4625, 15.0); //chassis.set_turn_constants(107, 3.2, .10583, 17.4625, 15.0);
    chassis.set_swing_constants(127, 3.704166667, 0.08466667, 21.1666667, 15);
    chassis.set_wall_constants(74, 0.065, 0, 0, 0); //chassis.set_wall_constants(127, 0.529166667, 0, 0, 0);
    
    // Each exit condition set is in the form of (settle_error, settle_time, timeout).
    chassis.set_drive_exit_conditions(1.875, 45, 3000);
    chassis.set_turn_exit_conditions(1.9, 45, 2000);
    chassis.set_swing_exit_conditions(1.82, 45, 2000);
    
    // (min_voltage, early_exit_distance)
    chassis.set_drive_motion_chain_constants(60, 4);
    chassis.set_turn_motion_chain_constants(50, 5);
}

// LemLib Stuff vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv

lemlib::Drivetrain drivetrain(
    &leftMotors, // left motors
    &rightMotors, // right motors
    10.5, // track width in inches
    2.75, // wheel diameter in inches
    800, // rpm of the wheels
    2 // horizontal drift 
);

lemlib::OdomSensors sensors(
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2
    &inertial // inertial sensor
);

// lateral PID controller. Only slew is used in Pure Pursuit. 
lemlib::ControllerSettings lateral_controller(
    chassis.drive_kp, // proportional gain (kP)
    chassis.drive_ki, // integral gain (kI)
    chassis.drive_kd, // derivative gain (kD)
    chassis.drive_starti, // anti windup (= startI)
    1, // small error range, in inches
    100000000000, // small error range timeout, in milliseconds // lemlib default is 100
    chassis.drive_settle_error, // large error range, in inches
    chassis.drive_settle_time, // large error range timeout, in milliseconds 
    0 // maximum acceleration (slew) // TODO: tune this if necessary
);

// angular PID controller. Not used in Pure Pursuit. 
lemlib::ControllerSettings angular_controller(
    chassis.turn_kp, // proportional gain (kP)
    chassis.turn_ki, // integral gain (kI)
    chassis.turn_kd, // derivative gain (kD)
    chassis.turn_starti, // anti windup (= startI)
    1, // small error range, in degrees
    100000000000, // small error range timeout, in milliseconds // lemlib default is 100
    chassis.turn_settle_error, // large error range, in degrees
    chassis.turn_settle_time, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis_lemlib(
    drivetrain, // drivetrain settings
    lateral_controller, // lateral PID settings
    angular_controller, // angular PID settings
    sensors // odometry sensors
);

// LemLib Stuff ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

void init() {
    delay(2500); // wait for imu to calibrate
    // chassis_lemlib.calibrate(); // TODO: THIS MIGHT BE NEEDED FOR LEMLIB TO WORK! But if this is called, the DriveR.get_position() and DriveL.get_position() units become not degrees anymore for some reason, which breaks my odometry. 
    start_dashboard();
    start_arm_task();
    // static Task screen_task(map_task);
    // Controller(CONTROLLER_MASTER).rumble("..");
}