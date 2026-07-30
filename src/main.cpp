#include "main.h"
#include "vexdash_pros/vexdash_pros.h" // vexdash car-side telemetry library

Task* intake_task = nullptr;

// vexdash PID-test controls: set target on the dashboard, flip the toggle to run.
double test_distance = 24.0;   // inches  — target for the drive PID test
double test_angle    = 90.0;   // degrees — target for the turn PID test
bool   run_drive_test = false; // toggle ON in dashboard to drive test_distance
bool   run_turn_test  = false; // toggle ON in dashboard to turn to test_angle

void initialize() {
	// Load real PID constants first so the dashboard sliders start at the
	// values your code actually uses (not 0).
	default_constants();

	// --- vexdash: tunable PID gains (sliders on the web Config panel, auto write-back) ---
	//
	// EVERY slider needs a name that is unique across the WHOLE robot, not just
	// unique inside its group. The registry de-duplicates on the NAME alone --
	// the group string is only a display path (see watch_registry.h: "same name,
	// same kind -> the later one overwrites the earlier one"). Four groups all
	// calling their gain "kP" would collapse into ONE slider wired to whichever
	// mechanism registered last, and dragging it would silently retune that one
	// mechanism instead of the one you are looking at.
	// 中文：每一顆滑桿的名字要在「整台車」是唯一的，不是「在自己那組裡」唯一就好。
	// 登記表只用名字去重，群組只是顯示路徑（watch_registry.h 白紙黑字寫「同名同類
	// 後者覆蓋前者」）。四組都叫 "kP" 的話會塌成一顆滑桿、綁到最後登記的那個機構，
	// 你以為在調底盤，其實在調別的東西——所以下面全部加機構前綴。
	vexdash::watch_config("drive_kP", &chassis.drive_kp, "drive/pid");
	vexdash::watch_config("drive_kI", &chassis.drive_ki, "drive/pid");
	vexdash::watch_config("drive_kD", &chassis.drive_kd, "drive/pid");
	vexdash::watch_config("turn_kP",  &chassis.turn_kp,  "turn/pid");
	vexdash::watch_config("turn_kI",  &chassis.turn_ki,  "turn/pid");
	vexdash::watch_config("turn_kD",  &chassis.turn_kd,  "turn/pid");

	// --- vexdash: live graph channels (streamed to the web Graph panel) ---
	vexdash::watch("drive_error",  &chassis.drive_error,       "in");
	vexdash::watch("drive_target", &chassis.tele_drive_target, "in");
	vexdash::watch("drive_output", &chassis.tele_drive_output, "V");
	vexdash::watch("turn_error",   &chassis.tele_turn_error,   "deg");
	vexdash::watch("turn_target",  &chassis.tele_turn_target,  "deg");
	vexdash::watch("turn_output",  &chassis.tele_turn_output,  "V");

	// --- vexdash: arm angle from the rotation sensor (use arm_angle to measure
	// the real ARM_POS_*_DEG values -- see arm.cpp) ---
	vexdash::watch("arm_angle",   &tele_arm_angle,  "deg");
	vexdash::watch("arm_target",  &tele_arm_target, "deg");
	vexdash::watch("arm_error",   &tele_arm_error,  "deg");
	vexdash::watch("arm_output",  &tele_arm_output, "V");
	vexdash::watch("arm_sensor_ok", &arm_sensor_ok);
	// How much of arm_output comes from the gravity feedforward (0 until ARM_KG
	// is tuned). The "arm/pid" path is a v1.4 CHANNEL_DEF field: it tells the
	// dashboard to show this line next to the arm/pid sliders instead of
	// guessing the grouping from the channel name.
	// 中文：arm_output 裡有多少是重力前饋給的（ARM_KG 沒調之前一直是 0）。後面的
	// "arm/pid" 是 v1.4 新增的分組欄位，明確告訴 dashboard 這條線跟 arm/pid 的
	// 滑桿是同一組，不用讓它自己猜。
	vexdash::watch("arm_ff", &tele_arm_ff, "V", -1, "arm/pid");

	// --- vexdash: live arm tuning (sliders on the Config panel, auto write-back) ---
	vexdash::watch_config("DOWN",  &ARM_DOWN_DEG,  "arm/presets");
	vexdash::watch_config("POS_1", &ARM_POS_1_DEG, "arm/presets");
	vexdash::watch_config("POS_2", &ARM_POS_2_DEG, "arm/presets");
	vexdash::watch_config("POS_3", &ARM_POS_3_DEG, "arm/presets");
	vexdash::watch_config("arm_kP", &ARM_KP,       "arm/pid");
	vexdash::watch_config("arm_kI", &ARM_KI,       "arm/pid");
	vexdash::watch_config("arm_kD", &ARM_KD,       "arm/pid");
	// Gravity feedforward. arm_kG = the voltage that just holds the arm still
	// when it is HORIZONTAL; arm_horizontal_deg = the arm_angle at that pose.
	// Both default to 0, which means "feedforward off" -- same as before.
	// 中文：重力前饋。kG＝手臂放水平時剛好撐住不掉的電壓；horizontal_deg＝那一刻
	// arm_angle 讀到的角度。兩個都預設 0＝不啟用，跟以前一樣。
	vexdash::watch_config("arm_kG", &ARM_KG,       "arm/pid");
	vexdash::watch_config("arm_horizontal_deg", &ARM_HORIZONTAL_DEG, "arm/pid");

	// --- vexdash: cascade (the lift on ports 7 / -2) live graph + tuning ---
	// The "cascade/pid" path is the v1.4 CHANNEL_DEF grouping field, so these
	// lines land next to the cascade sliders on the dashboard.
	// 中文：cascade（7 與 -2 那支升降）的即時圖表與調參。"cascade/pid" 是 v1.4 的
	// 分組欄位，會讓這幾條線跟 cascade 的滑桿排在一起。
	vexdash::watch("cascade_pos",    &tele_cascade_pos,    "deg", -1, "cascade/pid");
	vexdash::watch("cascade_target", &tele_cascade_target, "deg", -1, "cascade/pid");
	vexdash::watch("cascade_error",  &tele_cascade_error,  "deg", -1, "cascade/pid");
	vexdash::watch("cascade_output", &tele_cascade_output, "V",   -1, "cascade/pid");
	vexdash::watch("cascade_ff",     &tele_cascade_ff,     "V",   -1, "cascade/pid");

	// Tune in this order: kG first (the voltage that makes the lift hover),
	// then kP, then kD, and kI only if it keeps stopping just short.
	// 中文：調參順序：先 kG（讓升降停在半空中不動的電壓），再 kP，再 kD，每次都差
	// 一點點才動 kI。
	vexdash::watch_config("cascade_kP", &CASCADE_KP, "cascade/pid");
	vexdash::watch_config("cascade_kI", &CASCADE_KI, "cascade/pid");
	vexdash::watch_config("cascade_kD", &CASCADE_KD, "cascade/pid");
	vexdash::watch_config("cascade_kG", &CASCADE_KG, "cascade/pid");

	// --- vexdash: on-demand PID tests (set the target, toggle "run", watch the Graph) ---
	vexdash::watch_config("test_distance", &test_distance,  "drive/test"); // inches
	vexdash::watch_config("run_drive",     &run_drive_test, "drive/test"); // toggle ON to drive
	vexdash::watch_config("test_angle",    &test_angle,     "turn/test");  // degrees
	vexdash::watch_config("run_turn",      &run_turn_test,  "turn/test");  // toggle ON to turn

	// --- vexdash: stream the port-5 motor (intake) onto the Graph: pos/rpm/temp/amp ---
	vexdash::watch_motor("motor", intake);

	// --- vexdash: Device Map -- sensors section ---
	vexdash::declare_device(distance_sensorL.get_port(), vexdash::DeviceType::kDistance, "distance_L");
	vexdash::declare_device(distance_sensorR.get_port(), vexdash::DeviceType::kDistance, "distance_R");
	vexdash::declare_device(arm_rotation.get_port(), vexdash::DeviceType::kRotation, "arm_rotation");

	// --- vexdash: Device Map -- motors section ---
	vexdash::declare_device(arm.get_port(), vexdash::DeviceType::kMotor, "arm");

	// Start vexdash over the ESP32 Smart Port bridge (port 11 @ 115200 baud).
	// The ESP32 relays telemetry to the dashboard over WiFi (ws://192.168.4.1).
	// NOTE: port 11 is also used by distance_sensorL in robot-config.cpp — a
	// smart port can host only one device, so move one of them if both are wired.
	// Smart Port path leaves stdout free (printf still works); HUD off by default.
	vexdash::init_smartport(11, 115200);

	// HOLD so the arm stays put under gravity when no button is pressed
	// (arm_task() would normally set this, but it's disabled above).
	arm.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	start_dashboard();
	start_arm_task();
	// The cascade controller starts DISABLED; control_arcade() switches it on
	// for driver control and autonomous() switches it back off.
	// 中文：cascade 控制器一開始是關著的，遙控 control_arcade() 才打開，自走再關掉。
	start_cascade_task();
}

void disabled() {
	// Field control has cut the robot off. Park the cascade controller too --
	// it must not be sitting there piling up integral error against a target it
	// is not allowed to drive to, or it would dump all of it into the motors the
	// instant the match resumes.
	// 中文：場地控制把機器人斷掉了，順手把 cascade 控制器也停掉。不然它會一直對著
	// 一個「現在不准去」的目標累積積分，等比賽恢復的瞬間全部倒進馬達。
	cascade_control_set_enabled(false);

	// intake_state = IntakeTask::STOP;
	delay(1000);
	inertial.tare_euler(); // idk the difference between this and inertial.tare(). Both works. Does not work if called in competition_initialize() or disabled() for some reason. 
}

void competition_initialize() {
	init();
	toggle.set_value(false);
	claw.set_value(true);
	delay(2250);
	inertial.tare_euler(); // idk the difference between this and inertial.tare(). Both works. Does not work if called in competition_initialize() or disabled() for some reason.
}

ASSET(curveLeft_txt);

void autonomous() {
	chassis.set_coordinates(0, 0, 0);

	// Cascade encoder "0" is only tared in Drive::control_arcade() (teleop),
	// so without this, ScoringLevel::LEVEL_0 in auton would target whatever
	// position the encoder happened to read at power-on -- not the true
	// physical bottom the robot is placed at before a match.
	cascade1.tare_position();
	cascade2.tare_position();
	cascade_notify_tare();

	// Autonomous drives the cascade with move_absolute() (see score() in
	// auton-routines.cpp), so the teleop cascade controller must stay out of the
	// way -- otherwise the two would fight over the same two motors. It is off
	// by default; this line matters when a match runs teleop before autonomous.
	// 中文：自走是用 move_absolute() 開 cascade（見 auton-routines.cpp 的 score()），
	// 所以遙控用的 cascade 控制器要退場，不然兩邊會搶同兩顆馬達。它本來就預設關著，
	// 這一行是為了「先跑過遙控再跑自走」的情況。
	cascade_control_set_enabled(false);

	//route: whichever is selected on the dashboard's Auton Select tab
	switch (selected_auton) {
		case AutonRoutine::left:
			left();
			break;
		case AutonRoutine::right:
			right();
			break;
		case AutonRoutine::sawp:
			sawp();
			break;
	}
}

void opcontrol() {
	default_constants();
	bool prev_drive = false, prev_turn = false;
	while (true) {
		// A rising edge on a dashboard toggle runs ONE PID test move. The
		// drive/turn PID channels stream to the Graph while the move runs.
		// While a toggle stays ON, normal arcade driving is paused; flip it
		// OFF to drive again, and OFF->ON to repeat the test.
		// if (run_drive_test && !prev_drive) {
		// 	chassis.drive_distance(test_distance);
		// } else if (run_turn_test && !prev_turn) {
		// 	chassis.turn_to_angle(test_angle);
		// } else if (!run_drive_test && !run_turn_test) {
			chassis.control_arcade();
		// }
		// prev_drive = run_drive_test;
		// prev_turn  = run_turn_test;
		delay(10);
	}
}