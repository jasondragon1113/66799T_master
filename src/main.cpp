#include "main.h"
#include "vexdash_pros/vexdash_pros.h" // vexdash car-side telemetry library

Task* intake_task = nullptr;

void initialize() {
	// Load real PID constants first so the dashboard sliders start at the
	// values your code actually uses (not 0).
	default_constants();

	// --- vexdash: tunable PID gains (sliders on the web Config panel, auto write-back) ---
	vexdash::watch_config("kP", &chassis.drive_kp, "drive/pid");
	vexdash::watch_config("kI", &chassis.drive_ki, "drive/pid");
	vexdash::watch_config("kD", &chassis.drive_kd, "drive/pid");
	vexdash::watch_config("kP", &chassis.turn_kp, "turn/pid");
	vexdash::watch_config("kI", &chassis.turn_ki, "turn/pid");
	vexdash::watch_config("kD", &chassis.turn_kd, "turn/pid");

	// --- vexdash: live graph channels (streamed to the web Graph panel) ---
	vexdash::watch("drive_error",  &chassis.drive_error,       "in");
	vexdash::watch("drive_target", &chassis.tele_drive_target, "in");
	vexdash::watch("drive_output", &chassis.tele_drive_output, "V");
	vexdash::watch("turn_error",   &chassis.tele_turn_error,   "deg");
	vexdash::watch("turn_target",  &chassis.tele_turn_target,  "deg");
	vexdash::watch("turn_output",  &chassis.tele_turn_output,  "V");

	// Start vexdash over USB. HUD off because this project draws its own
	// Brain-screen dashboard (start_dashboard); the vexdash HUD would fight it.
	vexdash::init_usb(nullptr, false);

	start_dashboard();
}

void disabled() {
	// intake_state = IntakeTask::STOP;
	delay(1000);
	inertial.tare_euler(); // idk the difference between this and inertial.tare(). Both works. Does not work if called in competition_initialize() or disabled() for some reason. 
}

void competition_initialize() {
	init();

	delay(2250);
	inertial.tare_euler(); // idk the difference between this and inertial.tare(). Both works. Does not work if called in competition_initialize() or disabled() for some reason.
}

ASSET(curveLeft_txt);

void autonomous() {
	chassis.set_coordinates(0, 0, 0);
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
	while (true) {
	chassis.control_arcade();
	delay(10);
	}
}