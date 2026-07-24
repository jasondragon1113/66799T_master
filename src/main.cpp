#include "main.h"

Task* intake_task = nullptr;

void initialize() {
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