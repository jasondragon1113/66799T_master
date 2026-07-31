#include "main.h"
#include "vexdash_pros/vexdash_pros.h" // vexdash car-side telemetry library
#include "tune_opcontrol.h"            // PID-tuning build only -- see the header

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

	// drive_distance() does NOT just run the drive PID -- it runs a SECOND PID
	// at the same time, the heading loop, which is what keeps the robot pointing
	// straight while it drives (and what makes an arc when you pass it a heading
	// that differs from the current one). Without these three sliders you can
	// tune "how far" but not "how straight", and a drive that veers looks like a
	// bad drive_kD when it is really an untuned heading_kP.
	// 中文：drive_distance() 不是只跑直走 PID，它同時跑第二組 heading PID——那才是
	// 「開直線不歪」的那一組（給它一個跟現在不同的朝向就會變成畫弧）。沒有這三顆
	// 滑桿，你只調得到「開多遠」、調不到「開多直」；而車子跑歪看起來很像
	// drive_kD 沒調好，其實是 heading_kP 沒調。
	vexdash::watch_config("heading_kP", &chassis.heading_kp, "heading/pid");
	vexdash::watch_config("heading_kI", &chassis.heading_ki, "heading/pid");
	vexdash::watch_config("heading_kD", &chassis.heading_kd, "heading/pid");

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

	// --- vexdash: cascade (the lift on ports 7 / -3) live graph + tuning ---
	// The "cascade/pid" path is the v1.4 CHANNEL_DEF grouping field, so these
	// lines land next to the cascade sliders on the dashboard.
	// 中文：cascade（7 與 -3 那支升降）的即時圖表與調參。"cascade/pid" 是 v1.4 的
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

#ifdef PID_TUNE_PROGRAM
	// --- vexdash: cascade four-level height targets (tuning build only) ---
	// Targets for the tuning program's B/Y/X/A level keys (see
	// tune_opcontrol.cpp). Sliders instead of hardcoded numbers, same convention
	// as the arm presets above; defaults follow ScoringLevel LEVEL_0..LEVEL_3
	// (auton-routines.h), the values autonomous score() already drives to.
	// 中文：調參版 B/Y/X/A 四段高度鍵的目標值（見 tune_opcontrol.cpp）。做成
	// 滑桿、不寫死，跟上面手臂 preset 同一套慣例；預設值取自走 score() 已經在用
	// 的 ScoringLevel LEVEL_0～LEVEL_3（auton-routines.h）。
	vexdash::watch_config("cascade_LV0", &CASCADE_LV0_DEG, "cascade/presets");
	vexdash::watch_config("cascade_LV1", &CASCADE_LV1_DEG, "cascade/presets");
	vexdash::watch_config("cascade_LV2", &CASCADE_LV2_DEG, "cascade/presets");
	vexdash::watch_config("cascade_LV3", &CASCADE_LV3_DEG, "cascade/presets");

	// --- vexdash: arm trapezoid shape (tuning build only) ---
	// The four D-pad keys send the arm to the four EXISTING arm/presets sliders
	// (DOWN / POS_2 / POS_3 / POS_1 -- see the key table in tune_opcontrol.cpp),
	// so no second set of position numbers is registered here: those four sliders
	// already ARE the arm's four positions, and a duplicate set would be two
	// places to change the same angle plus four more registry entries this build
	// does not have room for.
	// These three shape how the arm gets there: cruise speed, ramp up, ramp down.
	// arm_setpoint is the angle the profile is commanding right now -- graph it
	// against arm_angle and the gap between the lines is the tracking error.
	// 中文：四顆方向鍵送手臂去的是「已經存在的」四顆 arm/presets 滑桿（DOWN／POS_2／
	// POS_3／POS_1，按鍵表見 tune_opcontrol.cpp），所以這裡不再登記第二套位置數字：
	// 那四顆滑桿本來就是手臂的四個位置，再開一套等於同一個角度有兩個地方要改，而且
	// 這一版的登記表也沒有那四格可用。
	// 下面三顆決定「怎麼過去」：巡航速度、加速、減速。arm_setpoint 是梯形當下要求的
	// 角度——跟 arm_angle 疊起來看，兩條線的差距就是追蹤誤差。
	vexdash::watch_config("arm_vel", &ARM_PROFILE_VEL_DPS,  "arm/profile"); // deg/s
	vexdash::watch_config("arm_acc", &ARM_PROFILE_ACC_DPS2, "arm/profile"); // deg/s^2
	vexdash::watch_config("arm_dec", &ARM_PROFILE_DEC_DPS2, "arm/profile"); // deg/s^2
	vexdash::watch("arm_setpoint", &tele_arm_setpoint, "deg", -1, "arm/profile");
#endif

	// --- vexdash: on-demand PID tests (set the target, toggle "run", watch the Graph) ---
	vexdash::watch_config("test_distance", &test_distance,  "drive/test"); // inches
	vexdash::watch_config("run_drive",     &run_drive_test, "drive/test"); // toggle ON to drive
	vexdash::watch_config("test_angle",    &test_angle,     "turn/test");  // degrees
	vexdash::watch_config("run_turn",      &run_turn_test,  "turn/test");  // toggle ON to turn

	// --- vexdash: stream the port-5 motor (intake) onto the Graph: pos/rpm/temp/amp ---
	vexdash::watch_motor("motor", intake);

	// --- vexdash: the two cascade motors, separately ---
	// The cascade channels above all come from cascade_get_position_deg(), which
	// reads cascade1 and only falls back to cascade2 when cascade1 reports
	// nothing. So a cascade2 that has stopped pulling -- unplugged, tripped,
	// overheating -- is invisible: the lift just gets weak and slow ("it only
	// moves one motor") with nothing on the dashboard saying why. Position and
	// temperature per motor make it obvious.
	//
	// Four hand-picked channels rather than watch_motor()'s eight (that helper
	// registers pos/rpm/temp/amp for EACH motor): the registry holds 64 entries
	// in total and this build needs the room for the arm profile below. rpm on
	// two motors bolted to the same lift adds nothing that "the two positions
	// drifted apart" has not already said, and current is something you read off
	// a graph afterwards -- position and temperature are what you watch live.
	// 中文：上面那幾條 cascade 頻道全部來自 cascade_get_position_deg()，而它讀的是
	// cascade1、只有 cascade1 讀不到才退而求其次讀 cascade2。所以 cascade2 只要不出
	// 力（線鬆了、跳保護、過熱）就完全看不出來——升降只是變弱變慢（就是「只動一顆」
	// 那個症狀），dashboard 上卻沒有任何線索。分開報位置與溫度就一眼看得出來。
	// 這裡挑四條、而不是用 watch_motor 的八條（那個助手一顆馬達就登記 pos/rpm/temp/
	// amp 四條）：登記表全部只有 64 格，下面手臂梯形要用到。兩顆鎖在同一支升降上的
	// 馬達，rpm 能講的事情「兩顆位置拉開」已經講完了；電流是事後看圖用的——現場要盯
	// 的是位置和溫度。
	vexdash::watch("cascade1_pos",  &tele_cascade1_pos,  "deg", -1, "cascade/pid");
	vexdash::watch("cascade2_pos",  &tele_cascade2_pos,  "deg", -1, "cascade/pid");
	vexdash::watch("cascade1_temp", &tele_cascade1_temp, "C",   -1, "cascade/pid");
	vexdash::watch("cascade2_temp", &tele_cascade2_temp, "C",   -1, "cascade/pid");

	// --- vexdash: Device Map -- sensors section ---
	vexdash::declare_device(distance_sensorL.get_port(), vexdash::DeviceType::kDistance, "distance_L");
	vexdash::declare_device(distance_sensorR.get_port(), vexdash::DeviceType::kDistance, "distance_R");
	vexdash::declare_device(arm_rotation.get_port(), vexdash::DeviceType::kRotation, "arm_rotation");

	// --- vexdash: Device Map -- motors section ---
	// Ports are written as literals on purpose: reversed motors are constructed
	// with a NEGATIVE port and get_port() can return that negative value -- cast
	// into declare_device()'s uint8_t it becomes e.g. 253 and the declaration is
	// silently rejected (out of range 1~21). Keep in sync with robot-config.cpp.
	// 中文：這裡故意寫埠號數字，不用 get_port()：反轉馬達是用「負埠號」建構的，
	// get_port() 可能回傳負值，轉成 uint8_t 會變成 253 之類的數字而被靜默拒絕。
	// 改接線時記得跟 robot-config.cpp 一起改。
	vexdash::declare_device(4,  vexdash::DeviceType::kMotor, "left_front");
	vexdash::declare_device(6,  vexdash::DeviceType::kMotor, "left_mid");
	vexdash::declare_device(16, vexdash::DeviceType::kMotor, "left_back");
	vexdash::declare_device(15, vexdash::DeviceType::kMotor, "right_front");
	vexdash::declare_device(1,  vexdash::DeviceType::kMotor, "right_mid");
	vexdash::declare_device(8,  vexdash::DeviceType::kMotor, "right_back");
	vexdash::declare_device(5,  vexdash::DeviceType::kMotor, "intake");
	vexdash::declare_device(7,  vexdash::DeviceType::kMotor, "cascade_1");
	vexdash::declare_device(3,  vexdash::DeviceType::kMotor, "cascade_2");
	vexdash::declare_device(arm.get_port(), vexdash::DeviceType::kMotor, "arm");
	// NOTE: fwd_tracker (Rotation 2) and sideways_tracker (Rotation 1) in
	// robot-config.cpp are placeholders for trackers the robot does not have,
	// and their port numbers collide with real devices (distance_L on port 2,
	// right_mid on port 1) -- they are deliberately NOT declared here.
	// 中文：robot-config.cpp 裡的 fwd_tracker／sideways_tracker 是不存在的假
	// tracker（隨手填的埠號還跟真裝置的 2、1 埠相撞），所以故意不宣告。

	// --- vexdash: Device Map -- IMU ---
	vexdash::declare_device(17, vexdash::DeviceType::kImu, "imu");

	// --- vexdash: Device Map -- ADI (three-wire) devices ---
	// There is no dedicated DeviceType for digital in/out, so these are declared
	// as kUnknown: the port shows up on the map with its name, but no live values
	// (ADI devices cannot be auto-detected anyway -- they are shown as declared).
	// 中文：協定沒有「數位輸出／輸入」這種裝置型別，所以用 kUnknown 宣告——孔位圖
	// 會顯示這個埠有名字，但沒有即時數值（ADI 本來就偵測不到插拔，照宣告顯示）。
	vexdash::declare_device(vexdash::adi_port('A'), vexdash::DeviceType::kUnknown, "claw");
	vexdash::declare_device(vexdash::adi_port('C'), vexdash::DeviceType::kUnknown, "toggle");
	vexdash::declare_device(vexdash::adi_port('D'), vexdash::DeviceType::kUnknown, "cascade_limit");

	// Start vexdash over the ESP32 Smart Port bridge (port 11 @ 921600 baud).
	// The ESP32 relays telemetry to the dashboard over WiFi (ws://192.168.4.1).
	// NOTE: smart port 11 is the ESP32 bridge and nothing else — distance_sensorL
	// is on port 2 (see robot-config.cpp), so there is no smart-port conflict.
	// 中文：智慧埠 11 就是 ESP32 橋接埠，沒有別的裝置（distance_sensorL 在埠 2）。
	// Smart Port path leaves stdout free (printf still works); HUD off by default.
	// 921600 must match the ESP32 bridge firmware exactly (official firmware
	// moved off 115200 on 2026-07-25 for headroom). If the dashboard suddenly
	// can't connect after this build: re-flash the ESP32 with the latest
	// ESP_32_WIFI.ino — an old 115200 bridge cannot talk to this baud rate.
	// 中文：921600 要跟 ESP32 橋韌體完全一致（官方韌體 7/25 起改 921600）。
	// 燒了這版之後 dashboard 連不上＝ESP32 還是舊韌體，重燒最新 ESP_32_WIFI.ino 即可。
	//
	// DIAGNOSTIC TOGGLE: set to 1 to send telemetry over the USB cable instead
	// of the ESP32 bridge (dashboard: switch to "Web Serial" and plug USB into
	// the Brain). If USB streams fine but the bridge shows zero data, the fault
	// is in the RS-485 wiring / ESP32 / smart-port path — not in this program.
	// 中文：診斷開關——改成 1 就走 USB 線傳遙測（dashboard 切「Web Serial」、
	// USB 插 Brain）。USB 通、橋接零資料＝問題在 RS-485 線/ESP32/智慧埠那段，
	// 不在車端程式。測完記得改回 0 重燒。
#define VEXDASH_OVER_USB 0
#if VEXDASH_OVER_USB
	// show_status=false: keep the Brain screen free (no LLEMU status HUD).
	// 中文：第二個參數 false＝不開 Brain 螢幕的狀態畫面（pros::lcd）。
	vexdash::init_usb(nullptr, /*show_status=*/false);
#else
	vexdash::init_smartport(11, 921600);
#endif

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
#ifdef PID_TUNE_PROGRAM
	// PID-TUNING BUILD. Built with -DPID_TUNE_PROGRAM (`pros make tune`) and
	// uploaded to SLOT 2; a match only ever runs slot 1. tune_opcontrol() never
	// returns, and everything in the #else branch below is not compiled at all
	// in this build -- the two programs never coexist in one binary, so there is
	// no mode flag to get stuck in the wrong position.
	// 中文：這是調參版（用 -DPID_TUNE_PROGRAM 編，燒 slot 2；比賽只跑 slot 1）。
	// tune_opcontrol() 不會回來，而且下面 #else 那一段在這一版根本不會被編進去
	// ——兩支程式不會同時存在於同一顆二進位檔，所以不存在「模式卡在錯的位置」。
	tune_opcontrol();
#else
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
#endif // PID_TUNE_PROGRAM
}