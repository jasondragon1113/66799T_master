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
	//
	// EVERY motion gets the WHOLE JAR parameter set, not just three gains. A
	// JAR move is set_*_constants(max_voltage, kP, kI, kD, startI) plus
	// set_*_exit_conditions(settle_error, settle_time, timeout), and the three
	// exit numbers decide as much of the felt behaviour as the gains: they are
	// what "it stops short", "it hunts at the end" and "it gives up early"
	// actually are. Tuning with only kP/kI/kD means reaching for a gain to fix
	// something no gain controls.
	// startI especially: PID.cpp only integrates while |error| < startI, so a
	// startI of 0 kills the I term outright -- that is the bug that made
	// drive_kI do nothing, and it is not visible anywhere unless the number is
	// on screen. Units follow JAR: drive is INCHES, turn/swing/heading DEGREES.
	// 中文：每一種動作都給「整組」JAR 參數，不是只有三個增益。一個 JAR 動作是
	// set_*_constants(最大電壓, kP, kI, kD, startI) 加上
	// set_*_exit_conditions(settle_error, settle_time, timeout)，而那三個結束條件對
	// 手感的影響不輸增益：「差一點點就停」「到終點前一直來回抖」「還沒到就放棄」講的
	// 就是它們。只給 kP/kI/kD 等於逼人用增益去修一個增益管不到的東西。
	// startI 尤其重要：PID.cpp 只有在 |誤差| < startI 時才積分，所以 startI＝0 等於把
	// I 項整個關掉——那正是 drive_kI 沒反應的元兇，而且數字不擺上畫面根本看不出來。
	// 單位照 JAR：直走是「吋」，轉彎／單邊轉／朝向是「度」。
	vexdash::watch_config("drive_kP",     &chassis.drive_kp,     "drive/pid");
	vexdash::watch_config("drive_kI",     &chassis.drive_ki,     "drive/pid");
	vexdash::watch_config("drive_kD",     &chassis.drive_kd,     "drive/pid");
	vexdash::watch_config("drive_startI", &chassis.drive_starti, "drive/pid"); // in
	vexdash::watch_config("drive_settle_error", &chassis.drive_settle_error, "drive/pid"); // in
	vexdash::watch_config("drive_settle_time",  &chassis.drive_settle_time,  "drive/pid"); // ms
	vexdash::watch_config("drive_timeout",      &chassis.drive_timeout,      "drive/pid"); // ms

	vexdash::watch_config("turn_kP",     &chassis.turn_kp,     "turn/pid");
	vexdash::watch_config("turn_kI",     &chassis.turn_ki,     "turn/pid");
	vexdash::watch_config("turn_kD",     &chassis.turn_kd,     "turn/pid");
	vexdash::watch_config("turn_startI", &chassis.turn_starti, "turn/pid"); // deg
	vexdash::watch_config("turn_settle_error", &chassis.turn_settle_error, "turn/pid"); // deg
	vexdash::watch_config("turn_settle_time",  &chassis.turn_settle_time,  "turn/pid"); // ms
	vexdash::watch_config("turn_timeout",      &chassis.turn_timeout,      "turn/pid"); // ms

	// Swing (one side held, the other drives) had NO sliders at all, so the one
	// motion whose gains differ most from turn's was the one that could not be
	// tuned from the dashboard.
	// 中文：單邊轉（一邊煞住、另一邊開）原本一顆滑桿都沒有——偏偏它的增益跟轉彎差最多，
	// 卻是唯一不能在 dashboard 上調的動作。
	vexdash::watch_config("swing_kP",     &chassis.swing_kp,     "swing/pid");
	vexdash::watch_config("swing_kI",     &chassis.swing_ki,     "swing/pid");
	vexdash::watch_config("swing_kD",     &chassis.swing_kd,     "swing/pid");
	vexdash::watch_config("swing_startI", &chassis.swing_starti, "swing/pid"); // deg
	vexdash::watch_config("swing_settle_error", &chassis.swing_settle_error, "swing/pid"); // deg
	vexdash::watch_config("swing_settle_time",  &chassis.swing_settle_time,  "swing/pid"); // ms
	vexdash::watch_config("swing_timeout",      &chassis.swing_timeout,      "swing/pid"); // ms

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
	// Heading is a SLAVE loop -- it has no exit conditions of its own because it
	// never ends a move; drive_distance() ends when the DRIVE PID settles. So
	// its full set is four numbers, not seven.
	// 中文：朝向是「從屬」迴路，沒有自己的結束條件——動作什麼時候結束是由直走那組決定
	// 的。所以它的完整參數是四個，不是七個。
	//
	// Declared under "drive/pid", NOT a group of its own. The dashboard's modes
	// filter by EXACT path, so a "heading/pid" group would exile these four to a
	// separate mode -- and the one moment they are needed is while tuning a
	// drive: heading is the loop running INSIDE drive_distance(), and a drive
	// that veers is heading_kP far more often than it is drive_kD. Same group
	// means the drive mode shows both halves of what "goes straight" is made of.
	// 中文：掛在 "drive/pid"，不另開一組。dashboard 的模式是照 path「完全相等」過濾的，
	// 開成 "heading/pid" 就會把這四顆流放到另一個模式裡——但真正需要它們的時刻正是在調
	// 直走：heading 就是 drive_distance() 裡面同時在跑的那一組，而車子跑歪十之八九是
	// heading_kP，不是 drive_kD。放同一組，調直走的模式才會一次看到「開得直」的兩半。
	vexdash::watch_config("heading_kP",     &chassis.heading_kp,     "drive/pid");
	vexdash::watch_config("heading_kI",     &chassis.heading_ki,     "drive/pid");
	vexdash::watch_config("heading_kD",     &chassis.heading_kd,     "drive/pid");
	vexdash::watch_config("heading_startI", &chassis.heading_starti, "drive/pid"); // deg

	// --- vexdash: live graph channels (streamed to the web Graph panel) ---
	vexdash::watch("drive_error",  &chassis.drive_error,       "in");
	vexdash::watch("drive_target", &chassis.tele_drive_target, "in");
	vexdash::watch("drive_output", &chassis.tele_drive_output, "V");
	vexdash::watch("turn_error",   &chassis.tele_turn_error,   "deg");
	vexdash::watch("turn_target",  &chassis.tele_turn_target,  "deg");
	vexdash::watch("turn_output",  &chassis.tele_turn_output,  "V");

	// Where the robot thinks it is. Declared under "drive/pid" so the drive and
	// turn tuning modes show it -- a mode filters by the declared path, and a
	// pose that lands in some other group is a pose the coach cannot see while
	// tuning the very loops that produce it. Units are JAR's (in / deg); the
	// Field panel's marker is fed separately by set_pose() in display.cpp, which
	// costs no registry slot at all.
	// 中文：車子認為自己在哪裡。宣告成 "drive/pid" 才會出現在底盤與轉彎的調車模式裡
	// ——模式是照宣告的 path 過濾的，座標掉到別組就等於「調著產生它的那兩個迴路時反而
	// 看不到它」。單位照 JAR（吋／度）；場地面板上的圖示由 display.cpp 的 set_pose()
	// 另外餵，那條路一格登記表都不佔。
	vexdash::watch("pose_x",   &tele_pose_x,       "in",  -1, "drive/pid");
	vexdash::watch("pose_y",   &tele_pose_y,       "in",  -1, "drive/pid");
	vexdash::watch("heading",  &tele_pose_heading, "deg", -1, "drive/pid");

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
	vexdash::watch_config("DOWN",  &ARM_DOWN_DEG,  "arm/pid");
	vexdash::watch_config("POS_1", &ARM_POS_1_DEG, "arm/pid");
	vexdash::watch_config("POS_2", &ARM_POS_2_DEG, "arm/pid");
	vexdash::watch_config("POS_3", &ARM_POS_3_DEG, "arm/pid");
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
	vexdash::watch_config("cascade_startI", &CASCADE_STARTI, "cascade/pid"); // motor deg
	// Height-dependent gravity feedforward: one number for the bottom, one for
	// the top, and the two heights they were measured at. A cascade's held
	// weight grows as it extends, so a single kG sags high and creeps low --
	// see cascade.h. All 0 by default = feedforward off, exactly as before.
	// 中文：跟高度有關的重力前饋：低點一個數字、高點一個數字，加上量測時的那兩個高度。
	// 串接式升降伸出去之後扛的重量會變，只用一個 kG 的話高處往下沉、低處自己往上爬
	// ——理由見 cascade.h。預設全 0＝不啟用，跟以前完全一樣。
	vexdash::watch_config("cascade_kG_bottom", &CASCADE_KG, "cascade/pid");
	vexdash::watch_config("cascade_kG_top",    &CASCADE_KG_TOP, "cascade/pid");
	vexdash::watch_config("cascade_kG_from_deg", &CASCADE_KG_RAMP_START_DEG, "cascade/pid");
	vexdash::watch_config("cascade_kG_to_deg",   &CASCADE_KG_RAMP_END_DEG,   "cascade/pid");

#ifdef PID_TUNE_PROGRAM
	// --- vexdash: cascade four-level height targets (tuning build only) ---
	// Targets for the tuning program's B/Y/X/A level keys (see
	// tune_opcontrol.cpp). Sliders instead of hardcoded numbers, same convention
	// as the arm presets above; defaults follow ScoringLevel LEVEL_0..LEVEL_3
	// (auton-routines.h), the values autonomous score() already drives to.
	// 中文：調參版 B/Y/X/A 四段高度鍵的目標值（見 tune_opcontrol.cpp）。做成
	// 滑桿、不寫死，跟上面手臂 preset 同一套慣例；預設值取自走 score() 已經在用
	// 的 ScoringLevel LEVEL_0～LEVEL_3（auton-routines.h）。
	vexdash::watch_config("cascade_LV0", &CASCADE_LV0_DEG, "cascade/pid");
	vexdash::watch_config("cascade_LV1", &CASCADE_LV1_DEG, "cascade/pid");
	vexdash::watch_config("cascade_LV2", &CASCADE_LV2_DEG, "cascade/pid");
	vexdash::watch_config("cascade_LV3", &CASCADE_LV3_DEG, "cascade/pid");

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
	vexdash::watch_config("arm_vel", &ARM_PROFILE_VEL_DPS,  "arm/pid"); // deg/s
	vexdash::watch_config("arm_acc", &ARM_PROFILE_ACC_DPS2, "arm/pid"); // deg/s^2
	vexdash::watch_config("arm_dec", &ARM_PROFILE_DEC_DPS2, "arm/pid"); // deg/s^2
	vexdash::watch("arm_setpoint", &tele_arm_setpoint, "deg", -1, "arm/pid");

	// --- vexdash: feedforward ramp test (tuning build only) ---
	// The dashboard's "量前饋" panel matches on these FIVE EXACT channel names and
	// shows nothing until all five exist -- see tune_opcontrol.h for the contract
	// and for why tune_volt carries move() command units rather than volts.
	// No group/path is passed on purpose: the panel looks the names up globally,
	// and a path would only add a grouping the panel does not read.
	// 中文：dashboard 的「量前饋」面板是照這五個**固定名字**比對的，五條沒到齊就整頁
	// 空白——命名規則、以及 tune_volt 為什麼帶的是 move() 指令刻度而不是伏特，見
	// tune_opcontrol.h。這裡故意不填群組路徑：面板是用全域名字找的，填了也只是多一個
	// 它不會讀的分組。
	vexdash::watch("tune_ms",   &TUNE_CH_MS,   "ms");
	vexdash::watch("tune_volt", &TUNE_CH_VOLT, "cmd");   // -127..127, same scale as *_KG
	vexdash::watch("tune_pos",  &TUNE_CH_POS,  "deg");
	vexdash::watch("tune_vel",  &TUNE_CH_VEL,  "deg/s");
	vexdash::watch("tune_seg",  &TUNE_CH_SEG,  "leg");   // 0 idle, 1 forward, 2 reverse

	// The panel only shows its one-press buttons for commands that exist, so the
	// five channels above are half the feature and these two are the other half.
	// requires_confirm=true: this moves a real mechanism with nobody's hand on the
	// controller, so the panel must ask first. Commands live in their own 16-slot
	// registry (command_registry.h), NOT in the 64-entry watch table -- they cost
	// nothing against the budget below.
	// 中文：面板只會為「真的存在的命令」長出一鍵按鈕，所以上面五條頻道是功能的一半，
	// 這兩顆命令是另一半。requires_confirm=true：它會在沒有人手放在遙控器上的情況下讓
	// 真的機構動起來，所以面板一定要先問。命令走的是自己那張 16 格的表
	// （command_registry.h），不佔下面那張 64 格的 watch 表，對預算是零成本。
	vexdash::declare_command("ff_ramp_arm", &tune_ff_ramp_arm_command,
	                         /*requires_confirm=*/true);
	vexdash::declare_command("ff_ramp_cascade", &tune_ff_ramp_cascade_command,
	                         /*requires_confirm=*/true);
	// Manual capture toggles: press once to start recording ordinary movement,
	// press again to end the run (the end is what makes the panel fit). These do
	// not move anything by themselves -- the coach does the moving with the D-pad
	// -- so they do not need a confirmation step the way the ramps do.
	// 中文：手動擷取的開關：按一下開始錄「平常的動作」，再按一下結束（結束才會讓面板
	// 擬合）。它們本身不會讓任何東西動——動作是教練自己用方向鍵做的——所以不像斜坡那樣
	// 需要先跳確認。
	vexdash::declare_command("ff_capture_arm", &tune_ff_capture_arm_command);
	vexdash::declare_command("ff_capture_cascade", &tune_ff_capture_cascade_command);

	// --- vexdash: on-demand PID tests (set the target, toggle "run", watch the Graph) ---
	// COMPETITION BUILD ONLY. In the tuning build opcontrol() hands straight over
	// to tune_opcontrol() and never reaches the code that reads these four, so
	// there they are four sliders that cannot do anything -- and the tuning
	// program's own L1/L2/R1/R2 keys are the same tests done properly (with abort,
	// timeouts and a competition watchdog). Keeping them out of that build is what
	// pays for the five tune_* channels above.
	// 中文：只有比賽版才登記這四顆。調參版的 opcontrol() 直接交棒給 tune_opcontrol()，
	// 永遠不會執行到讀這四個變數的程式，所以在那一版它們是「拉了也不會怎樣」的四顆滑桿
	// ——而且調參版自己的 L1/L2/R1/R2 就是同樣的測試、還做得更完整（有中止、有逾時、有
	// 比賽看門狗）。把它們排除在調參版之外，正是上面五條 tune_* 頻道的名額來源。
#else
	vexdash::watch_config("test_distance", &test_distance,  "drive/test"); // inches
	vexdash::watch_config("run_drive",     &run_drive_test, "drive/test"); // toggle ON to drive
	vexdash::watch_config("test_angle",    &test_angle,     "turn/test");  // degrees
	vexdash::watch_config("run_turn",      &run_turn_test,  "turn/test");  // toggle ON to turn
#endif

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
	// amp 四條）。登記表上限現在是 96（不是原本的 64），但真正緊的不是那個數字，是
	// 「一次 flush 最多塞 40 個 sample」（kMaxSamplesPerFrame）——每一條遙測頻道每次
	// flush 就佔一格。兩顆鎖在同一支升降上的馬達，rpm 能講的事情「兩顆位置拉開」已經
	// 講完了；電流是事後看圖用的——現場要盯的是位置和溫度。
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
	// Telemetry pacing. Two separate numbers decide whether a frame is honest:
	//   * how often the link flushes (this one, telemetry_period_ms), and
	//   * how many samples fit in one flush (kMaxSamplesPerFrame = 40, a
	//     protocol limit -- NOT the 96-slot registry cap, which is a different
	//     thing that people mix up).
	// This build registers 34 telemetry channels (the 49 config sliders do not
	// stream -- they only move when someone drags them), so ONE sample per
	// channel per flush = 34 <= 40. It fits with six to spare; a seventh new
	// GRAPH channel is what would start spilling frames, not a new slider.
	//
	// 25 ms rather than the 20 ms default, matching 66994V: it is the same
	// pacing decision, arrived at there because a publisher running FASTER than
	// the flush puts two samples of the same channel into one window and
	// silently doubles the frame. This robot samples inside the pre_flush hook,
	// so exactly one sample per channel per flush is structural here and that
	// particular bug cannot occur -- but 25 ms still buys headroom on the same
	// serial link (40 Hz instead of 50 Hz) and keeps the two robots' streams
	// comparable when the same dashboard is used to tune both.
	// 中文：遙測節奏。一幀資料誠不誠實由兩個數字決定：多久 flush 一次（就是這個
	// telemetry_period_ms），以及一次 flush 塞得下幾個 sample（kMaxSamplesPerFrame＝40，
	// 那是協定上限——**不是**登記表那個 96，兩者常被搞混）。
	// 這一版登記了 37 條遙測頻道（config 滑桿不會串流），每條每次 flush 佔一個 sample＝
	// 37 ≤ 40，塞得下、只剩三格；再多加四條就會開始溢到第二幀。
	// 用 25ms 而不是預設的 20ms，跟 66994V 一致：那邊會這樣定，是因為「發布比 flush 快」
	// 會讓同一條頻道在同一個視窗裡塞進兩筆、把幀量默默翻倍。這台車是在 pre_flush 鉤子
	// 裡取樣的，「每條每次 flush 剛好一筆」是結構保證，那個 bug 在這裡不可能發生——但
	// 25ms 仍然替同一條序列線多留了餘裕（40Hz 而不是 50Hz），也讓兩台車用同一個
	// dashboard 調參時的資料流節奏一致。
	vexdash::PumpConfig pump_cfg;
	pump_cfg.telemetry_period_ms = 25;
	// registration_pace_bytes / registration_pace_window_ms keep their defaults
	// (320 B per 10 ms) = the registration TRICKLE. What it fixes is documented
	// on PumpConfig in connection_pump.h; the arithmetic below is THIS robot's,
	// recomputed from its own registry, not 66994V's:
	//   * registry = 83 entries in the tuning build (74 in the competition one),
	//     one definition frame each at ~68 B on the wire = ~5.6 KB per burst,
	//     sent on link-up AND again every registration_resend_period_ms.
	//   * steady-state telemetry = 34 channels x ~10 B + framing = ~350 B per
	//     flush, which at the 25 ms period above is ~14 KB/s.
	//   * the bucket is shared, so registration gets 32 - 14 = ~18 KB/s of the
	//     320 B/10 ms budget: ~5.6 KB / 18 KB/s = ~0.31 s to place the whole
	//     registry -- an order of magnitude inside link_timeout_ms (3000 ms),
	//     with the paced_stall PING every 500 ms as the backstop if a burst
	//     ever ran long.
	// 中文：registration_pace_bytes／window_ms 沿用預設（每 10ms 320 bytes），也就是
	// 註冊涓流。它在修什麼看 connection_pump.h 裡 PumpConfig 那段；下面這筆帳是**這台車
	// 自己**的登記量重算的，不是 66994V 的：
	//   * 登記表＝調參版 83 筆（比賽版 74 筆），每筆一個定義幀、線上約 68 bytes＝一輪
	//     約 5.6KB；開機首次註冊與之後每個 registration_resend_period_ms 各送一輪。
	//   * 穩態遙測＝34 條頻道 × 約 10 bytes ＋封包框架 ≒ 每次 flush 約 350 bytes，
	//     以上面 25ms 的週期算約 14KB/s。
	//   * 桶是共用的，所以註冊分到 32−14＝約 18KB/s：5.6KB ÷ 18KB/s ≒ 0.31 秒送完整份
	//     登記表——比 link_timeout_ms（3000ms）小一個數量級；萬一某一輪拖長，還有
	//     paced_stall 每 500ms 的 PING 接住。

#define VEXDASH_OVER_USB 0
#if VEXDASH_OVER_USB
	// show_status=false: keep the Brain screen free (no LLEMU status HUD).
	// 中文：第二個參數 false＝不開 Brain 螢幕的狀態畫面（pros::lcd）。
	vexdash::init_usb(nullptr, /*show_status=*/false);
#else
	vexdash::init_smartport(11, 921600, /*on_register=*/nullptr, /*show_status=*/false,
	                        /*user_data=*/nullptr, pump_cfg);
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