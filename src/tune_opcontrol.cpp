#include "main.h"
#include "tune_opcontrol.h"

#include <cstring>
#include <cstdint>

// ============================================================================
// PID-tuning teleop program (build with -DPID_TUNE_PROGRAM, upload to slot 2)
// ============================================================================
//
// EVERYTHING in this file is inside the #ifdef below. In a normal build this
// translation unit is empty: no code, no data, no symbols, nothing to call.
// That is the whole safety argument -- the competition build cannot run any of
// this, because none of it exists in the competition build.
// 中文：這個檔案的所有內容都包在下面的 #ifdef 裡。正常編譯時它是一個空檔案——
// 沒有程式碼、沒有變數、沒有符號。這就是安全性的全部理由：比賽版跑不到這裡面
// 任何一行，因為比賽版裡根本沒有它。
//
// ---------------------------------------------------------------------------
// 按鍵表（只有這一版才有）
// ---------------------------------------------------------------------------
//   L1    前進 50 cm         chassis.drive_distance(+50cm→吋)
//   L2    後退 50 cm         chassis.drive_distance(-50cm→吋)
//   R1    左轉 90°           chassis.turn_to_angle(現在朝向 - 90)
//   R2    右轉 180°          chassis.turn_to_angle(現在朝向 + 179.5)
//   A     滑軌升到高目標     cascade_set_target(CASCADE_PRESET_2_DEG = 595)
//   B     滑軌降到低目標     cascade_set_target(CASCADE_LEFT_FINAL_DEG = 0)
//   UP    手臂抬到 POS_2     arm_set_position(ArmPosition::POS_2)
//   DOWN  手臂回 DOWN        arm_set_position(ArmPosition::DOWN)
//   左搖桿 Y／右搖桿 X       正常方向盤式駕駛（沒有測試動作在跑的時候才有效），
//                            用來把車開回起點，不必用手搬
//
// 中止：**動作進行中按任何一顆按鍵**（同一顆也算）就中止——遙控器震一下、
// 底盤電壓歸零、滑軌／手臂就地停住。搖桿大幅推動（超過 25／127）也會中止。
//
// ---------------------------------------------------------------------------
// 為什麼手臂是 POS_2／DOWN，不是教練說的 120°／10°
// ---------------------------------------------------------------------------
// 手臂走既有的 arm_set_position/POS 慣例，而現有的 preset 只有四個值：
// DOWN=0°、POS_2=160°、POS_3=265°、POS_1=283°（見 src/Template/arm.cpp）。
// 120° 與 10° 都不在裡面，所以照規格取「最接近的現值」：
//   120° → POS_2（160°，四個裡最接近）
//    10° → DOWN（0°，四個裡最接近）
// 要正好 120／10 的話不用改程式：這四個數字本身就是 dashboard 的滑桿
// （arm/presets 群組的 POS_2／DOWN），在 dashboard 上把 POS_2 拉到 120、
// DOWN 拉到 10，這兩顆按鍵就會跑到 120／10。
//
// ---------------------------------------------------------------------------
// 增益怎麼吃到 dashboard 的值
// ---------------------------------------------------------------------------
// 底盤：JAR 模板的 drive_distance()／turn_to_angle() 是在函式一開頭 new 一個
// PID 物件、把 chassis.drive_kp/ki/kd（turn_* 同理）複製進去。那幾個成員正是
// Z1 掛在 dashboard 上的 drive_kP／turn_kP…滑桿的寫入目標，所以**每按一次鍵
// ＝重新讀一次滑桿現值**。流程因此是「先拉滑桿 → 再按鍵 → 看曲線」，
// 動作跑到一半才拉滑桿不會影響那一次動作（這對階躍響應反而是對的）。
// 這一版刻意**不呼叫 default_constants()**：呼叫的話會把 dashboard 上剛調好的
// 增益覆蓋回程式碼裡的硬編值。initialize() 已經呼叫過一次了，開機值是對的。
// 手臂／滑軌：arm_task()／cascade_task() 本來就每圈重抄增益（Z1 已修），
// 所以那兩支是拉了立刻生效。
// ============================================================================

#ifdef PID_TUNE_PROGRAM

// Teleop cascade preset targets. They live in src/Template/drive.cpp with
// external linkage and no header declaration; declared here rather than adding
// them to a header, so nothing on the normal driving path is touched.
// 中文：滑軌的既有 preset 目標值定義在 drive.cpp、沒有放進標頭檔。這裡直接宣告
// extern，而不是去改標頭檔——正常駕駛那條路徑一個字都不動。
extern int CASCADE_PRESET_2_DEG;    // 595 -- highest existing preset (LEFT sequence)
extern int CASCADE_LEFT_FINAL_DEG;  // 0   -- lowest existing preset (fully retracted)

namespace {

// --- what the test buttons aim at -------------------------------------------
constexpr float TUNE_DRIVE_CM = 50.0f;   // L1 forward / L2 back
constexpr float TUNE_TURN_LEFT_DEG = -90.0f;

// R2 is "right 180" but is commanded as 179.5. reduce_negative_180_to_180()
// maps exactly +180 to -180 (see util.cpp), so a literal +180 target produces a
// negative error on the first cycle and the robot commits to turning LEFT.
// Half a degree short keeps the error positive, i.e. an unambiguous RIGHT turn,
// and 0.5 deg is irrelevant to a step response.
// 中文：R2 是「右轉 180°」，但下的命令是 179.5°。角度歸一化把正好 +180 折成
// -180（util.cpp），所以真的填 180 的話第一圈誤差是負的、車子會往左轉。少半度
// 就能讓誤差保持正值＝確定往右轉，而 0.5 度對階躍響應的觀察完全沒有影響。
constexpr float TUNE_TURN_RIGHT_DEG = 179.5f;

constexpr int TUNE_LOOP_MS = 10;

// The whole abort mechanism leans on ONE property of the JAR moves: that they
// eventually fall out of their own loop. PID::is_settled() says so explicitly:
//
//     if (time_spent_running>timeout && timeout != 0) return true;
//     // If timeout does equal 0, the move will never actually time out.
//     // Setting timeout to 0 is the equivalent of setting it to infinity.
//
// So a drive_timeout or turn_timeout of 0 means the move NEVER ends, the abort
// wait below never completes, and the voltage caps would stay pinned at 0
// forever -- a robot that cannot move until it is rebooted.
//
// Rather than paper over that in the abort path (restoring the caps while the
// move is still looping would un-abort it at full power, which is worse than
// being stuck), this build refuses to run with a zero timeout at all: see
// enforce_move_timeouts(), called before any test can be started. These are the
// values default_constants() uses, and are only ever applied if someone edited
// the timeout to 0.
// 中文：整套中止機制只靠一件事：那兩個動作最後會自己結束。PID::is_settled() 白紙
// 黑字寫著 timeout 設 0 等於無限大＝永遠不會結束。真的設 0 的話，下面的等待就永遠
// 等不完，電壓上限會永遠停在 0——車子在重開機之前動不了。
// 這裡不在中止流程裡硬掰（動作還在跑就把上限還回去＝剛剛的中止白做、而且是滿電壓
// 復活，比動不了更糟），而是直接不讓這一版帶著 0 逾時上路：見 enforce_move_timeouts()，
// 在任何測試能被啟動之前就跑過了。下面兩個數字就是 default_constants() 的原值，
// 只有在有人把逾時改成 0 的時候才會被套上。
constexpr float TUNE_MIN_DRIVE_TIMEOUT_MS = 3000;
constexpr float TUNE_MIN_TURN_TIMEOUT_MS = 2000;

// Extra head-room on top of the longer of the two timeouts before the abort
// wait gives up. Derived at abort time from the LIVE values, so it stays honest
// if someone lengthens a timeout in default_constants().
// 中文：等待上限＝兩個逾時裡比較長的那個再加這麼多。是在中止當下讀「現在的值」
// 算出來的，所以有人把逾時改長也不會失準。
constexpr int TUNE_ABORT_WAIT_MARGIN_MS = 1000;

// A mechanism test that never arrives gets parked instead of left leaning on an
// unreachable target (a stalled motor pulling stall current) -- same reasoning
// as preset_abort() in drive.cpp.
// 中文：機構測試到不了目標就把它停在原地，不要留著一個到不了的目標讓 PID 死推
// ——理由跟 drive.cpp 的 preset_abort() 一樣，死推＝堵轉＝燒東西。
constexpr int TUNE_ARM_TIMEOUT_MS = 5000;
constexpr int TUNE_CASCADE_TIMEOUT_MS = 6000;

// Stick deflection that counts as "the driver wants the robot back".
constexpr int TUNE_STICK_ABORT = 25;
constexpr int TUNE_STICK_DEADBAND = 5;

pros::Controller tune_master(pros::E_CONTROLLER_MASTER);

// --- which test is running ---------------------------------------------------
enum class ActiveTest { NONE, CHASSIS, ARM, CASCADE };
enum class ChassisMove { NONE, DRIVE_FWD, DRIVE_BACK, TURN_LEFT, TURN_RIGHT };

ActiveTest active_test = ActiveTest::NONE;
std::uint32_t test_started_ms = 0;

// Handshake with the worker task below. Plain volatile bools: one writer each,
// 10 ms polling, nothing that a race could make unsafe (the worst case is one
// extra cycle of latency).
// 中文：跟下面那支 worker task 的握手旗標。各自只有一個寫入者、10ms 輪詢，
// 最壞的情況只是慢一圈，不會有危險的競態。
volatile bool move_requested = false;
volatile bool move_running = false;
volatile ChassisMove requested_move = ChassisMove::NONE;

// Voltage caps as default_constants() left them, captured once at entry so an
// abort can zero them and put them straight back.
float saved_drive_max_voltage = 0;
float saved_heading_max_voltage = 0;
float saved_turn_max_voltage = 0;
bool caps_saved = false;

// --- controller screen -------------------------------------------------------
// The V5 controller LCD is slow: PROS drops prints that come faster than about
// 50 ms apart. So the loop only ever declares what it WANTS on screen, and this
// pushes at most one changed line every 60 ms.
// 中文：遙控器螢幕很慢，PROS 會把 50ms 內連發的 print 丟掉。所以主迴圈只負責
// 「宣告想顯示什麼」，這裡每 60ms 最多送一行有變動的出去。
constexpr int TUNE_SCREEN_LINES = 3;
constexpr int TUNE_SCREEN_COLS = 20;
char want_line[TUNE_SCREEN_LINES][TUNE_SCREEN_COLS];
char shown_line[TUNE_SCREEN_LINES][TUNE_SCREEN_COLS];
std::uint32_t last_screen_ms = 0;

void screen_set(int line, const char* text){
  if(line < 0 || line >= TUNE_SCREEN_LINES) return;
  std::strncpy(want_line[line], text, TUNE_SCREEN_COLS - 1);
  want_line[line][TUNE_SCREEN_COLS - 1] = '\0';
}

void screen_pump(){
  if(pros::millis() - last_screen_ms < 60) return;
  for(int i = 0; i < TUNE_SCREEN_LINES; i++){
    if(std::strcmp(want_line[i], shown_line[i]) == 0) continue;
    tune_master.print(i, 0, "%-15s", want_line[i]);
    std::strcpy(shown_line[i], want_line[i]);
    last_screen_ms = pros::millis();
    return; // one line per pass, on purpose
  }
}

// --- the chassis worker ------------------------------------------------------
// drive_distance() and turn_to_angle() block until they settle or time out, so
// they cannot run in the button loop. They run here instead.
//
// This task is started ONCE and never dies. It is deliberately never killed:
// drive_distance() contains a printf(), and killing a task in the middle of a
// printf leaves the stdout lock held and takes the serial link down with it.
// Aborting works by clamping the voltage caps to 0 (below) -- both loops
// re-read those caps every cycle, so the wheels stop within 10 ms and the loop
// then exits on its own timeout with the motors already at 0 V.
// 中文：drive_distance()／turn_to_angle() 是會卡住的函式，不能放在按鍵迴圈裡跑，
// 所以丟到這支 task。這支 task 只建立一次、永遠不砍：drive_distance() 裡面有
// printf()，在 printf 中途砍 task 會把 stdout 的鎖留在死掉的 task 手上，整條
// 序列埠就跟著壞掉。中止改用「把電壓上限夾成 0」——那兩個迴圈每圈都重讀上限，
// 所以輪子 10ms 內就停，之後迴圈自己逾時結束，而且結束前輪子早就是 0V。
void tune_move_worker(){
  while(true){
    if(!move_requested){
      pros::delay(TUNE_LOOP_MS);
      continue;
    }
    // Order matters: raise the "running" flag BEFORE clearing "requested", or
    // the button loop can catch the instant where both are false and conclude
    // the move already finished.
    // 中文：這兩行的順序不能反。先把 running 舉起來再清掉 requested，不然按鍵
    // 迴圈剛好在兩者都是 false 的那一瞬間去看，就會以為動作已經跑完了。
    move_running = true;
    move_requested = false;

    switch(requested_move){
      case ChassisMove::DRIVE_FWD:
        chassis.drive_distance((float)cm_to_inch(TUNE_DRIVE_CM));
        break;
      case ChassisMove::DRIVE_BACK:
        chassis.drive_distance(-(float)cm_to_inch(TUNE_DRIVE_CM));
        break;
      case ChassisMove::TURN_LEFT:
        chassis.turn_to_angle(chassis.get_absolute_heading() + TUNE_TURN_LEFT_DEG);
        break;
      case ChassisMove::TURN_RIGHT:
        chassis.turn_to_angle(chassis.get_absolute_heading() + TUNE_TURN_RIGHT_DEG);
        break;
      default:
        break;
    }

    chassis.drive_with_voltage(0, 0);
    move_running = false;
  }
}

void save_voltage_caps(){
  if(caps_saved) return;
  saved_drive_max_voltage = chassis.drive_max_voltage;
  saved_heading_max_voltage = chassis.heading_max_voltage;
  saved_turn_max_voltage = chassis.turn_max_voltage;
  caps_saved = true;
}

// Refuse to run with an exit-condition timeout of 0 (== infinity, see the
// comment on TUNE_MIN_*_TIMEOUT_MS). Returns true if it had to patch something,
// so the caller can say so on the controller. Written as !(x > 0) rather than
// x == 0 so a NaN is caught too.
// 中文：不接受逾時 0（＝無限大，理由見上面）。有補過就回 true，讓呼叫端在手把上
// 講一聲。寫成 !(x > 0) 而不是 x == 0，是為了連 NaN 也一起擋掉。
bool enforce_move_timeouts(){
  bool patched = false;
  if(!(chassis.drive_timeout > 0)){
    chassis.drive_timeout = TUNE_MIN_DRIVE_TIMEOUT_MS;
    patched = true;
  }
  if(!(chassis.turn_timeout > 0)){
    chassis.turn_timeout = TUNE_MIN_TURN_TIMEOUT_MS;
    patched = true;
  }
  return patched;
}

// Longest the abort wait is allowed to take, derived from the live timeouts.
int abort_wait_limit_ms(){
  float longest = chassis.drive_timeout > chassis.turn_timeout
                    ? chassis.drive_timeout : chassis.turn_timeout;
  if(!(longest > 0)) longest = TUNE_MIN_DRIVE_TIMEOUT_MS; // belt and braces
  return (int)longest + TUNE_ABORT_WAIT_MARGIN_MS;
}

void restore_voltage_caps(){
  if(!caps_saved) return;
  chassis.drive_max_voltage = saved_drive_max_voltage;
  chassis.heading_max_voltage = saved_heading_max_voltage;
  chassis.turn_max_voltage = saved_turn_max_voltage;
}

// Stop a running chassis move NOW. Called from the button loop AND from the
// competition watchdog, possibly at the same time -- which is why there is no
// mutex here. Every step is idempotent (zero the caps, wait for the move to
// fall out, put the saved caps back), so two callers doing it at once reach the
// same end state, and a caller that gets killed halfway cannot strand a lock.
// 中文：按鍵迴圈跟看門狗都會呼叫，而且可能同時呼叫——所以這裡刻意不用 mutex。
// 每一步都是「做幾次結果都一樣」（把上限歸零、等動作結束、把存下來的上限放回
// 去），兩邊同時做的結果一樣；而且中途被砍掉也不會留下一把沒人還的鎖。
void abort_chassis_move(){
  move_requested = false;

  // Every volt drive_distance() and turn_to_angle() can produce passes through
  // one of these three clamps, and neither call uses motion chaining (which is
  // what would force a MINIMUM voltage), so 0 here really is 0 at the wheels on
  // the next 10 ms cycle.
  // 中文：那兩個函式送出去的每一伏特都會先經過這三個上限，而且都沒有用 motion
  // chaining（那才會強制一個「最低電壓」），所以這裡設 0，下一個 10ms 迴圈輪子
  // 就真的是 0V。
  chassis.drive_max_voltage = 0;
  chassis.heading_max_voltage = 0;
  chassis.turn_max_voltage = 0;

  // Give the worker a few cycles first. If the abort landed in the sliver
  // between "requested" and "running", the worker may still be about to start
  // the move -- the caps are already 0 by then, so it starts at 0 V and the
  // wait below catches it instead of it slipping through un-aborted.
  // 中文：先讓 worker 跑幾圈。如果中止剛好卡在「已下單、還沒開跑」的縫隙，
  // worker 可能還是會把動作跑起來——但那時上限已經是 0，它是用 0V 起跑，
  // 下面的等待就會接住它，不會有動作漏網。
  pros::delay(TUNE_LOOP_MS * 3);

  const int limit = abort_wait_limit_ms();
  int waited = 0;
  while(move_running && waited < limit){
    pros::delay(TUNE_LOOP_MS);
    waited += TUNE_LOOP_MS;
  }

  // Only hand the voltage back once the move really is over. Restoring while it
  // is still looping would un-abort it at full power.
  // 中文：確定動作真的結束了才把電壓上限還回去。還在跑的時候還回去＝剛剛的中止
  // 白做了，而且是滿電壓復活。
  if(!move_running){
    restore_voltage_caps();
  }
  else {
    // Should be unreachable: enforce_move_timeouts() runs before any test can
    // start, so both timeouts are non-zero and the wait above outlasts them.
    // If it ever happens the caps STAY at 0 -- a robot that will not move is
    // the safe failure here, and a robot that resumes a 180 turn you already
    // aborted is not. Shout about it instead of quietly recovering.
    // 中文：照理到不了這裡（測試開始前 enforce_move_timeouts() 已經保證兩個逾時
    // 都不是 0，等待時間一定撐得比它們久）。萬一真的發生，電壓上限就**留在 0**
    // ——這裡「車子不會動」是安全的失敗，「已經中止的 180 度轉彎自己復活」不是。
    // 所以寧可大聲抱怨，也不要安靜地把電壓還回去。
    screen_set(1, "!ABORT STUCK!");
    screen_set(2, "REBOOT ROBOT");
    tune_master.rumble("---");
  }
  chassis.drive_stop(MotorBrake::coast);
}

// Stop whatever test is running and leave every mechanism where it is.
void abort_active_test(bool buzz){
  // Buzz FIRST. Aborting a chassis move blocks here until the move loop falls
  // out (the wheels are already at 0 V, but that can still be a second or two),
  // and a driver who pressed abort needs to know it registered right now, not
  // when it is over.
  // 中文：先震動再做事。中止底盤動作會在這裡卡到那個迴圈自己結束（輪子早就 0V
  // 了，但還是可能一兩秒），按下中止的人要「馬上」知道有收到，不是等結束才知道。
  if(buzz) tune_master.rumble("-");

  switch(active_test){
    case ActiveTest::CHASSIS:
      abort_chassis_move();
      break;
    case ActiveTest::CASCADE:
      // Park the lift on its current position: the PID then holds it instead of
      // leaning on a target it was told to stop chasing.
      cascade_set_target(cascade_get_position_deg());
      break;
    case ActiveTest::ARM:
      arm_hold_here();
      break;
    default:
      break;
  }
  active_test = ActiveTest::NONE;
}

// --- competition watchdog ----------------------------------------------------
// Field control deletes the opcontrol task the instant the match is disabled or
// autonomous starts -- but it does NOT delete the worker task above, which
// would happily keep driving. This runs forever and cuts a move dead as soon as
// the robot is not in teleop.
//
// It deliberately does not touch the arm or the cascade: during autonomous the
// team's own routines own those two, and disabled() in main.cpp already parks
// the cascade. All this owns is "a tuning move must not outlive teleop".
// 中文：比賽一被 disable 或進入自走，場控會直接砍掉 opcontrol，但**不會**砍上面
// 那支 worker——它會若無其事繼續開車。這支看門狗永遠在跑，只要不在遙控期就立刻
// 把動作掐掉。它刻意不碰手臂與滑軌：自走期間那兩個是自走程式在管，disabled()
// 本來就已經把滑軌控制器停掉了。它只負責一件事：調參動作不准活過遙控期。
void tune_safety_task(){
  while(true){
    if(pros::competition::is_disabled() || pros::competition::is_autonomous()){
      if(move_running || move_requested) abort_chassis_move();
    }
    pros::delay(20);
  }
}

bool competition_lockout(){
  return pros::competition::is_disabled() || pros::competition::is_autonomous();
}

void request_chassis_move(ChassisMove move, const char* label){
  requested_move = move;
  move_requested = true;
  active_test = ActiveTest::CHASSIS;
  test_started_ms = pros::millis();
  screen_set(1, label);
}

} // namespace

void tune_opcontrol(){
  save_voltage_caps();
  // Teleop can start again after field control killed this task mid-abort, with
  // the voltage caps still pinned at 0. Putting them back here is the one place
  // that is guaranteed to run before any new test.
  // 中文：場控有可能在「中止進行到一半」的時候把這支 task 砍掉，那時電壓上限還
  // 停在 0；下一段遙控期一定會先跑到這裡，所以在這裡把它們放回去最保險。
  restore_voltage_caps();

  // Same teleop hand-over the normal driving loop does (see control_arcade()):
  // zero the cascade encoders at the physical bottom the robot was placed at,
  // then let the cascade PID hold position 0. Without this the cascade
  // controller stays disabled and the A/B tests would do nothing.
  // 中文：跟正常駕駛開場做同一件事（見 control_arcade()）：把滑軌編碼器在「機器人
  // 現在擺放的物理底部」歸零，再讓滑軌 PID 撐在 0。不做的話滑軌控制器是關著的，
  // A／B 兩顆鍵按了不會動。
  chassis.drive_stop(MotorBrake::coast);
  cascade1.tare_position();
  cascade2.tare_position();
  cascade_notify_tare();
  cascade_set_target(0);
  cascade_control_set_enabled(true);

  static pros::Task move_worker(tune_move_worker, "tune move");
  static pros::Task safety_watchdog(tune_safety_task, "tune safety");

  // Two buzzes and a permanent banner: nobody can pick this controller up and
  // think they are holding the match program.
  // 中文：震兩下＋螢幕第一行永遠掛著「PID TUNE」，任何人拿起這支遙控器都不會
  // 以為自己拿到的是比賽用的程式。
  tune_master.rumble(". .");
  screen_set(0, "== PID TUNE ==");
  screen_set(1, "READY");
  screen_set(2, "L1F L2B R1L R2R");

  // Runs BEFORE the button loop, i.e. before any test can be started, so the
  // abort path can rely on both timeouts being finite. See TUNE_MIN_*_TIMEOUT_MS.
  // 中文：在按鍵迴圈之前跑，也就是在任何測試能被啟動之前，中止流程才能放心假設
  // 兩個逾時都是有限的。
  if(enforce_move_timeouts()){
    screen_set(1, "TIMEOUT=0 FIXED");
    tune_master.rumble("- -");
  }

  bool last_any = false;

  while(true){
    screen_pump();

    // ---- competition lockout ------------------------------------------------
    if(competition_lockout()){
      if(active_test != ActiveTest::NONE) abort_active_test(false);
      screen_set(1, "COMP LOCK");
      last_any = false;
      pros::delay(TUNE_LOOP_MS);
      continue;
    }

    // ---- read every button once --------------------------------------------
    bool b_l1   = tune_master.get_digital(DIGITAL_L1);
    bool b_l2   = tune_master.get_digital(DIGITAL_L2);
    bool b_r1   = tune_master.get_digital(DIGITAL_R1);
    bool b_r2   = tune_master.get_digital(DIGITAL_R2);
    bool b_a    = tune_master.get_digital(DIGITAL_A);
    bool b_b    = tune_master.get_digital(DIGITAL_B);
    bool b_x    = tune_master.get_digital(DIGITAL_X);
    bool b_y    = tune_master.get_digital(DIGITAL_Y);
    bool b_up   = tune_master.get_digital(DIGITAL_UP);
    bool b_down = tune_master.get_digital(DIGITAL_DOWN);
    bool b_left = tune_master.get_digital(DIGITAL_LEFT);
    bool b_right= tune_master.get_digital(DIGITAL_RIGHT);

    bool any = b_l1 || b_l2 || b_r1 || b_r2 || b_a || b_b || b_x || b_y ||
               b_up || b_down || b_left || b_right;
    bool new_press = any && !last_any;
    last_any = any;

    double throttle = tune_master.get_analog(ANALOG_LEFT_Y);
    double turn = tune_master.get_analog(ANALOG_RIGHT_X);

    // ---- a test is running: the ONLY thing any input does is stop it --------
    if(active_test != ActiveTest::NONE){
      bool stick_grab = fabs(throttle) > TUNE_STICK_ABORT ||
                        fabs(turn) > TUNE_STICK_ABORT;

      if(new_press || stick_grab){
        screen_set(1, "ABORTED");
        screen_pump();
        abort_active_test(true);
        pros::delay(TUNE_LOOP_MS);
        continue;
      }

      // Finished on its own?
      std::uint32_t elapsed = pros::millis() - test_started_ms;
      if(active_test == ActiveTest::CHASSIS){
        if(!move_running && !move_requested){
          active_test = ActiveTest::NONE;
          screen_set(1, "READY");
        }
      }
      else if(active_test == ActiveTest::ARM){
        // arm_settled is recomputed every 10 ms and stays stale (true for the
        // PREVIOUS target) for a moment after arm_set_position() -- same 30 ms
        // latency the preset sequences in drive.cpp allow for.
        if(elapsed > 30 && arm_settled){
          active_test = ActiveTest::NONE;
          screen_set(1, "ARM DONE");
        }
        else if(elapsed > (std::uint32_t)TUNE_ARM_TIMEOUT_MS){
          abort_active_test(true);
          screen_set(1, "ARM TIMEOUT");
        }
      }
      else if(active_test == ActiveTest::CASCADE){
        if(elapsed > 30 && cascade_settled){
          active_test = ActiveTest::NONE;
          screen_set(1, "CASC DONE");
        }
        else if(elapsed > (std::uint32_t)TUNE_CASCADE_TIMEOUT_MS){
          abort_active_test(true);
          screen_set(1, "CASC TIMEOUT");
        }
      }

      pros::delay(TUNE_LOOP_MS);
      continue;
    }

    // ---- idle: buttons start a test ----------------------------------------
    if(new_press){
      if(b_l1)        request_chassis_move(ChassisMove::DRIVE_FWD,  "FWD 50cm");
      else if(b_l2)   request_chassis_move(ChassisMove::DRIVE_BACK, "BACK 50cm");
      else if(b_r1)   request_chassis_move(ChassisMove::TURN_LEFT,  "TURN L90");
      else if(b_r2)   request_chassis_move(ChassisMove::TURN_RIGHT, "TURN R180");
      else if(b_a){
        cascade_set_target((float)CASCADE_PRESET_2_DEG);
        active_test = ActiveTest::CASCADE;
        test_started_ms = pros::millis();
        screen_set(1, "CASC UP");
      }
      else if(b_b){
        cascade_set_target((float)CASCADE_LEFT_FINAL_DEG);
        active_test = ActiveTest::CASCADE;
        test_started_ms = pros::millis();
        screen_set(1, "CASC DOWN");
      }
      else if(b_up){
        // 120 deg requested -> POS_2 is the closest existing preset (see the
        // header comment). Drag POS_2 to 120 on the dashboard for an exact 120.
        arm_set_position(ArmPosition::POS_2);
        active_test = ActiveTest::ARM;
        test_started_ms = pros::millis();
        screen_set(1, "ARM POS_2");
      }
      else if(b_down){
        // 10 deg requested -> DOWN is the closest existing preset.
        arm_set_position(ArmPosition::DOWN);
        active_test = ActiveTest::ARM;
        test_started_ms = pros::millis();
        screen_set(1, "ARM DOWN");
      }
      // X / Y / LEFT / RIGHT are not test buttons here. They still counted as
      // "any button" above, so they work as abort keys and do nothing else.
      // 中文：X／Y／左／右在這一版不是測試鍵。它們仍然算在「任何按鍵」裡，
      // 所以只當中止鍵用，其他什麼都不做。
    }
    else {
      // ---- plain arcade driving, so the robot can be repositioned ----------
      // Only ever reached with no test running, so this and the worker task can
      // never write to the drive motors at the same time.
      // 中文：只有在「沒有測試在跑」的時候才會走到這裡，所以這裡跟 worker task
      // 不可能同時寫同一組馬達。
      if(fabs(throttle) < TUNE_STICK_DEADBAND) throttle = 0;
      if(fabs(turn) < TUNE_STICK_DEADBAND) turn = 0;
      chassis.DriveL.move(throttle + turn);
      chassis.DriveR.move(throttle - turn);
    }

    pros::delay(TUNE_LOOP_MS);
  }
}

#endif // PID_TUNE_PROGRAM
