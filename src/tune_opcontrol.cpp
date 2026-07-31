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
//   B     滑軌到 LEVEL0      tune_cascade_goto(CASCADE_LV0_DEG，預設 0＝收起)
//   Y     滑軌到 LEVEL1      tune_cascade_goto(CASCADE_LV1_DEG，預設 300)
//   X     滑軌到 LEVEL2      tune_cascade_goto(CASCADE_LV2_DEG，預設 1000)
//   A     滑軌到 LEVEL3      tune_cascade_goto(CASCADE_LV3_DEG，預設 2000)
//         （四段高度都是 dashboard 滑桿 cascade/presets 群組，可線上調）
//   ── 手臂四位置（方向鍵，走梯形軌跡）──────────────────────────────
//   UP    手臂到 POS_1       最高（預設 283°）
//   RIGHT 手臂到 POS_3       高工作位（預設 265°）
//   LEFT  手臂到 POS_2       低工作位（預設 160°）
//   DOWN  手臂到 DOWN        收起（預設 0°）
//         （四個角度就是既有的 arm/presets 四顆滑桿 DOWN／POS_2／POS_3／POS_1，
//          在 dashboard 上直接拉；沒有另外開一套「arm_POS0~3」，那會變成同一個
//          角度有兩個地方要改，登記表也塞不下——詳見 main.cpp 的說明）
//         這四顆走 arm_move_profiled()：命令角度沿「加速→等速→減速」的梯形走，
//         形狀由 arm/profile 的 arm_vel／arm_acc／arm_dec 三顆滑桿決定。
//         其他所有手臂路徑（駕駛版的 RIGHT/LEFT 預設動作、自走）照舊直達，沒變。
//   左搖桿 Y／右搖桿 X       正常方向盤式駕駛（沒有測試動作在跑的時候才有效），
//                            用來把車開回起點，不必用手搬
//
//   ── 前饋斜坡測試（沒有按鍵，從 dashboard 觸發）────────────────────
//   dashboard「量前饋」面板上的兩顆按鈕：ff_ramp_arm（手臂）、ff_ramp_cascade
//   （滑軌）。按下去會先跳確認（requires_confirm），確認後車端把電壓慢慢往一個
//   方向加、再往另一個方向加，過程中送出 tune_ms／tune_volt／tune_pos／tune_vel／
//   tune_seg 五條頻道給面板去算前饋值。
//   安全：電壓上限 60/127 且是慢慢加、逼近行程末端就結束該段、每段與整場都有逾時、
//   按任何一顆按鍵或推搖桿即中止、比賽狀態一變看門狗就收掉。測試期間該機構的控制器
//   會先讓位（arm_control_set_enabled／cascade_control_set_enabled），結束後自動
//   交還並就地停住。已經有東西在動的時候按面板按鈕：直接丟掉不排隊。
//
// 中止：**動作進行中按任何一顆按鍵**（同一顆也算）就中止——遙控器震一下、
// 底盤電壓歸零、滑軌／手臂就地停住。搖桿大幅推動（超過 25／127）也會中止。
//
// 方向鍵拿去當手臂鍵之後，「任意鍵中止」還在嗎？在，而且一個字都沒改：
// 「有測試在跑」的分支排在「閒置時按鍵啟動測試」之前，任何一顆新按下的鍵——包含
// 這四顆方向鍵——在那個分支就被吃掉當成中止，根本走不到啟動測試那一段。所以：
//   測試進行中按方向鍵 ＝ 純中止，手臂不會動
//   閒置時按方向鍵     ＝ 手臂去該位置
// 這跟滑軌 B/Y/X/A 是同一套規則（正在跑的先取消，要再下一個命令就再按一次），
// 遙控器螢幕第三行也會照狀態切換：閒置顯示 UDLR=ARM POS，測試中顯示 ANY KEY=ABORT。
//
// ---------------------------------------------------------------------------
// 機構保護：跟駕駛版同一套
// ---------------------------------------------------------------------------
// 這一版跑的是**同一支** arm_task() 與 cascade_task()，所以手臂軟行程夾限
// （arm.cpp:141 / hold 走 arm.cpp:176）、手臂下降降壓（arm.cpp:207）、滑軌上下
// 分別限壓（cascade.cpp:176-177）全部照舊生效，不用也不該在這裡重做。
// 另外兩層是這個檔案自己補的，因為它們原本長在 Drive::control_arcade() 裡：
//   - tune_service_cascade_limit()：限位開關歸零＋自癒，主迴圈每圈跑
//     （逐行照抄 drive.cpp:944-953，含 Z1 補的 cascade_notify_tare()）
//   - tune_cascade_goto()：所有滑軌命令都先夾到 [0, CASCADE_EXTEND_LIMIT_DEG]，
//     用的是 Template/cascade.h 那一份共用常數，沒有另抄數字
//
// ---------------------------------------------------------------------------
// 手臂四個位置的角度是哪來的
// ---------------------------------------------------------------------------
// 手臂走既有的 arm_set_position/POS 慣例，現有的四個 preset 就是四個位置：
// DOWN=0°、POS_2=160°、POS_3=265°、POS_1=283°（見 src/Template/arm.cpp），
// 由低到高剛好對上 DOWN／LEFT／RIGHT／UP 四顆方向鍵。
// 要別的角度（例如教練要的 120°／10°）不用改程式：這四個數字本身就是 dashboard
// 的滑桿（arm/presets 群組），在 dashboard 上把 POS_2 拉到 120、DOWN 拉到 10，
// 那兩顆按鍵就會跑到 120／10。
//
// ---------------------------------------------------------------------------
// 梯形怎麼調
// ---------------------------------------------------------------------------
// 順序：先把增益調到「跟得上」，再調梯形決定「要它跑多快」。
//   1. arm_vel 先放小（例如 60 deg/s），按一顆方向鍵，看 Graph 上的
//      arm_setpoint（梯形要求的角度）與 arm_angle（實際角度）。
//   2. 兩條線幾乎重疊＝跟得上，可以把 arm_vel／arm_acc 往上加。
//   3. 兩條線分開＝手臂追不上梯形。這時候該加的是增益（或 arm_kG），不是把梯形
//      拉更快——梯形拉快只會讓差距更大。
//   4. 停下來會晃就把 arm_dec 調小（減速更早開始）。
// 手臂落後超過 25° 時梯形會自己停住不再往前（arm.cpp 的 ARM_PROFILE_MAX_LAG_DEG），
// 所以卡住的時候 PID 不會對著一個一路跑掉的命令角度死推。
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

// Cascade four-level height targets (N1): B=LEVEL0, Y=LEVEL1, X=LEVEL2,
// A=LEVEL3. NOT hardcoded at the buttons -- these are dashboard sliders
// (registered in main.cpp initialize(), "cascade/presets" group, same
// convention as the arm presets), so the heights are tuned live. Defaults
// follow the ScoringLevel enum that autonomous score() already uses
// (auton-routines.h: LEVEL_0=0 .. LEVEL_3=2000) -- the same motor-degree
// frame the teleop cascade PID runs in, with LEVEL0 = fully retracted.
// External linkage on purpose: main.cpp registers them via tune_opcontrol.h.
// 中文：四段高度鍵的目標值。不寫死在按鍵上——它們是 dashboard 滑桿（main.cpp
// 的 initialize() 登記，cascade/presets 群組，跟手臂 preset 同一套慣例），
// 高度可以線上調。預設值取自走 score() 已經在用的 ScoringLevel 列舉
// （LEVEL_0=0～LEVEL_3=2000，跟遙控滑軌 PID 同一套馬達角度座標），
// LEVEL0＝完全收起。
double CASCADE_LV0_DEG = 0;     // B  -- fully retracted / bottom 收起
double CASCADE_LV1_DEG = 300;   // Y
double CASCADE_LV2_DEG = 1000;  // X
double CASCADE_LV3_DEG = 2000;  // A

// The five channels the dashboard's feedforward panel looks for by exact name.
// See tune_opcontrol.h for the naming contract and for why tune_volt carries
// move() command units instead of volts. External linkage: main.cpp registers
// them.
// 中文：dashboard「量前饋」面板照固定名字找的那五條頻道。命名規則、以及 tune_volt
// 為什麼帶的是 move() 指令刻度而不是伏特，都寫在 tune_opcontrol.h。這裡用外部連結，
// 因為登記的動作在 main.cpp。
double TUNE_CH_MS = 0;
double TUNE_CH_VOLT = 0;
double TUNE_CH_POS = 0;
double TUNE_CH_VEL = 0;
std::int32_t TUNE_CH_SEG = 0;

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

// Backstop for a profiled arm move. TUNE_ARM_TIMEOUT_MS above only measures the
// PID settling AFTER the trapezoid has finished (a slow arm_vel can take longer
// than 5 s to cross the whole travel, and that is not a fault). This one is
// measured from the button press and covers the case the other cannot see: a
// profile frozen by the lag guard because the arm is jammed, which would
// otherwise sit there "running" until someone noticed.
// 中文：走梯形的手臂動作的最後一道保險。上面的 TUNE_ARM_TIMEOUT_MS 只計算「梯形走完
// 之後 PID 收尾」那段（arm_vel 調慢的話，走完整支行程超過 5 秒是正常的，不是故障）。
// 這一個是從按下按鍵開始算，補的是另一個看不到的情況：手臂卡住、梯形被落後保護凍住，
// 不然它會一直掛在「測試進行中」等人發現。
constexpr int TUNE_ARM_TOTAL_TIMEOUT_MS = 20000;

// Stick deflection that counts as "the driver wants the robot back".
constexpr int TUNE_STICK_ABORT = 25;
constexpr int TUNE_STICK_DEADBAND = 5;

pros::Controller tune_master(pros::E_CONTROLLER_MASTER);

// --- feedforward ramp test ---------------------------------------------------
// A slow, capped voltage ramp in one direction and then the other, with the
// five tune_* channels published every cycle. The dashboard panel does the maths
// on that data; this side only has to produce it safely.
//
// Three layers of protection, deliberately independent of each other, so no
// single wrong number can let the ramp hurt the robot:
//   1. Ceiling + slope: the command never exceeds TUNE_FF_MAX_CMD (well under
//      the 127 the mechanism can take) and gets there slowly, so anything going
//      wrong develops at walking pace instead of instantly.
//   2. Travel limits: the leg ends as soon as the mechanism comes within a
//      margin of either end of its travel. Checked against the SAME position
//      source the controllers use, so it cannot disagree with them.
//   3. Clocks: a per-leg timeout and a total timeout, either of which ends the
//      test even if position readings have stopped making sense.
// Plus the two the tuning program already had: any button or a stick push
// aborts, and the competition watchdog ends it if the match state changes.
// 中文：前饋斜坡測試——把電壓慢慢往一個方向加、再往另一個方向加，過程中每一圈把五條
// tune_* 頻道送出去。計算是 dashboard 面板在做，車端只要負責「安全地把資料生出來」。
// 三層保護，而且刻意彼此獨立，任何一個數字寫錯都不足以讓斜坡弄壞機構：
//   1. 上限＋斜率：指令永遠不超過 TUNE_FF_MAX_CMD（遠低於機構吃得下的 127），而且是
//      慢慢加上去的，出事也是用走的速度發生，不是一瞬間。
//   2. 行程夾限：機構一接近行程兩端的安全邊界就結束這一段。用的是跟控制器同一個位置
//      來源，所以不可能兩邊講的位置不一樣。
//   3. 計時：每一段有逾時、整場也有總逾時，就算位置讀數整個壞掉也一定會結束。
// 再加上調參程式本來就有的兩層：按任何鍵或推搖桿即中止、比賽狀態一變看門狗就收掉。
enum class FfTarget { NONE, ARM, CASCADE };

constexpr float TUNE_FF_MAX_CMD = 60.0f;        // out of 127 中文：127 分之
constexpr float TUNE_FF_RAMP_RATE = 12.0f;      // command units per second 中文：每秒加多少
constexpr int TUNE_FF_SEG_TIMEOUT_MS = 8000;
constexpr int TUNE_FF_TOTAL_TIMEOUT_MS = 20000;
// How close to the end of travel the mechanism may get before the leg is ended.
// 中文：機構最多可以逼近行程末端到什麼程度，超過就結束這一段。
constexpr float TUNE_FF_ARM_MARGIN_DEG = 15.0f;
constexpr float TUNE_FF_CASCADE_MARGIN_DEG = 200.0f;
// Velocity is a difference of two positions 10 ms apart, which is nearly all
// sensor noise on its own. This one-pole filter is what makes the curve
// readable; it lags by a few cycles, which does not matter for a ramp this slow.
// 中文：速度是相隔 10ms 的兩個位置相減，單看幾乎全是感測雜訊。這個一階濾波是讓曲線
// 看得懂的關鍵；代價是慢個幾圈，對這麼慢的斜坡完全無所謂。
constexpr float TUNE_FF_VEL_FILTER = 0.25f;

// Raised by the dashboard command handlers, which run on the vexdash pump task.
// 0 = nothing pending, 1 = arm, 2 = cascade. volatile for the same reason the
// chassis handshake flags are: one writer, one reader, 10 ms polling.
// 中文：由 dashboard 命令的處理函式舉起來的旗標，那些函式跑在 vexdash 的背景 task。
// 0＝沒事、1＝手臂、2＝滑軌。用 volatile 的理由跟底盤那組握手旗標一樣：一個寫、一個
// 讀、10ms 輪詢。
volatile int ff_request = 0;

FfTarget ff_target = FfTarget::NONE;
float ff_cmd = 0;
float ff_vel = 0;
float ff_last_pos = 0;
std::uint32_t ff_start_ms = 0;
std::uint32_t ff_seg_start_ms = 0;

// --- which test is running ---------------------------------------------------
enum class ActiveTest { NONE, CHASSIS, ARM, CASCADE, FF_RAMP };
enum class ChassisMove { NONE, DRIVE_FWD, DRIVE_BACK, TURN_LEFT, TURN_RIGHT };

ActiveTest active_test = ActiveTest::NONE;
std::uint32_t test_started_ms = 0;
// When the arm key was pressed, kept separately because test_started_ms is held
// at "now" while a trapezoid runs (see the ARM branch of the loop).
// 中文：手臂鍵是什麼時候按的。要另外記，因為梯形在跑的時候 test_started_ms 會一直
// 被押在「現在」（見迴圈裡的 ARM 分支）。
std::uint32_t arm_test_started_ms = 0;

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

// --- the driving program's cascade protections, verbatim --------------------
//
// Send the cascade somewhere, clamped to the SAME travel limit the driving
// program uses. CASCADE_EXTEND_LIMIT_DEG comes from Template/cascade.h -- the
// one copy the teleop buttons and the controller already share -- so this is
// not a second number that can drift away from it.
//
// cascade_set_target() clamps to exactly this range on its own
// (src/Template/cascade.cpp:59), so this is belt and braces rather than the
// only guard. It is here so the clamp is visible at the point where a tuning
// button chooses a target: the B/Y/X/A level heights are dashboard sliders,
// so someone can drag one past the travel limit -- the clamp catches that
// right here, not only two files away.
// 中文：把滑軌送到某個位置，並且夾在**跟駕駛版同一個**行程上限
// （CASCADE_EXTEND_LIMIT_DEG 來自 Template/cascade.h，就是遙控按鍵與控制器共用的
// 那一份，不是另外抄一個數字）。cascade_set_target() 本身就已經夾同一個範圍
// （cascade.cpp:59），所以這裡是多一層保險；放在「按鍵決定目標」的地方是為了
// 讓夾限看得見——B/Y/X/A 的四段高度是 dashboard 滑桿，有人把滑桿拉超過行程
// 上限的話，這裡一樣擋得住。
void tune_cascade_goto(float deg){
  cascade_set_target(clamp(deg, 0.0f, CASCADE_EXTEND_LIMIT_DEG));
}

// The limit switch on ADI 'D' reads 1 when pressed. Every time it triggers,
// re-zero both cascade encoders so the physical hard stop is always "0 degrees"
// -- this is what corrects encoder drift picked up over a long session.
//
// This is Drive::control_arcade()'s block (src/Template/drive.cpp:944-953)
// copied verbatim, INCLUDING Z1's cascade_notify_tare(): the controller's target
// is in the same frame that was just re-zeroed so nothing needs re-aiming, but
// the PID's accumulated and previous error must be cleared or the position jump
// reads as a fake error spike in the I and D terms.
//
// The tuning program used to be the one place without this, which meant the
// cascade's zero could drift over a long tuning session and every cascade
// number on the graph would quietly be measured from the wrong origin.
// 中文：ADI 'D' 的限位開關壓到會讀 1。每次壓到就把兩顆滑軌編碼器歸零，讓物理底部
// 永遠等於「0 度」——這就是長時間調參後修正編碼器漂移的機制。
// 這段是 Drive::control_arcade()（drive.cpp:944-953）原封不動搬過來的，**包含 Z1
// 補的 cascade_notify_tare()**：控制器的目標跟編碼器是同一套座標，歸零後不用重設
// 目標，但 PID 的積分與「上一次誤差」一定要清掉，不然位置突跳會被 I／D 當成真的
// 誤差爆一下。
// 原本調參版是唯一沒有這段的地方，代表長時間調參後滑軌的原點會漂掉，圖表上每一個
// cascade 數字都是從錯的原點量出來的。
// Defined with the ramp code below. Declared here because the limit switch is
// serviced above it and has to tell it about the position jump.
// 中文：定義在下面的斜坡那一段。因為限位開關的處理在它上面，而那裡必須通知它「位置
// 剛剛跳了」，所以先在這裡宣告。
void ff_notify_position_jump();

void tune_service_cascade_limit(){
  if(cascade_limit.get_value() == 1){
    cascade1.tare_position();
    cascade2.tare_position();
    cascade_notify_tare();
    // The cascade's zero just moved, so every consumer holding a position from
    // BEFORE the tare is now comparing two different coordinate frames. The PID
    // is handled by cascade_notify_tare() above; the feedforward ramp keeps its
    // own previous position to difference for velocity, and left stale it would
    // produce one sample of tens of thousands of deg/s -- a single outlier that
    // is more than enough to bend the curve fit the whole test exists to feed.
    // 中文：滑軌的零點剛剛被移走，所以任何還握著「歸零之前」位置的人，現在都是拿兩套
    // 座標在比。PID 那邊上面的 cascade_notify_tare() 已經處理掉了；前饋斜坡自己也留著
    // 上一圈的位置在算速度，不通知它的話會生出一筆好幾萬 deg/s 的樣本——這一筆離群值
    // 就足以把整個測試辛苦要餵的那條擬合線拉歪。
    ff_notify_position_jump();
  }
}

// --- feedforward ramp: mechanism-agnostic plumbing ---------------------------
// Position comes from the SAME function the mechanism's own controller uses, so
// the ramp's travel-limit checks and the controller's can never disagree about
// where the mechanism is. For the arm that is the rotation sensor on port 21
// (arm_get_position_deg() -- the arm controller does not read the motor's
// encoder at all), which is also what tune_pos and tune_vel below are computed
// from, so the feedforward number the dashboard works out belongs to the same
// feedback the PID will use it with.
// 中文：位置一律取自「機構自己的控制器用的那個函式」，這樣斜坡的行程判斷跟控制器的
// 判斷不可能對不上。手臂那邊就是埠 21 的 rotation 感測器（arm_get_position_deg()——
// 手臂控制器根本不讀馬達編碼器），下面的 tune_pos／tune_vel 也是從同一個來源算出來
// 的，所以 dashboard 算出來的前饋值，跟之後 PID 搭配使用的回饋是同一套。
float ff_position(){
  return ff_target == FfTarget::ARM ? arm_get_position_deg()
                                    : cascade_get_position_deg();
}

float ff_limit_lo(){
  return ff_target == FfTarget::ARM ? ARM_MIN_DEG + TUNE_FF_ARM_MARGIN_DEG
                                    : TUNE_FF_CASCADE_MARGIN_DEG;
}

float ff_limit_hi(){
  return ff_target == FfTarget::ARM
             ? ARM_MAX_DEG - TUNE_FF_ARM_MARGIN_DEG
             : CASCADE_EXTEND_LIMIT_DEG - TUNE_FF_CASCADE_MARGIN_DEG;
}

// Something re-zeroed the encoder the ramp is measuring. Re-anchor the previous
// position so the next velocity sample is a real movement instead of the size of
// the coordinate shift. Velocity itself is untouched: the mechanism did not
// actually move, only the numbering did.
// 中文：有人把斜坡正在量的那顆編碼器歸零了。這裡把「上一圈的位置」重新對準，好讓下一
// 筆速度算的是真正的位移，不是座標平移的大小。速度本身不動：機構其實沒有動，動的只是
// 編號方式。
void ff_notify_position_jump(){
  if(ff_target == FfTarget::NONE) return;
  ff_last_pos = ff_position();
}

void ff_write(float cmd){
  if(ff_target == FfTarget::ARM){
    arm.move(cmd);
  }
  else if(ff_target == FfTarget::CASCADE){
    cascade1.move(cmd);
    cascade2.move(cmd);
  }
}

// Put the mechanism back under its own controller and stop feeding the panel.
// Safe to call twice (the abort path and the "finished" path can both reach it).
// 中文：把機構交還給它自己的控制器，並停止餵資料給面板。呼叫兩次也安全（中止流程跟
// 正常結束流程都會走到這裡）。
void ff_ramp_finish(){
  ff_write(0);
  ff_cmd = 0;
  TUNE_CH_VOLT = 0;
  TUNE_CH_SEG = 0;   // 0 = not running, the panel stops collecting 中文：0＝沒在跑

  if(ff_target == FfTarget::ARM){
    // Re-enabling parks the arm on its current angle (arm.cpp), so it holds
    // instead of dropping the moment the ramp stops pushing.
    // 中文：重新啟用會把手臂停在它現在的角度（見 arm.cpp），所以斜坡一停手，手臂是
    // 撐住、不是掉下去。
    arm_control_set_enabled(true);
  }
  else if(ff_target == FfTarget::CASCADE){
    cascade_set_target(cascade_get_position_deg());
    cascade_notify_tare();
    cascade_control_set_enabled(true);
  }
  ff_target = FfTarget::NONE;
}

void ff_ramp_start(FfTarget target, const char* label){
  ff_target = target;
  // Take the motor away from its controller FIRST. Two writers on one motor is
  // two people steering: the ramp would be fighting the PID for every cycle and
  // the voltage on the wire would be neither one's.
  // 中文：先把馬達從它的控制器手上接過來。同一顆馬達兩個人在寫＝兩個人一起轉方向盤，
  // 斜坡會跟 PID 每一圈互搶，實際上線的電壓誰的也不是。
  if(target == FfTarget::ARM) arm_control_set_enabled(false);
  else                        cascade_control_set_enabled(false);

  ff_cmd = 0;
  ff_vel = 0;
  ff_write(0);
  ff_start_ms = pros::millis();
  ff_seg_start_ms = ff_start_ms;
  ff_last_pos = ff_position();

  TUNE_CH_MS = 0;
  TUNE_CH_VOLT = 0;
  TUNE_CH_POS = ff_last_pos;
  TUNE_CH_VEL = 0;
  TUNE_CH_SEG = 1;   // leg 1 = forward 中文：第 1 段＝正向

  active_test = ActiveTest::FF_RAMP;
  test_started_ms = ff_start_ms;
  screen_set(1, label);
}

// One 10 ms step. Returns false when the test is over (either leg finished, or
// a protection ended it) -- the caller then calls ff_ramp_finish().
// 中文：一個 10ms 步進。回傳 false＝測試結束（兩段都跑完，或是被某一層保護收掉），
// 呼叫端接著呼叫 ff_ramp_finish()。
bool ff_ramp_step(){
  const float dt = TUNE_LOOP_MS / 1000.0f;
  const std::uint32_t now = pros::millis();

  float pos = ff_position();
  float raw_vel = (pos - ff_last_pos) / dt;
  ff_last_pos = pos;
  ff_vel += TUNE_FF_VEL_FILTER * (raw_vel - ff_vel);

  // Publish BEFORE any early return, so the last sample the panel sees is the
  // one that ended the test rather than a stale cycle.
  // 中文：先送資料再做任何提早結束的判斷，這樣面板看到的最後一筆就是「結束當下」那一
  // 筆，不是上一圈的舊資料。
  TUNE_CH_MS = (double)(now - ff_start_ms);
  TUNE_CH_POS = pos;
  TUNE_CH_VEL = ff_vel;
  // tune_volt is NOT published here: this cycle's command has not been worked
  // out yet, so publishing now would pair every position/velocity sample with
  // the PREVIOUS cycle's voltage -- a constant one-sample lag between the two
  // columns the panel regresses against each other. It is published right after
  // ff_write() below, in the same pass that puts it on the motor. On the early
  // returns between here and there it keeps the value that really was on the
  // motor for the last cycle, which is still the honest answer.
  // 中文：tune_volt 不在這裡送：這一圈的指令還沒算出來，現在送等於把每一筆位置／速度
  // 都配上「上一圈」的電壓——面板拿來互相回歸的那兩欄之間會固定差一筆。它改成在下面
  // ff_write() 之後、跟寫進馬達同一圈送出。從這裡到那裡之間的提早結束路徑，它保留的是
  // 上一圈真正加在馬達上的值，那依然是誠實的答案。

  // Protection 3a: no trustworthy arm position means no travel-limit protection
  // at all, so there is no safe way to keep pushing.
  // 中文：手臂位置讀不到＝行程保護整層失效，那就沒有「繼續推下去」的安全做法了。
  if(ff_target == FfTarget::ARM && !arm_sensor_ok) return false;

  // Protection 3b: total clock.
  if(now - ff_start_ms > (std::uint32_t)TUNE_FF_TOTAL_TIMEOUT_MS) return false;

  const bool forward = (TUNE_CH_SEG == 1);
  bool leg_done = false;

  if(forward){
    ff_cmd += TUNE_FF_RAMP_RATE * dt;
    if(ff_cmd > TUNE_FF_MAX_CMD) ff_cmd = TUNE_FF_MAX_CMD;
    leg_done = ff_cmd >= TUNE_FF_MAX_CMD || pos >= ff_limit_hi();
  }
  else{
    ff_cmd -= TUNE_FF_RAMP_RATE * dt;
    if(ff_cmd < -TUNE_FF_MAX_CMD) ff_cmd = -TUNE_FF_MAX_CMD;
    leg_done = ff_cmd <= -TUNE_FF_MAX_CMD || pos <= ff_limit_lo();
  }
  if(now - ff_seg_start_ms > (std::uint32_t)TUNE_FF_SEG_TIMEOUT_MS) leg_done = true;

  ff_write(ff_cmd);
  // Same pass as the write, so the sample the panel receives has the voltage
  // that is on the motor next to the position and velocity it produced.
  // 中文：跟寫馬達同一圈送出，這樣面板收到的那一筆裡，電壓就是「現在真的加在馬達上
  // 的那個」，跟它造成的位置與速度擺在一起。
  TUNE_CH_VOLT = ff_cmd;

  if(!leg_done) return true;
  if(!forward) return false;   // both legs done 中文：兩段都跑完了

  // Forward leg finished: hand over to the reverse leg from zero command, with
  // its own clock and its own velocity history.
  // 中文：正向那段跑完，交棒給反向那段：指令從 0 重新開始，計時與速度歷史也重來。
  ff_cmd = 0;
  ff_write(0);
  ff_vel = 0;
  ff_seg_start_ms = now;
  TUNE_CH_SEG = 2;   // leg 2 = reverse 中文：第 2 段＝反向
  TUNE_CH_VOLT = 0;
  return true;
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
      // leaning on a target it was told to stop chasing. Clamped like every
      // other cascade command -- if the lift has been pushed past the limit by
      // hand, "hold where you are" must still resolve to a legal target.
      // 中文：把滑軌停在現在的位置，PID 就地撐住，不會對著一個已經放棄的目標死推。
      // 跟其他所有滑軌命令一樣要夾限——萬一機構被人推超過上限，「停在原地」也必須
      // 收斂成一個合法的目標。
      tune_cascade_goto(cascade_get_position_deg());
      break;
    case ActiveTest::ARM:
      arm_hold_here();
      break;
    case ActiveTest::FF_RAMP:
      // Zero the command, give the motor back to its controller, and stop
      // feeding the panel -- all three are in ff_ramp_finish().
      // 中文：把指令歸零、把馬達還給它的控制器、停止餵資料給面板——三件事都在
      // ff_ramp_finish() 裡。
      ff_ramp_finish();
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
// It deliberately does not touch the arm or the cascade while they are under
// their own controllers: during autonomous the team's own routines own those
// two, and disabled() in main.cpp already parks the cascade.
//
// A FEEDFORWARD RAMP is the exception, and it is the one case where doing
// nothing is not "leaving it to the other program" but a robot left powered.
// The ramp is not a controller with a target -- it is a raw voltage written to a
// motor by the opcontrol task, plus that mechanism's controller switched OFF.
// Field control deletes opcontrol without warning, and everything the ramp does
// on the way out lives in that deleted task, so without this branch:
//   * the last command written (up to 60/127) stays on the arm or the lift,
//     because nothing ever writes to those motors again, and
//   * arm_control_enabled_flag stays false forever, so arm_task() never touches
//     the arm again either -- autonomous arm_set_position() calls would set a
//     target that nobody drives to, silently, until the robot is rebooted.
// ff_ramp_finish() is idempotent (it zeroes the command, hands the motor back
// and clears ff_target), so calling it here races safely with the main loop
// calling it for the same test.
// 中文：比賽一被 disable 或進入自走，場控會直接砍掉 opcontrol，但**不會**砍上面那支
// worker——它會若無其事繼續開車。這支看門狗永遠在跑，只要不在遙控期就立刻把動作掐掉。
// 它刻意不碰「還在自己控制器手上」的手臂與滑軌：自走期間那兩個是自走程式在管，
// disabled() 本來就已經把滑軌控制器停掉了。
// 前饋斜坡是例外，而且是「什麼都不做」不等於「交給別的程式」、而等於「把機器人通著電
// 丟在那裡」的唯一情況。斜坡不是一個有目標的控制器，它是 opcontrol task 直接寫給馬達
// 的生電壓，外加那個機構的控制器被關掉。場控砍 opcontrol 不會先講，而斜坡所有的善後
// 都寫在那支被砍掉的 task 裡，所以沒有這個分支的話：
//   * 最後寫下去的指令（最高 60/127）會一直留在手臂或升降上，因為之後再也沒有人寫那
//     兩顆馬達，而且
//   * arm_control_enabled_flag 會永遠停在 false，arm_task() 從此不再碰手臂——自走呼叫
//     arm_set_position() 只會設一個沒人去開的目標，安靜地失效到重開機為止。
// ff_ramp_finish() 是冪等的（指令歸零、把馬達交還、清掉 ff_target），所以這裡跟主迴圈
// 同時對同一次測試呼叫它也是安全的。
void tune_safety_task(){
  while(true){
    if(pros::competition::is_disabled() || pros::competition::is_autonomous()){
      if(move_running || move_requested) abort_chassis_move();
      if(ff_target != FfTarget::NONE) ff_ramp_finish();
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

// Send the arm to one of its four presets along the trapezoid, and register it
// as the running test so the abort path and the DONE/TIMEOUT logic already in
// the loop apply to it unchanged -- exactly like the cascade level keys.
// 中文：把手臂沿梯形送到四個預設位置之一，並登記成「正在跑的測試」，這樣迴圈裡
// 既有的中止流程、到位／逾時判斷就原封不動地套用在它身上——跟滑軌四段鍵一模一樣。
void tune_arm_goto(ArmPosition pos, const char* label){
  arm_move_profiled(pos);
  active_test = ActiveTest::ARM;
  test_started_ms = pros::millis();
  arm_test_started_ms = test_started_ms;
  screen_set(1, label);
}

} // namespace

// --- dashboard command handlers ---------------------------------------------
// Called by the vexdash pump task when someone presses the panel's button and
// confirms. They do NOT move anything: they raise a request that the main loop
// picks up on its next 10 ms pass, and only when nothing else is running. See
// tune_opcontrol.h for why a background task must not start a motion directly.
// 中文：使用者在面板上按下按鈕並確認之後，由 vexdash 的背景 task 呼叫。它們不會讓任何
// 東西動起來：只是舉一個旗標，由主迴圈在下一個 10ms 迴圈、而且是在「什麼都沒在跑」的
// 情況下才接手。為什麼背景 task 不可以直接讓機器人動，見 tune_opcontrol.h。
void tune_ff_ramp_arm_command(void* /*user_data*/){
  ff_request = 1;
}

void tune_ff_ramp_cascade_command(void* /*user_data*/){
  ff_request = 2;
}

void tune_opcontrol(){
  save_voltage_caps();
  // Teleop can start again after field control killed this task mid-abort, with
  // the voltage caps still pinned at 0. Putting them back here is the one place
  // that is guaranteed to run before any new test.
  // 中文：場控有可能在「中止進行到一半」的時候把這支 task 砍掉，那時電壓上限還
  // 停在 0；下一段遙控期一定會先跑到這裡，所以在這裡把它們放回去最保險。
  restore_voltage_caps();
  // Same story for a test that was running when the task died: the watchdog
  // already made the machine safe, but the state variables can still say "mid
  // FF ramp", which would eat keys for up to a segment timeout. Fresh teleop,
  // fresh state. 中文：同理，task 被砍時若有測試在跑，看門狗已把機器收乾淨，但
  // 狀態變數可能還停在「斜坡進行中」，會白吃按鍵直到段逾時——重進遙控就重設。
  active_test = ActiveTest::NONE;
  ff_target = FfTarget::NONE;

  // Same teleop hand-over the normal driving loop does (see control_arcade()):
  // zero the cascade encoders at the physical bottom the robot was placed at,
  // then let the cascade PID hold position 0. Without this the cascade
  // controller stays disabled and the B/Y/X/A level keys would do nothing.
  // 中文：跟正常駕駛開場做同一件事（見 control_arcade()）：把滑軌編碼器在「機器人
  // 現在擺放的物理底部」歸零，再讓滑軌 PID 撐在 0。不做的話滑軌控制器是關著的，
  // B／Y／X／A 四段高度鍵按了不會動。
  chassis.drive_stop(MotorBrake::coast);
  cascade1.tare_position();
  cascade2.tare_position();
  cascade_notify_tare();
  tune_cascade_goto(0);
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
  // Line 2 is now a live hint, rewritten every pass of the loop below: it says
  // what the D-pad does when idle and what ANY key does while a test runs. That
  // matters more on a 15-character screen than the old static chassis-key list
  // (which is in the key table at the top of this file), because the D-pad is
  // the one place where the same button means two different things.
  // 中文：第三行改成會跟著狀態變的提示，由下面的迴圈每圈重寫：閒置時講方向鍵是什麼、
  // 測試進行中講「按任何鍵＝中止」。在一行只有 15 個字的螢幕上，這比原本那串固定的
  // 底盤按鍵表更該佔位置（底盤按鍵表在本檔開頭的按鍵表裡），因為方向鍵是唯一一個
  // 「同一顆鍵在兩種狀態下意思不同」的地方。
  screen_set(2, "UDLR=ARM POS");

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

    // Live hint line. screen_set() only records what SHOULD be on screen and
    // screen_pump() pushes a line only when it actually changed, so rewriting
    // it every 10 ms costs nothing on the wire.
    // 中文：狀態提示行。screen_set() 只是記下「應該顯示什麼」，screen_pump() 只在
    // 真的變了才送出去，所以每 10ms 重寫一次完全不佔傳輸。
    screen_set(2, active_test != ActiveTest::NONE ? "ANY KEY=ABORT" : "UDLR=ARM POS");

    // ---- competition lockout ------------------------------------------------
    if(competition_lockout()){
      if(active_test != ActiveTest::NONE) abort_active_test(false);
      // Drop any dashboard request too: a button pressed during a match must not
      // be waiting to fire the moment teleop comes back.
      // 中文：dashboard 的請求也一併丟掉——比賽中按到的按鈕，不可以留在那裡等遙控期
      // 一恢復就自己啟動。
      ff_request = 0;
      screen_set(1, "COMP LOCK");
      last_any = false;
      pros::delay(TUNE_LOOP_MS);
      continue;
    }

    // ---- cascade limit switch self-heal, every cycle ------------------------
    // Placed here, after the competition lockout and BEFORE the "a test is
    // running" branch below, so it runs on every path teleop can take -- idle,
    // driving on the sticks, and mid-test -- exactly like control_arcade()
    // runs it on every pass of its own loop. Skipped while disabled or in
    // autonomous for the same reason control_arcade() is: it is not running
    // then either.
    // 中文：放在比賽鎖之後、「有測試在跑」那個分支之前，所以遙控期的每一條路徑
    // （閒置、用搖桿開車、測試進行中）每一圈都會跑到，跟 control_arcade() 在自己
    // 迴圈裡每一圈都做是一樣的。比賽被鎖住時不做，理由也一樣：那時候駕駛版的
    // control_arcade() 本來也沒在跑。
    tune_service_cascade_limit();

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
      // A dashboard button pressed while something is already moving is dropped,
      // not queued. Queuing it would mean the robot starts a ramp by itself some
      // seconds later, with nobody's hand near the controller -- the one thing a
      // remote-triggered motion must never do.
      // 中文：機構正在動的時候按 dashboard 按鈕，一律丟掉、不排隊。排隊的話等於機器人
      // 過幾秒之後自己開始跑斜坡，而那時候沒有人的手在遙控器旁邊——遠端觸發的動作最不
      // 該做的就是這件事。
      ff_request = 0;

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
        // The profile gave up: the arm could not keep up and has ALREADY been
        // parked on its own angle by arm.cpp, so nothing is being pushed. Report
        // it and end the test right now. This is the fast exit that stops a jam
        // sitting there until the 20 s backstop -- with it in place the backstop
        // really is a last resort rather than the normal way out of a stall.
        // 中文：梯形放棄了——手臂跟不上，而且 arm.cpp 已經把它停在自己的角度上，沒有
        // 在頂任何東西。這裡立刻回報並結束測試。有了這條快速出口，卡住就不用等到 20 秒
        // 的總逾時；那個總逾時因此退回成真正的「最後一道」保險。
        if(arm_profile_stalled()){
          abort_active_test(true);   // arm_hold_here() inside clears the flag
          screen_set(1, "ARM STALL");
          pros::delay(TUNE_LOOP_MS);
          continue;
        }

        // While the trapezoid is still walking the commanded angle, the arm is
        // doing exactly what it was told, so the settle timeout must not be
        // counting: a slow arm_vel can legitimately take longer than
        // TUNE_ARM_TIMEOUT_MS to cross the whole travel. Holding the clock at
        // "now" pauses that timeout, and the 30 ms staleness guard below then
        // starts from the moment the profile hands the arm over to the PID --
        // which is exactly what it is there for.
        // 中文：梯形還在走的時候，手臂是照著命令在做，所以收尾逾時不能在這時候計時：
        // arm_vel 調慢的話，走完整支行程本來就可能比 TUNE_ARM_TIMEOUT_MS 久。把計時
        // 起點一直押在「現在」＝把逾時暫停；而下面那個 30ms 的保護就會從「梯形交棒給
        // PID」的那一刻開始算——這正是它存在的目的。
        if(arm_profile_running()){
          test_started_ms = pros::millis();
          elapsed = 0;
        }

        // arm_settled is recomputed every 10 ms and stays stale (true for the
        // PREVIOUS target) for a moment after a new arm command -- same 30 ms
        // latency the preset sequences in drive.cpp allow for. It is measured
        // against the DESTINATION, not the moving profile point (see arm.cpp),
        // so it cannot go true in the middle of a profiled move.
        // 中文：arm_settled 每 10ms 重算一次，新命令剛下的瞬間它還是「上一個目標」的
        // 答案，所以等 30ms——跟 drive.cpp 的預設動作同一個數字。它是拿「終點」量的、
        // 不是拿梯形移動中的那一點量的（見 arm.cpp），所以不會在動作中途變成 true。
        if(elapsed > 30 && arm_settled){
          active_test = ActiveTest::NONE;
          screen_set(1, "ARM DONE");
        }
        else if(elapsed > (std::uint32_t)TUNE_ARM_TIMEOUT_MS ||
                pros::millis() - arm_test_started_ms >
                    (std::uint32_t)TUNE_ARM_TOTAL_TIMEOUT_MS){
          abort_active_test(true);
          screen_set(1, "ARM TIMEOUT");
        }
      }
      else if(active_test == ActiveTest::FF_RAMP){
        // The ramp owns the motor while this runs; ff_ramp_step() is where every
        // protection lives and it says when the test is over.
        // 中文：這段期間馬達歸斜坡管；所有保護都在 ff_ramp_step() 裡，什麼時候結束
        // 也由它說了算。
        if(!ff_ramp_step()){
          ff_ramp_finish();
          active_test = ActiveTest::NONE;
          screen_set(1, "FF DONE");
          tune_master.rumble(".");
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

    // ---- idle: a dashboard command can start the feedforward ramp ----------
    // Reached only when nothing is running (the branch above never falls
    // through), so a remote-triggered motion starts from exactly the same
    // standing-still state a button press would. The panel already made the user
    // confirm (requires_confirm), and the buzz is the robot-side warning that
    // something is about to move without anyone touching the controller.
    // 中文：只有在「什麼都沒在跑」的時候才會走到這裡（上面那個分支不會漏下來），所以
    // 遠端觸發的動作，起點跟人按按鍵一模一樣＝機構是靜止的。面板那邊已經要求使用者確認
    // 過（requires_confirm），這裡震一下則是車端的警告：沒有人碰遙控器，但東西要動了。
    if(ff_request != 0){
      int req = ff_request;
      ff_request = 0;
      tune_master.rumble("-");
      if(req == 1) ff_ramp_start(FfTarget::ARM,     "FF RAMP ARM");
      else         ff_ramp_start(FfTarget::CASCADE, "FF RAMP CASC");
      pros::delay(TUNE_LOOP_MS);
      continue;
    }

    // ---- idle: buttons start a test ----------------------------------------
    if(new_press){
      if(b_l1)        request_chassis_move(ChassisMove::DRIVE_FWD,  "FWD 50cm");
      else if(b_l2)   request_chassis_move(ChassisMove::DRIVE_BACK, "BACK 50cm");
      else if(b_r1)   request_chassis_move(ChassisMove::TURN_LEFT,  "TURN L90");
      else if(b_r2)   request_chassis_move(ChassisMove::TURN_RIGHT, "TURN R180");
      else if(b_b){
        tune_cascade_goto((float)CASCADE_LV0_DEG);
        active_test = ActiveTest::CASCADE;
        test_started_ms = pros::millis();
        screen_set(1, "CASC LV0");
      }
      else if(b_y){
        tune_cascade_goto((float)CASCADE_LV1_DEG);
        active_test = ActiveTest::CASCADE;
        test_started_ms = pros::millis();
        screen_set(1, "CASC LV1");
      }
      else if(b_x){
        tune_cascade_goto((float)CASCADE_LV2_DEG);
        active_test = ActiveTest::CASCADE;
        test_started_ms = pros::millis();
        screen_set(1, "CASC LV2");
      }
      else if(b_a){
        tune_cascade_goto((float)CASCADE_LV3_DEG);
        active_test = ActiveTest::CASCADE;
        test_started_ms = pros::millis();
        screen_set(1, "CASC LV3");
      }
      // The four arm positions, low to high: DOWN < POS_2 < POS_3 < POS_1,
      // laid out on the D-pad the way the arm actually moves (DOWN key = down).
      // The angles are the arm/presets sliders, so they are tuned on the
      // dashboard, not here. All four go through the trapezoid.
      // 中文：手臂四個位置由低到高：DOWN < POS_2 < POS_3 < POS_1，按鍵位置照著手臂
      // 實際的高低擺（往下的鍵＝往下）。角度是 arm/presets 那四顆滑桿，在 dashboard
      // 上調、不寫在這裡。四顆都走梯形。
      else if(b_up)    tune_arm_goto(ArmPosition::POS_1, "ARM UP POS_1");
      else if(b_right) tune_arm_goto(ArmPosition::POS_3, "ARM POS_3");
      else if(b_left)  tune_arm_goto(ArmPosition::POS_2, "ARM POS_2");
      else if(b_down)  tune_arm_goto(ArmPosition::DOWN,  "ARM DOWN");
      // Reaching this point at all means no test was running (if one had been,
      // the branch above would have aborted it and skipped the rest of the
      // loop), which is exactly why the D-pad can be both an abort key and a
      // position key without the two ever colliding.
      // 中文：能走到這一段就代表「沒有測試在跑」（有的話上面那個分支早就 continue
      // 掉了）。這正是方向鍵可以同時當中止鍵與位置鍵、而兩者永遠不會撞在一起的原因。
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
