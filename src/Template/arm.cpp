#include "main.h"

ArmPosition arm_target = ArmPosition::DOWN;

// Target angles in ARM degrees, straight from arm_rotation (port 21).
//
// To re-measure any of these:
//   1. Build and run. The arm holds wherever it starts, which becomes 0.
//   2. Move the arm by hand (or with the buttons) to the position you want.
//   3. Read "arm_angle" on the vexdash Graph panel and put that number here.
// Or drag ARM_*_DEG on the dashboard Config panel and watch the arm move --
// the values write back live, so you can find them without rebuilding.
//
// WHERE ZERO IS, and why it matters more than it looks.
//
// Every number in this file is in ROTATION-SENSOR degrees: the whole arm
// controller reads arm_rotation (port 21) through arm_get_position_deg() and
// never reads the arm motor's own encoder. (arm.tare_position() in arm_task()
// zeroes the motor encoder purely so the V5 Brain's built-in MOTORS screen shows
// a sensible number; nothing reads it back.)
//
// But arm_task() calls arm_rotation.reset_position() when it starts, so ZERO IS
// WHEREVER THE ARM HAPPENS TO BE SITTING AT PROGRAM START -- it is NOT a fixed
// physical position. The arm has no limit switch to re-zero against (only the
// cascade does, on ADI 'D'), so there is nothing that can correct it later.
// Practical consequence: park the arm on its bottom hard stop BEFORE running the
// program. Start with it half-raised and all four presets below are offset by
// that much for the whole session, which looks exactly like "the presets are
// wrong" or "it isn't reading the sensor" -- it is, from a shifted origin.
// (An earlier version of this comment claimed zero was the hard stop and pointed
// at an ARM_ZERO_ANGLE_DEG that does not exist in this file. It never did.)
// 中文：零點在哪裡，以及為什麼它比看起來重要。
// 這個檔案裡每一個數字都是「rotation 感測器的度數」：整套手臂控制器都是透過
// arm_get_position_deg() 讀 arm_rotation（埠 21），從來不讀手臂馬達自己的編碼器。
// （arm_task() 裡的 arm.tare_position() 只是為了讓 V5 大腦內建的 MOTORS 畫面顯示
// 合理數字，沒有任何程式讀它。）
// 但是 arm_task() 一開始會呼叫 arm_rotation.reset_position()，所以「零」＝程式啟動
// 那一刻手臂剛好停在哪裡，不是一個固定的物理位置。手臂沒有限位開關可以重新歸零
// （只有滑軌有，在 ADI 'D'），所以之後也沒有任何東西能把它修回來。
// 實務上的意思：跑程式之前先把手臂放到最底下的硬止點。要是啟動時手臂是半抬著的，
// 底下四個預設位置整場都會差那麼多——看起來就會像「預設值錯了」或「它根本沒讀感測
// 器」，其實它有讀，只是原點被移走了。
// （這段註解的舊版本說零點是硬止點、還叫人去看一個 ARM_ZERO_ANGLE_DEG——這個常數在
// 這個檔案裡從來不存在。）
float ARM_DOWN_DEG = 0;
float ARM_POS_1_DEG = 283;
float ARM_POS_2_DEG = 160;   // LEFT sequence's final position, after the cascade is back at 0
float ARM_POS_3_DEG = 265;  // LEFT sequence's raised position, before coming back to POS_2

// Soft travel limits in arm degrees. Targets are clamped here so a bad preset
// stalls the motor against nothing instead of slamming the hard stop.
// Set ARM_MAX_DEG to the arm's real measured travel.
float ARM_MIN_DEG = 0;
float ARM_MAX_DEG = 287;

// The rotation sensor must count UP when the motor drives the arm UP. If the
// arm runs away from its target instead of settling on it, flip this first --
// it's the usual cause, not the gains.
bool ARM_ROTATION_REVERSED = false;

// The arm runs its own PID (see PID.h/PID.cpp, same class the drive/turn PID
// uses) against the rotation sensor -- tune these directly.
//
// kP/kD are 3x the old values. Error used to be motor degrees; it's now arm
// degrees, which is ~3x smaller for the same physical error (old
// ARM_GEAR_RATIO), so the gains are scaled up to keep the same volts-per-inch
// of actual arm movement as before. Re-tune from here.
float ARM_KP = 2;
// KI only turns on within ARM_STARTI of target -- lets the I term slowly
// build enough torque to grind through friction/stiction on the last few
// degrees (where KP*error alone is too weak to move the arm), without
// winding up during a large move.
float ARM_KI = 0.2;
float ARM_KD = 0.5;
float ARM_STARTI = 10; // max error (arm degrees) before the I term starts accumulating

// Gravity feedforward, added on top of the PID output (see arm.h for the full
// explanation). 0 keeps the old behaviour exactly: 0 * anything = 0, so the
// output below is byte-for-byte the PID output until someone raises ARM_KG.
//
// How to find it (do this BEFORE touching kP/kI/kD -- feedforward first, gains
// second, because once the feedforward carries the weight the gains only have
// to clean up the leftovers and can stay gentle):
//   1. Set ARM_HORIZONTAL_DEG to the arm_angle reading when the arm is level.
//   2. Set ARM_KP/KI/KD to 0 on the dashboard so only the feedforward is left.
//   3. Move the arm to horizontal, then raise ARM_KG until the arm just stops
//      sagging (and does not creep upward). That number is ARM_KG.
//   4. Put the gains back and re-tune kP -> kD -> kI from gentle values.
// 中文：重力前饋，加在 PID 輸出上面。預設 0＝跟以前完全一樣。調法：先量水平角度、
// 把 PID 三個增益暫時歸零、把手臂擺水平，慢慢加 ARM_KG 到「剛好不掉也不往上爬」，
// 那就是 ARM_KG；之後才回頭調 kP → kD → kI（前饋扛重量，增益就能放溫和）。
float ARM_KG = 0;
float ARM_HORIZONTAL_DEG = 0;

const int ARM_MAX_VOLTAGE = 97; // out of 127, clamps the PID output
const int ARM_DOWN_MAX_VOLTAGE = 57; // out of 127, clamps output while descending so the arm goes down slower

// Max error (arm degrees) to be considered "arrived" -- see arm_settled below.
// The old 20 motor degrees was ~6.7 arm degrees; this is a bit tighter. Loosen
// it if preset sequences start hitting their PRESET_STEP_TIMEOUT_MS.
float ARM_SETTLE_ERROR_DEG = 5;

// True once the arm is within ARM_SETTLE_ERROR_DEG of arm_target. Lets other
// code (e.g. Drive::control_arcade) wait for the arm to actually get there
// before doing something that depends on it, instead of guessing a delay.
bool arm_settled = false;

bool arm_sensor_ok = false;

float tele_arm_angle = 0;
float tele_arm_target = 0;
float tele_arm_error = 0;
float tele_arm_output = 0;
float tele_arm_ff = 0;

// Last known-good angle, so a momentary sensor dropout doesn't read as 0 (which
// would look like a huge error and slam the arm).
static float arm_last_good_deg = 0;

#ifdef PID_TUNE_PROGRAM
// PID-TUNING BUILD ONLY. See arm.h. Everything between these #ifdef guards is
// absent from the competition build, so nothing below changes the arm's
// behaviour there -- there is nothing there to change it.
// 中文：只有調參版才會編到。比賽版完全沒有這幾行，所以不可能影響比賽版的手臂
// 行為——因為比賽版裡根本沒有這段。
static bool arm_hold_active = false;
static float arm_hold_deg = 0;

// --- trapezoidal motion profile (tuning build only) -------------------------
// The arm PID is handed the FINAL angle and answers with whatever voltage the
// error is worth, so every preset move starts with the biggest kick the caps
// allow. A profile changes what the PID is asked for, not how it answers: the
// commanded angle is walked from where the arm is to where it should go along
// an accelerate / cruise / decelerate ramp, and the PID only ever sees a small
// error. Same gains, much gentler motion, and the shape is three numbers you
// can drag on the dashboard instead of a gain compromise.
// 中文：手臂 PID 本來是直接拿到「最終角度」，誤差多大就給多大電壓，所以每次移動
// 一開始都是上限那一記猛的。梯形不是改 PID 怎麼回答，而是改「問它什麼」：把命令
// 角度從現在的位置沿著「加速→等速→減速」慢慢走到目標，PID 從頭到尾只看到很小的
// 誤差。同一組增益、動作卻柔順很多，而且形狀是三個可以在 dashboard 上拉的數字，
// 不是靠犧牲增益去換。
float ARM_PROFILE_VEL_DPS = 120;   // cruise speed 巡航速度 (arm deg / s)
float ARM_PROFILE_ACC_DPS2 = 300;  // ramp up 加速 (arm deg / s^2)
float ARM_PROFILE_DEC_DPS2 = 300;  // ramp down 減速 (arm deg / s^2)

// Where the profile currently says the arm should be -- the number actually fed
// to the PID. Graph it against arm_angle: the gap between them IS the tracking
// error, and it is the one plot that tells you whether a bad move is the
// profile being too aggressive or the gains being too soft.
// 中文：梯形現在要求手臂待的角度，也就是真正餵給 PID 的那個數字。把它跟 arm_angle
// 疊在一起看，兩條線的差距就是追蹤誤差——這張圖是唯一能分辨「梯形開太快」還是
// 「增益太軟」的圖。
float tele_arm_setpoint = 0;

static bool arm_profile_active = false;
static float arm_profile_setpoint = 0;
static float arm_profile_vel = 0;

// If the arm falls this far behind the commanded angle it has hit something (or
// the gains cannot carry it), so the profile stops walking away from it. Not a
// slider: this is a safety stop, not a shape parameter -- a runaway setpoint
// would have the PID pushing against a jam at full voltage.
// 中文：手臂落後命令角度超過這麼多，就代表它卡住了（或增益根本拉不動），這時候
// 梯形就不再往前走。這個不做成滑桿：它是保護，不是形狀參數——命令角度一路跑掉的
// 話，PID 就會用滿電壓去頂一個卡死的機構。
constexpr float ARM_PROFILE_MAX_LAG_DEG = 25;

// Freezing the setpoint stops it running away, but on its own it is only half a
// protection: a frozen profile is still "running", and the PID is still holding
// a 25-degree error against whatever the arm is stuck on. At ARM_KP=2 that is
// ~50/127 of continuous stall current into a green motor -- the PTC trips in
// tens of seconds. So the freeze is explicitly a SHORT diagnostic state:
//   * while frozen, the output is capped well below the normal limit, and
//   * if it does not clear within ARM_PROFILE_FREEZE_ABORT_MS the move is given
//     up on: profile cleared, arm parked exactly where it is (zero error, zero
//     push), and a stall flag raised for the tuning program to report.
// A profile that recovers on its own -- the arm was merely slow and caught up --
// clears the freeze and carries on, untouched.
// 中文：把設定點凍住只擋掉「命令角度愈跑愈遠」，但那只做了一半：凍住的梯形在狀態
// 上還是「進行中」，PID 也還在用 25 度的誤差頂著卡住的機構。ARM_KP=2 的話那是
// 大約 50/127 的堵轉電流一直灌進綠馬達，PTC 幾十秒就跳。所以凍結被明確設計成一個
// 「短暫的診斷狀態」：
//   * 凍結期間輸出額外夾低，
//   * 超過 ARM_PROFILE_FREEZE_ABORT_MS 還沒解除就整個放棄：清掉梯形、把手臂就地
//     停住（誤差 0、不再出力），並升起 stall 旗標讓調參程式回報。
// 自己恢復的情況（只是慢了一點、後來追上了）會解除凍結、照常跑完，完全不受影響。
constexpr int ARM_PROFILE_FREEZE_ABORT_MS = 2500;
constexpr int ARM_PROFILE_FROZEN_MAX_VOLTAGE = 40; // out of 127 中文：127 分之

static bool arm_profile_frozen = false;
static std::uint32_t arm_profile_freeze_start_ms = 0;
static bool arm_profile_stall = false;

// Whether arm_task() is allowed to write to the arm motor at all. The
// feedforward ramp test drives the motor directly with its own voltages, and two
// writers on one motor means neither one is in control. Mirrors exactly what
// cascade_control_set_enabled() already does for the lift.
// 中文：arm_task() 現在准不准寫手臂馬達。前饋斜坡測試要自己直接給電壓，同一顆馬達
// 有兩個人在寫＝兩個人都沒在控制。作法跟滑軌那邊的 cascade_control_set_enabled()
// 完全一樣。
static bool arm_control_enabled_flag = true;
#endif

void arm_set_position(ArmPosition pos){
#ifdef PID_TUNE_PROGRAM
  // A real preset command always wins over a hold.
  arm_hold_active = false;
  // ...and it is the UNPROFILED entry point, so it also cancels any profile in
  // flight. This is what keeps every existing caller (the RIGHT/LEFT preset
  // sequences in drive.cpp, autonomous) behaving exactly as before: they call
  // this function, so they get the straight-to-target PID they always got.
  // 中文：這是「不走梯形」的入口，所以它也會取消還在跑的梯形。既有的呼叫者
  // （drive.cpp 的 RIGHT/LEFT 預設動作、自走）全都是走這個函式，所以它們的行為
  // 跟以前一模一樣：照舊直接給最終目標。
  arm_profile_active = false;
  // Clearing the freeze/stall flags too keeps the 40/127 diagnostic clamp from
  // outliving a cancelled profile. 中文：一併清掉凍結/卡死旗標，避免取消梯形後
  // 40/127 的診斷夾制殘留下來。
  arm_profile_frozen = false;
  arm_profile_stall = false;
#endif
  arm_target = pos;
}

// Rotation::get_position() returns centidegrees, and PROS_ERR when the sensor
// isn't reporting. Updates arm_sensor_ok as a side effect.
float arm_get_position_deg(){
  std::int32_t centideg = arm_rotation.get_position();

  if(centideg == PROS_ERR){
    arm_sensor_ok = false;
    return arm_last_good_deg;
  }

  arm_sensor_ok = true;
  arm_last_good_deg = centideg / 100.0;
  return arm_last_good_deg;
}

#ifdef PID_TUNE_PROGRAM
// PID-TUNING BUILD ONLY. See arm.h.
void arm_hold_here(){
  arm_hold_deg = arm_get_position_deg();
  arm_hold_active = true;
  // Abort means "stop here". A profile left running would keep walking the
  // commanded angle towards the preset the driver just cancelled.
  // 中文：中止＝「就地停住」。梯形沒關掉的話，命令角度還會繼續往剛剛被取消的那個
  // 目標走過去。
  arm_profile_active = false;
  arm_profile_frozen = false;
  // Reading the stall flag is what the tuning program does right before it
  // aborts, and aborting comes through here -- so this is where it is cleared.
  // 中文：調參程式就是「讀到 stall 旗標 → 中止」，而中止一定會走到這裡，所以旗標
  // 在這裡清掉。
  arm_profile_stall = false;
}

// PID-TUNING BUILD ONLY. Same destination as arm_set_position(), but the
// commanded angle gets there along the trapezoid instead of jumping to it.
// 中文：目的地跟 arm_set_position() 一樣，差別只在命令角度是沿著梯形走過去，
// 不是一步跳過去。
void arm_move_profiled(ArmPosition pos){
  arm_hold_active = false;
  // Start from where the arm actually is, at a standstill. Restarting the ramp
  // from zero speed on every new command costs a fraction of a second and makes
  // the profile impossible to get into a bad state by mashing buttons -- there
  // is no leftover velocity from the previous move to carry into this one, in
  // the wrong direction or otherwise.
  // 中文：從手臂「現在的實際位置」、速度 0 開始。每次新命令都重新起步只慢一點點，
  // 但換來的是「怎麼亂按都不會把梯形按進奇怪的狀態」——不會有上一次動作殘留的速度
  // （更不會有方向相反的殘留速度）被帶進這一次。
  arm_profile_setpoint = arm_get_position_deg();
  arm_profile_vel = 0;
  arm_profile_active = true;
  arm_profile_frozen = false;
  arm_profile_stall = false;
  // Deliberately NOT arm_set_position(): that one cancels the profile.
  // 中文：這裡故意不呼叫 arm_set_position()——那支會把梯形取消掉。
  arm_target = pos;
}

bool arm_profile_running(){
  return arm_profile_active;
}

bool arm_profile_stalled(){
  return arm_profile_stall;
}

void arm_control_set_enabled(bool enabled){
  // Handing control back: park the arm on its current angle and drop any profile
  // so it cannot resume a move that was interrupted by the ramp, and so the PID
  // starts from zero error instead of yanking the arm to a stale target.
  // 中文：把控制權交還回來的時候，把手臂停在它現在的角度、順手丟掉梯形，這樣被斜坡
  // 打斷的動作不會自己續跑，PID 也是從「誤差 0」開始，不會把手臂拉去一個過期的目標。
  if(enabled && !arm_control_enabled_flag){
    arm_hold_here();
  }
  arm_control_enabled_flag = enabled;
}

bool arm_control_enabled(){
  return arm_control_enabled_flag;
}

// One 10 ms step of the trapezoid. Returns the angle the PID should be given
// this cycle: the moving setpoint while a profile runs, the plain goal
// otherwise. `goal` is already clamped to the soft travel limits by the caller.
// 中文：梯形的一個 10ms 步進。回傳「這一圈要餵給 PID 的角度」：有梯形在跑就是移動
// 中的設定點，沒有就是原本的目標。傳進來的 goal 已經被呼叫端夾在軟行程內了。
static float arm_profile_step(float goal, float position){
  if(!arm_profile_active){
    // Idle: the setpoint tracks reality, so the next profile starts from the
    // arm's real angle even if it was pushed by hand in the meantime.
    // 中文：沒在跑的時候讓設定點貼著現實走，這樣就算中途有人用手把手臂扳過，下一次
    // 梯形也是從真正的角度起步。
    arm_profile_setpoint = position;
    arm_profile_vel = 0;
    // Report the setpoint that is actually in effect (= the arm's own angle),
    // NOT the goal. Reporting the goal made the line jump to the destination the
    // instant a profile ended, which reads on the graph as an overshoot that
    // never happened. Now the setpoint line simply joins arm_angle when the
    // profile hands over, which is the truth.
    // 中文：回報「現在真正生效的設定點」（＝手臂自己的角度），不是終點。回報終點的話，
    // 梯形一結束這條線就跳到終點，在圖上看起來像一個根本沒發生過的過衝。改成這樣之後，
    // 梯形交棒的瞬間設定點線會直接接上 arm_angle，這才是實情。
    tele_arm_setpoint = arm_profile_setpoint;
    return goal;
  }

  // The arm is not keeping up: freeze the commanded angle here rather than let
  // it run away from a jammed mechanism, and start a clock. See
  // ARM_PROFILE_MAX_LAG_DEG / ARM_PROFILE_FREEZE_ABORT_MS.
  // 中文：手臂跟不上，就把命令角度凍在這裡，不要對著一個卡住的機構愈跑愈遠，同時開始
  // 計時。見 ARM_PROFILE_MAX_LAG_DEG／ARM_PROFILE_FREEZE_ABORT_MS。
  if(fabs(arm_profile_setpoint - position) > ARM_PROFILE_MAX_LAG_DEG){
    if(!arm_profile_frozen){
      arm_profile_frozen = true;
      arm_profile_freeze_start_ms = pros::millis();
    }
    arm_profile_vel = 0;

    // Still stuck after the grace period: give the move up entirely. Parking the
    // arm on its own current angle is what actually takes the load off -- the
    // error goes to zero, so the PID stops pushing, instead of leaning on a jam
    // until the motor's thermal protection trips.
    // 中文：寬限時間過了還是卡著，就整個放棄這次動作。把手臂停在「它自己現在的角度」
    // 才是真的把負載拿掉——誤差歸零、PID 不再出力，而不是一直頂到馬達過熱保護跳掉。
    if(pros::millis() - arm_profile_freeze_start_ms >
       (std::uint32_t)ARM_PROFILE_FREEZE_ABORT_MS){
      arm_profile_active = false;
      arm_profile_frozen = false;
      arm_profile_stall = true;      // the tuning program reports and aborts
      arm_profile_setpoint = position;
      arm_hold_deg = position;       // hold here from the next cycle on
      arm_hold_active = true;
      tele_arm_setpoint = position;
      return position;
    }

    tele_arm_setpoint = arm_profile_setpoint;
    return arm_profile_setpoint;
  }

  // Caught up on its own -- nothing was wrong, the arm was just slow.
  // 中文：自己追上了——沒事，只是手臂慢了一點。
  arm_profile_frozen = false;

  const float dt = 0.01f; // this loop runs every 10 ms 中文：這個迴圈 10ms 一圈
  // A zero or negative number from a slider would divide by zero / stall the
  // ramp, so each one has a floor.
  // 中文：滑桿被拉到 0 或負的話會除以 0、或讓斜坡永遠走不動，所以三個都給下限。
  float vmax = ARM_PROFILE_VEL_DPS > 1 ? ARM_PROFILE_VEL_DPS : 1;
  float acc  = ARM_PROFILE_ACC_DPS2 > 1 ? ARM_PROFILE_ACC_DPS2 : 1;
  float dec  = ARM_PROFILE_DEC_DPS2 > 1 ? ARM_PROFILE_DEC_DPS2 : 1;

  float remaining = goal - arm_profile_setpoint;
  float dist = fabs(remaining);
  float dir = remaining >= 0 ? 1.0f : -1.0f;

  // Braking distance at the current speed. Inside it, slow down; outside it,
  // speed up (capped at the cruise speed) -- that is the whole trapezoid.
  // 中文：以現在的速度要煞停需要多少距離。進到這個距離內就減速，還沒進去就加速
  // （加到巡航速度為止）——梯形就只是這一句話。
  float stop_dist = (arm_profile_vel * arm_profile_vel) / (2 * dec);
  if(dist <= stop_dist) arm_profile_vel -= dec * dt;
  else                  arm_profile_vel += acc * dt;
  arm_profile_vel = clamp(arm_profile_vel, 0.0f, vmax);

  float step = arm_profile_vel * dt;
  if(step >= dist){
    // Close enough to land exactly on the goal this cycle.
    // 中文：這一圈剛好可以落在目標上。
    arm_profile_setpoint = goal;
    arm_profile_vel = 0;
    arm_profile_active = false;
  }
  else{
    arm_profile_setpoint += dir * step;
  }

  tele_arm_setpoint = arm_profile_setpoint;
  return arm_profile_setpoint;
}
#endif

float arm_target_degrees(ArmPosition pos){
  float arm_deg;
  switch(pos){
    case ArmPosition::DOWN:  arm_deg = ARM_DOWN_DEG;  break;
    case ArmPosition::POS_1: arm_deg = ARM_POS_1_DEG; break;
    case ArmPosition::POS_2: arm_deg = ARM_POS_2_DEG; break;
    case ArmPosition::POS_3: arm_deg = ARM_POS_3_DEG; break;
    default:                 arm_deg = ARM_DOWN_DEG;  break;
  }
  return clamp(arm_deg, ARM_MIN_DEG, ARM_MAX_DEG);
}

void arm_task(){
  // Default brake mode is COAST, which would let the arm sag under gravity
  // between PID updates instead of holding position.
  arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  arm_rotation.set_reversed(ARM_ROTATION_REVERSED);
  arm_rotation.set_data_rate(5); // ms, so the 10ms loop always has a fresh sample
  arm_rotation.reset_position(); // wherever the arm physically is at program start becomes 0

  // The motor encoder no longer drives the PID, but the MOTORS tab of the V5
  // dashboard shows it, so keep it zeroed at the same moment.
  arm.tare_position();

  arm_target = ArmPosition::DOWN;

  PID armPID(0, ARM_KP, ARM_KI, ARM_KD, ARM_STARTI);

  while(true){
    // Copy the live gains in every cycle so the dashboard sliders (ARM_KP/KI/KD
    // are registered with watch_config in main.cpp) actually reach the PID.
    // The PID object copies its gains in the constructor, so without this the
    // sliders moved a number nobody read.
    // 中文：每圈把可調增益抄進 PID。PID 是在建構時把 kp/ki/kd 複製走的，不重新抄
    // 的話，dashboard 上拉滑桿只是改到一個沒人看的變數。
    armPID.kp = ARM_KP;
    armPID.ki = ARM_KI;
    armPID.kd = ARM_KD;
    armPID.starti = ARM_STARTI;

#ifdef PID_TUNE_PROGRAM
    // Someone else owns the motor right now (the feedforward ramp test). Do not
    // write to it at all, and keep the PID's memory clear -- an integral built up
    // against a target nobody was driving to would be dumped into the arm the
    // instant control comes back. Telemetry keeps flowing so the ramp's own
    // sampling and the graph still agree on where the arm is.
    // 中文：現在馬達歸別人管（前饋斜坡測試）。這時候一個字都不寫給它，並且把 PID 的
    // 記憶清乾淨——對著一個沒人在開的目標累積的積分，等控制權回來就會整包倒進手臂。
    // 遙測照常更新，這樣斜坡自己的取樣跟圖表對「手臂在哪」的說法才會一致。
    if(!arm_control_enabled_flag){
      armPID.accumulated_error = 0;
      armPID.previous_error = 0;
      tele_arm_angle = arm_get_position_deg();
      tele_arm_output = 0;
      arm_settled = false;
      delay(10);
      continue;
    }
#endif

    // goal = where the arm is being sent. target = what the PID is asked for
    // THIS cycle, which is the same thing unless a profile is walking it there.
    // In the competition build the two are always identical -- arm_profile_step()
    // does not exist there.
    // 中文：goal＝手臂要去的地方；target＝這一圈實際餵給 PID 的角度。沒有梯形在跑
    // 的時候兩者相同。比賽版裡兩者永遠相同——arm_profile_step() 在那一版根本不存在。
    float goal = arm_target_degrees(arm_target);
#ifdef PID_TUNE_PROGRAM
    // PID-TUNING BUILD ONLY: the abort key parks the arm where it is.
    if(arm_hold_active) goal = clamp(arm_hold_deg, ARM_MIN_DEG, ARM_MAX_DEG);
#endif
    float position = arm_get_position_deg();
    float target = goal;
#ifdef PID_TUNE_PROGRAM
    target = arm_profile_step(goal, position);
#endif
    float error = target - position;

    tele_arm_angle = position;
    // The DESTINATION, so the graph line still means what it always meant. The
    // moving profile point is a separate channel (arm_setpoint).
    // 中文：這條線畫的是「終點」，意思跟以前一樣。梯形移動中的那一點是另一條線
    // （arm_setpoint）。
    tele_arm_target = goal;
    tele_arm_error = error;

    // With no trustworthy position there's no safe direction to drive, so stop
    // and let the brake mode hold the arm where it is.
    if(!arm_sensor_ok){
      arm.move(0);
      tele_arm_output = 0;
      tele_arm_ff = 0;
      arm_settled = false;
      delay(10);
      continue;
    }

    // Gravity feedforward, added on top of the PID -- the shared PID class is
    // NOT touched (drive and turn use the same class). Full push at horizontal,
    // none when the arm points straight up or straight down. ARM_KG defaults to
    // 0, so this whole line is a no-op until someone tunes it.
    // 中文：重力前饋加在 PID 外面，共用的 PID class 一個字都沒改（底盤直走/轉彎也在
    // 用它）。水平時給滿、垂直時給 0。ARM_KG 預設 0，沒調之前這行等於不存在。
    float gravity_ff = ARM_KG * cosf(to_rad(position - ARM_HORIZONTAL_DEG));
    tele_arm_ff = gravity_ff;

    float output = armPID.compute(error) + gravity_ff;

    int max_voltage = output < 0 ? ARM_DOWN_MAX_VOLTAGE : ARM_MAX_VOLTAGE;
#ifdef PID_TUNE_PROGRAM
    // The lag guard has frozen the setpoint, which means the arm is being held
    // against something it cannot move. Whatever the error is worth, it is not
    // worth full voltage: cap it low for the couple of seconds before the stall
    // logic gives the move up, so a jam cannot cook the motor in the meantime.
    // 中文：落後保護已經把設定點凍住了，代表手臂正頂著一個它推不動的東西。這時候誤差
    // 再大也不值得給滿電壓：在 stall 判定放棄這次動作之前的這兩三秒把輸出夾低，卡住
    // 就不會順便把馬達煮了。
    if(arm_profile_frozen && max_voltage > ARM_PROFILE_FROZEN_MAX_VOLTAGE){
      max_voltage = ARM_PROFILE_FROZEN_MAX_VOLTAGE;
    }
#endif
    output = clamp(output, (float)-max_voltage, (float)max_voltage);

    arm.move(output);
    tele_arm_output = output;
    // Settled is measured against the DESTINATION, never against the profile's
    // moving setpoint -- the PID tracks that setpoint closely the whole way, so
    // settling on it would report "arrived" a fraction of a second into a move
    // that has barely started. Everything that waits on the arm (the preset
    // sequences in drive.cpp, the tuning program's ARM DONE) depends on this.
    // 中文：到位與否一律拿「終點」來量，不能拿梯形移動中的設定點——PID 全程都緊貼
    // 著那個設定點，拿它來量的話，動作才剛起步就會回報「到了」。所有等手臂的程式
    // （drive.cpp 的預設動作、調參版的 ARM DONE）都靠這一行。
    arm_settled = fabs(goal - position) < ARM_SETTLE_ERROR_DEG;
    delay(10);
  }
}

void start_arm_task(){
  static Task arm_bg_task(arm_task);
}
