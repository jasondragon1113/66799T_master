#include "main.h"

// Cascade position controller -- see include/Template/cascade.h for what this
// is and how the pieces fit together.
// 中文：cascade 位置控制器。這支機構在做什麼、各部分怎麼配合，看 cascade.h。

float CASCADE_EXTEND_LIMIT_DEG = 3900;

// Conservative starting gains. kP is small on purpose, kI/kD/kG are off.
// Tuning order on the real robot: kG first (find the voltage that makes the
// lift hover), then kP (until it reaches the presets without slamming), then
// kD if it overshoots, kI last and only if it stops just short every time.
// 中文：保守的起步增益。kP 故意調小，kI/kD/kG 先關掉。上車調參順序：先 kG（找到
// 讓升降停在半空中不動的電壓），再 kP（調到能到位又不會撞），會過頭才加 kD，最後
// 才考慮 kI（每次都差一點點才用）。
float CASCADE_KP = 0.1;
float CASCADE_KI = 0;
float CASCADE_KD = 0;
float CASCADE_STARTI = 50;
float CASCADE_KG = 0;

const int CASCADE_MAX_VOLTAGE = 100;       // same as the old L1 jog voltage
const int CASCADE_DOWN_MAX_VOLTAGE = 97;   // same as the old L2 jog voltage

float CASCADE_SETTLE_ERROR_DEG = 20;
bool cascade_settled = false;

float tele_cascade_pos = 0;
float tele_cascade_target = 0;
float tele_cascade_error = 0;
float tele_cascade_output = 0;
float tele_cascade_ff = 0;

// Where the controller is trying to hold the cascade, in motor degrees.
// 中文：控制器現在想把 cascade 停在哪裡（馬達度數）。
static float cascade_target_deg = 0;

// Driver override state: while cascade_jog_on is true the PID output is not
// used at all and cascade_jog_voltage goes straight to the motors.
// 中文：駕駛點動狀態。點動中就完全不用 PID 的輸出，直接把電壓餵給馬達。
static bool cascade_jog_on = false;
static float cascade_jog_voltage = 0;

// The controller starts asleep so autonomous keeps its move_absolute() moves.
// 中文：控制器預設睡著，自走才能繼續用它原本的 move_absolute()。
static bool cascade_enabled = false;

// Last position both motors agreed on, used when a motor stops reporting.
// 中文：上一個讀得到的位置，馬達突然讀不到時拿來頂著用。
static float cascade_last_good_deg = 0;

// The same PID class the drive, turn and arm loops use -- a separate INSTANCE,
// the class itself is not modified.
// 中文：跟底盤直走／轉彎／手臂同一顆 PID class，只是另外開一個實例；class 本身一
// 個字都沒改。
static PID cascade_pid(0, CASCADE_KP, CASCADE_KI, CASCADE_KD, CASCADE_STARTI);

void cascade_set_target(float deg){
  cascade_target_deg = clamp(deg, 0.0f, CASCADE_EXTEND_LIMIT_DEG);
}

float cascade_get_target(){
  return cascade_target_deg;
}

// Motor::get_position() hands back PROS_ERR_F (an infinity) when the motor is
// not reporting, which would look like an enormous error and slam the lift.
// 中文：馬達讀不到時 get_position() 會回一個「無限大」，直接拿來算誤差會炸，所以
// 這裡先擋掉。
float cascade_get_position_deg(){
  double p = cascade1.get_position();
  if(!std::isfinite(p)){
    p = cascade2.get_position();
  }
  if(!std::isfinite(p)){
    return cascade_last_good_deg;
  }
  cascade_last_good_deg = (float)p;
  return cascade_last_good_deg;
}

void cascade_jog(float voltage){
  cascade_jog_voltage = voltage;
  cascade_jog_on = true;
}

void cascade_jog_stop(){
  cascade_jog_on = false;
  cascade_jog_voltage = 0;
}

bool cascade_jog_active(){
  return cascade_jog_on;
}

void cascade_control_set_enabled(bool enabled){
  // Waking up: throw away everything the PID remembers. Whatever happened while
  // the controller was parked (a match pause, an autonomous run that moved the
  // lift somewhere else) is not error this loop should answer for -- and a
  // non-zero kI plus a few seconds of stale accumulated error is exactly how a
  // lift jumps the moment control comes back.
  // 中文：要重新啟用的時候，把 PID 記得的東西全部丟掉。它被停用期間發生的事（比賽
  // 暫停、自走把升降開去別的地方）不該算在這一圈頭上；kI 只要不是 0，累了幾秒的
  // 積分一恢復控制就是一記暴衝。
  if(enabled && !cascade_enabled){
    cascade_notify_tare();
  }
  cascade_enabled = enabled;
}

bool cascade_control_enabled(){
  return cascade_enabled;
}

void cascade_notify_tare(){
  cascade_pid.accumulated_error = 0;
  cascade_pid.previous_error = 0;
}

void cascade_task(){
  while(true){
    // Asleep: do not write to the motors at all. Whoever else is driving them
    // (an auton move_absolute(), say) is left completely alone.
    // 中文：睡著的時候一個字都不寫給馬達，讓別人（例如自走的 move_absolute()）自
    // 己開，完全不干擾。
    if(!cascade_enabled){
      cascade_notify_tare();  // nothing accumulated while asleep 中文：睡著期間不累積積分
      cascade_settled = false;
      delay(10);
      continue;
    }

    // Copy the live gains in every cycle so the dashboard sliders actually
    // reach the PID -- it copies its gains in the constructor, so a slider that
    // only changed CASCADE_KP would otherwise change nothing.
    // 中文：每圈把可調增益抄進 PID。PID 是建構時複製增益的，不抄的話 dashboard 上
    // 拉滑桿等於白拉。
    cascade_pid.kp = CASCADE_KP;
    cascade_pid.ki = CASCADE_KI;
    cascade_pid.kd = CASCADE_KD;
    cascade_pid.starti = CASCADE_STARTI;

    float position = cascade_get_position_deg();
    tele_cascade_pos = position;

    if(cascade_jog_on){
      // Driver has the button down. The PID steps aside and the target follows
      // the lift, so letting go holds it exactly where it stopped.
      // 中文：駕駛正壓著按鍵。PID 讓位，目標跟著升降跑，放開就停在那一格。
      cascade_target_deg = position;
      cascade_notify_tare();  // no windup while the driver is in charge 中文：駕駛開的時候不要累積積分

      cascade1.move(cascade_jog_voltage);
      cascade2.move(cascade_jog_voltage);

      tele_cascade_target = position;
      tele_cascade_error = 0;
      tele_cascade_ff = 0;
      tele_cascade_output = cascade_jog_voltage;
      cascade_settled = false;
      delay(10);
      continue;
    }

    float target = clamp(cascade_target_deg, 0.0f, CASCADE_EXTEND_LIMIT_DEG);
    float error = target - position;

    // Constant gravity term -- the cascade carries the same load at every
    // height, so there is no angle to scale it by. 0 until it is tuned.
    // 中文：固定的重力補償。升降不管在哪一格扛的重量都一樣，沒有角度要乘。沒調之
    // 前是 0。
    float gravity_ff = CASCADE_KG;

    float output = cascade_pid.compute(error) + gravity_ff;

    int max_voltage = output < 0 ? CASCADE_DOWN_MAX_VOLTAGE : CASCADE_MAX_VOLTAGE;
    output = clamp(output, (float)-max_voltage, (float)max_voltage);

    cascade1.move(output);
    cascade2.move(output);

    tele_cascade_target = target;
    tele_cascade_error = error;
    tele_cascade_ff = gravity_ff;
    tele_cascade_output = output;
    cascade_settled = fabs(error) < CASCADE_SETTLE_ERROR_DEG;
    delay(10);
  }
}

void start_cascade_task(){
  static Task cascade_bg_task(cascade_task);
}
