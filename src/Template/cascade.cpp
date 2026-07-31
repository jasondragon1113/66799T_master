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

// Gravity feedforward, in TWO numbers plus the band between them, because one
// number is not enough for this mechanism. A cascade lift does not carry a
// constant load: as the stages come out, more of the lift's own mass hangs off
// the last stage and the band tension changes, so the voltage that just holds it
// still near the bottom is not the voltage that holds it near the top. With a
// single kG you end up splitting the difference -- sagging high, creeping low.
//
// CASCADE_KG      = holding command at/below RAMP_START (bottom)
// CASCADE_KG_TOP  = holding command at/above RAMP_END   (top)
// in between, it blends straight-line between the two.
// All default to 0 / a full-travel band, so the sum below is 0 until someone
// tunes it: behaviour is byte-for-byte what it was.
//
// How to find them: park the lift low, raise CASCADE_KG until it stops sagging
// and does not creep up -- that is the bottom number. Do the same near the top
// for CASCADE_KG_TOP. Then set RAMP_START/END to the two heights you measured at.
// 中文：重力前饋改成「兩個數字＋中間的過渡帶」，因為這支機構用一個數字不夠。串接式
// 升降扛的重量不是固定的：節數伸出去之後，越多自身重量掛在最後一節上、皮帶張力也變，
// 所以「在低點剛好撐住」的電壓不等於「在高點剛好撐住」的電壓。只給一個 kG 的結果就是
// 兩邊各妥協一半——高的地方往下沉、低的地方自己往上爬。
// CASCADE_KG＝在 RAMP_START（低點）以下的撐住電壓；CASCADE_KG_TOP＝在 RAMP_END
// （高點）以上的撐住電壓；中間直線內插。
// 預設全 0、過渡帶涵蓋整個行程，所以沒調之前這一項就是 0，行為跟以前一模一樣。
// 量法：把升降停在低處，把 CASCADE_KG 往上加到「不往下沉、也不會自己往上爬」，那就是
// 低點的數字；在高處對 CASCADE_KG_TOP 做一樣的事。最後把 RAMP_START／END 填成你量測
// 時的那兩個高度。
float CASCADE_KG = 0;
float CASCADE_KG_TOP = 0;
float CASCADE_KG_RAMP_START_DEG = 0;
float CASCADE_KG_RAMP_END_DEG = 3900;

const int CASCADE_MAX_VOLTAGE = 100;       // same as the old L1 jog voltage
const int CASCADE_DOWN_MAX_VOLTAGE = 97;   // same as the old L2 jog voltage

float CASCADE_SETTLE_ERROR_DEG = 20;
bool cascade_settled = false;

float tele_cascade_pos = 0;
float tele_cascade_target = 0;
float tele_cascade_error = 0;
float tele_cascade_output = 0;
float tele_cascade_ff = 0;

// Per-motor telemetry -- see cascade.h for why both motors are reported
// separately. Sampled by cascade_task() below.
// 中文：兩顆馬達分開上報的遙測，理由見 cascade.h。由下面的 cascade_task() 取樣。
float tele_cascade1_pos = 0;
float tele_cascade2_pos = 0;
float tele_cascade1_temp = 0;
float tele_cascade2_temp = 0;

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
  // Anyone commanding a position is taking the lift back from the driver, so the
  // jog override ends here as well. Without this a jog that was left switched on
  // (the old button chain could do exactly that) would swallow every preset:
  // the task's jog branch runs first and never looks at the target.
  // 中文：只要有人下「去這個位置」的命令，就等於把升降從駕駛手上收回來，所以順手把
  // 點動狀態關掉。不關的話，一個沒被關掉的點動（舊的按鍵鏈就會留下這種狀態）會把之後
  // 每一個預設動作都吃掉——task 裡點動那一支排在前面，根本不會去看目標值。
  cascade_jog_stop();
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
  // Parking the controller (disabled(), autonomous()) also cancels any jog left
  // switched on, so a driver who was holding L1 when the field cut the robot off
  // does not have that voltage waiting to resume the moment control comes back.
  // 中文：把控制器停用（disabled()、autonomous()）的時候，順手把還開著的點動取消。
  // 不然「場地斷電那一刻剛好壓著 L1」的電壓會一直留著，等控制恢復就直接續開。
  if(!enabled){
    cascade_jog_stop();
  }
  cascade_enabled = enabled;
}

bool cascade_control_enabled(){
  return cascade_enabled;
}

void cascade_notify_tare(){
  cascade_pid.accumulated_error = 0;
  cascade_pid.previous_error = 0;
  // Same reason the D term is seeded on the first tick of a move (PID.h): this
  // controller is a long-lived singleton, so "forget everything" has to include
  // "and do not treat the next error as a step change from zero", or every tare
  // and every hand-back would produce one saturated derivative spike.
  // 中文：跟每個動作第一圈要種 previous_error 是同一個理由（見 PID.h）：這顆控制器是
  // 長生命週期的單例，「把記憶清掉」就必須連「下一圈不要把誤差當成從 0 跳上來的階躍」
  // 一起清，否則每次歸零、每次交還控制權都會生出一記飽和的微分尖峰。
  cascade_pid.first_update = true;
}

// Sample both motors for the dashboard. Positions every cycle (they are cheap
// and a divergence between them is the whole point); temperature only a few
// times a second, because it is a slow-moving number and every read is another
// smart-port round trip.
// 中文：把兩顆馬達的狀態取樣給 dashboard。位置每圈都讀（很便宜，而且「兩顆差多少」
// 正是重點）；溫度一秒讀幾次就好——它本來就變得慢，而每讀一次就是一趟智慧埠來回。
static void cascade_sample_motors(){
  static int temp_divider = 0;
  double p1 = cascade1.get_position();
  double p2 = cascade2.get_position();
  if(std::isfinite(p1)) tele_cascade1_pos = (float)p1;
  if(std::isfinite(p2)) tele_cascade2_pos = (float)p2;

  if(++temp_divider >= 25){ // 25 * 10ms = every 250 ms 中文：250ms 一次
    temp_divider = 0;
    double t1 = cascade1.get_temperature();
    double t2 = cascade2.get_temperature();
    if(std::isfinite(t1)) tele_cascade1_temp = (float)t1;
    if(std::isfinite(t2)) tele_cascade2_temp = (float)t2;
  }
}

void cascade_task(){
  while(true){
    // Runs before the enabled check below, so the two motors are still being
    // watched during autonomous and while the controller is parked -- exactly
    // when a motor that has dropped out is hardest to notice by feel.
    // 中文：放在下面「有沒有啟用」的判斷之前，所以自走期間、控制器被停用期間也照樣
    // 在盯這兩顆馬達——那正是「哪一顆掉了」最難用手感察覺的時候。
    cascade_sample_motors();

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
      cascade_target_deg = clamp(position, 0.0f, CASCADE_EXTEND_LIMIT_DEG);
      cascade_notify_tare();  // no windup while the driver is in charge 中文：駕駛開的時候不要累積積分

      // Travel limits apply to the JOG path too. They used to be checked only by
      // the caller (control_arcade) and only by the PID branch below, so any loop
      // that skipped the L1/L2 branch left a stale jog voltage running with
      // nothing stopping it at the ends of travel. Enforcing it here means the
      // limit holds no matter who called cascade_jog() or how they got distracted.
      // 中文：行程上下限對「點動」這條路徑也要生效。以前只有呼叫端（control_arcade）
      // 和下面 PID 那條在夾，所以只要有哪一圈沒走到 L1/L2 那支，殘留的點動電壓就會
      // 一路開下去、到底了也沒人喊停。改成在這裡強制夾，不管是誰呼叫 cascade_jog()、
      // 中途被什麼打斷，上下限都一定成立。
      float jog_voltage = cascade_jog_voltage;
      if(jog_voltage > 0 && position >= CASCADE_EXTEND_LIMIT_DEG) jog_voltage = 0;
      if(jog_voltage < 0 && position <= 0) jog_voltage = 0;

      cascade1.move(jog_voltage);
      cascade2.move(jog_voltage);

      tele_cascade_target = cascade_target_deg;
      tele_cascade_error = 0;
      tele_cascade_ff = 0;
      tele_cascade_output = jog_voltage;
      cascade_settled = false;
      delay(10);
      continue;
    }

    float target = clamp(cascade_target_deg, 0.0f, CASCADE_EXTEND_LIMIT_DEG);
    float error = target - position;

    // Height-dependent gravity term: CASCADE_KG at the bottom of the band,
    // CASCADE_KG_TOP at the top, straight-line blend in between (see the note
    // where these are declared for why one number is not enough here). Both
    // default to 0, so this is 0 until someone tunes it -- exactly the old
    // behaviour, which was a single constant that also defaulted to 0.
    // 中文：跟高度有關的重力補償：過渡帶底部用 CASCADE_KG、頂部用 CASCADE_KG_TOP、
    // 中間直線內插（為什麼一個數字不夠，見宣告處）。兩個都預設 0，所以沒調之前這裡
    // 就是 0——跟原本那個同樣預設 0 的單一常數完全一樣。
    float ramp_span = CASCADE_KG_RAMP_END_DEG - CASCADE_KG_RAMP_START_DEG;
    float ramp_t = ramp_span > 1
                     ? clamp((position - CASCADE_KG_RAMP_START_DEG) / ramp_span, 0.0f, 1.0f)
                     : 0.0f;
    float gravity_ff = CASCADE_KG + (CASCADE_KG_TOP - CASCADE_KG) * ramp_t;

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
