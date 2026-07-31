#pragma once
#include "main.h"
using namespace pros;

// Cascade (the two-stage lift on ports 7 and -2) position control.
//
// Same shape as the arm: a background task runs a PID against a target you set
// from anywhere, plus a gravity feedforward so the lift does not have to sag
// before the controller pushes back. Position comes from the motors' own
// built-in encoders -- there is no external encoder on this mechanism -- and
// the limit switch on ADI port 'D' re-zeroes them at the physical bottom.
//
// 中文：cascade（7 與 -2 兩顆馬達的伸縮升降）位置控制。做法跟 arm 一樣：一個背景
// task 跑 PID 追你設的目標，再加一份重力前饋，讓它不用先掉下去才有力氣撐住。位置
// 讀馬達內建編碼器（這支機構沒有外接編碼器），底部的限位開關會幫編碼器歸零。
//
// Two motors, one output: cascade1 and cascade2 always get the SAME number.
// cascade2 is configured on port -2 (reversed) in robot-config.cpp, so the same
// number turns them the right way round physically.
// 中文：兩顆馬達永遠餵同一個數字；cascade2 在 robot-config.cpp 是負埠號（反轉），
// 所以同一個數字物理上就是對的方向。

// Highest position the cascade is allowed to reach, in cascade motor degrees
// counted from the last tare_position(). Lives here (not in drive.cpp) so the
// teleop buttons and the controller share one number.
// 中文：cascade 最高只能到這裡（馬達度數，從上次歸零算起）。放這裡是為了讓遙控
// 按鍵跟控制器共用同一個數字，不會兩邊各寫一份。
extern float CASCADE_EXTEND_LIMIT_DEG;

// --- PID gains ------------------------------------------------------------
// Deliberately gentle out of the box: small kP, no kI, no kD, no feedforward.
// The lift will feel weak until these are tuned on the real robot -- that is
// on purpose, a lift that is too eager is how gearboxes get broken.
// 中文：出廠值刻意調得很保守（kP 小、kI/kD/kG 都 0）。沒上車調之前它會軟軟的，
// 這是故意的——升降太衝是打齒輪最快的方法。
extern float CASCADE_KP;
extern float CASCADE_KI;
extern float CASCADE_KD;
extern float CASCADE_STARTI;  // max error (motor degrees) before the I term starts accumulating

// Gravity feedforward: the voltage (same -127..127 units as motor.move()) that
// just holds the cascade still in mid-air. A cascade lift carries the same
// weight at every height, so unlike the arm this is a CONSTANT -- no cos().
// 0 = feedforward off.
// 中文：重力前饋＝把升降停在半空中「剛好不掉」的電壓（跟 move() 同樣的 -127~127
// 單位）。升降不管停在哪一格，扛的重量都一樣，所以這是個常數，不像手臂要乘
// cos(角度)。設 0＝不啟用。
extern float CASCADE_KG;

// Voltage caps, out of 127. The up cap matches the old L1 jog voltage and the
// down cap matches the old L2 jog voltage, so the controller can never drive
// the cascade harder than the driver already could by hand.
// 中文：輸出上限（滿格 127）。往上的上限＝以前 L1 點動用的電壓，往下＝以前 L2 用
// 的電壓，所以控制器最凶也不會比駕駛手動點動更凶。
extern const int CASCADE_MAX_VOLTAGE;
extern const int CASCADE_DOWN_MAX_VOLTAGE;

// Max error (cascade motor degrees) to count as "arrived".
// 中文：誤差小於這個值就算「到位」。
extern float CASCADE_SETTLE_ERROR_DEG;

// True once the cascade is within CASCADE_SETTLE_ERROR_DEG of its target.
// 中文：到位了就是 true。
extern bool cascade_settled;

// --- vexdash live telemetry (streamed to the web dashboard) ---
extern float tele_cascade_pos;     // current position (motor deg)
extern float tele_cascade_target;  // target position (motor deg)
extern float tele_cascade_error;   // target - current (motor deg)
extern float tele_cascade_output;  // what is being sent to both motors
extern float tele_cascade_ff;      // the gravity feedforward part of that output

// Per-motor telemetry. Everything above reports the lift as ONE thing (position
// comes from cascade_get_position_deg(), which reads cascade1 and only falls
// back to cascade2 if cascade1 says nothing), so a cascade2 that has stopped
// pulling is invisible: the lift just gets weak and slow -- the "it only moves
// one motor" symptom -- with nothing on the dashboard saying why. These four
// channels make it obvious: the two positions drift apart, or one temperature
// climbs on its own.
// 中文：上面那些是把升降當成「一個東西」報的（位置來自 cascade_get_position_deg()，
// 它讀 cascade1、只有讀不到才退去讀 cascade2），所以 cascade2 不出力的時候完全看不
// 出來——升降只是變弱變慢，就是「只動一顆」那個症狀，dashboard 上卻沒有線索。
// 這四條線就直接看得出來：兩顆位置拉開，或某一顆溫度自己往上爬。
extern float tele_cascade1_pos;    // cascade1 position (motor deg)
extern float tele_cascade2_pos;    // cascade2 position (motor deg)
extern float tele_cascade1_temp;   // cascade1 temperature (C)
extern float tele_cascade2_temp;   // cascade2 temperature (C)

// --- API -------------------------------------------------------------------

// Where the controller should hold the cascade, in motor degrees. Clamped into
// [0, CASCADE_EXTEND_LIMIT_DEG] before use, so a bad number cannot drive the
// lift into its hard stop. 中文：叫 cascade 去哪一格（馬達度數），會先夾在合法範圍
// 內，填錯數字不會把機構撞到底。
void cascade_set_target(float deg);
float cascade_get_target();

// Current cascade position in motor degrees. Falls back to the other motor and
// then to the last good reading if a motor stops reporting, so one unplugged
// cable cannot make the controller think the lift teleported to 0.
// 中文：目前位置。某顆馬達讀不到就換讀另一顆，兩顆都讀不到就沿用上一個好讀數——
// 不會因為一條線鬆掉就以為升降瞬間跳到 0。
float cascade_get_position_deg();

// Driver override. While a jog voltage is set the PID steps aside completely
// and `voltage` goes straight to both motors, exactly like the old L1/L2 code.
// The target quietly follows the current position while jogging, so
// cascade_jog_stop() holds the lift right where the driver let go.
// 中文：駕駛點動。點動期間 PID 完全讓位，電壓直接餵給兩顆馬達（跟以前 L1/L2 一模
// 一樣）。點動時目標會偷偷跟著目前位置跑，所以放開按鍵就停在那裡不掉下去。
void cascade_jog(float voltage);
void cascade_jog_stop();
bool cascade_jog_active();

// The controller only touches the motors while it is enabled. It starts
// DISABLED, and autonomous keeps it that way, so the auton routines can keep
// driving the cascade with move_absolute() exactly as they always have.
// control_arcade() switches it on for driver control.
// 中文：控制器只有「啟用」時才會碰馬達。預設關著，自走也維持關著，所以自走程式照
// 樣用 move_absolute()、行為完全沒變；遙控 control_arcade() 才把它打開。
void cascade_control_set_enabled(bool enabled);
bool cascade_control_enabled();

// Call right after cascade1/cascade2.tare_position(). Taring moves the zero
// under the controller's feet; this clears the accumulated and previous error
// so the jump does not show up as a fake spike in the I and D terms.
// 中文：編碼器歸零之後要呼叫這個。歸零等於把原點抽掉，先把積分項跟上一次誤差清
// 乾淨，位置的突跳才不會被 I/D 當成真的誤差爆一下。
void cascade_notify_tare();

void cascade_task();
void start_cascade_task();
