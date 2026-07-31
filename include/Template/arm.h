#pragma once
#include "main.h"
using namespace pros;

// Arm preset positions, driven by a background task via its own PID.
// B -> DOWN, A -> POS_1, X -> POS_2 (see Drive::control_arcade in drive.cpp).
enum class ArmPosition {
    DOWN = 0,
    POS_1 = 1,
    POS_2 = 2,
    POS_3 = 3
};

extern ArmPosition arm_target;

// Target angles in ARM degrees, read straight off arm_rotation (the Rotation
// sensor on port 21, see robot-config.cpp). No gear-ratio conversion is
// involved anymore -- the sensor sits on the arm shaft, so what it reads IS
// the arm angle.
//
// ARM_DOWN_DEG is 0: arm_task() zeroes the sensor at program start, so 0 is
// wherever the arm is resting when the program boots.
extern float ARM_DOWN_DEG;
extern float ARM_POS_1_DEG;
extern float ARM_POS_2_DEG;
extern float ARM_POS_3_DEG;

// Soft travel limits in arm degrees. Every target is clamped into this range,
// so a bad preset can't drive the arm into its hard stop at full voltage.
// Set these to the arm's real measured travel.
extern float ARM_MIN_DEG;
extern float ARM_MAX_DEG;

// Set true if the sensor counts DOWN while the motor drives the arm UP. With
// this wrong the PID runs away from the target -- verify before tuning gains.
extern bool ARM_ROTATION_REVERSED;

// Arm position PID gains -- tune these to adjust how the arm reaches and
// holds its target angle. Error is in arm degrees. See arm_task() in arm.cpp.
extern float ARM_KP;
extern float ARM_KI;
extern float ARM_KD;
extern float ARM_STARTI;

// Gravity feedforward: the extra push added on top of the PID output so the
// PID does not have to "earn" the holding voltage with error. ARM_KG is the
// voltage (same -127..127 units as arm.move()) that just barely holds the arm
// still WHEN IT IS HORIZONTAL; away from horizontal it is scaled by
// cos(angle), because a horizontal arm fights all of gravity and a vertical
// arm fights none. 0 = feedforward off, i.e. exactly the old behaviour.
// 中文：重力前饋。ARM_KG＝「手臂放到水平時，剛好撐住不掉下來」的電壓（跟
// arm.move() 同樣的 -127~127 單位）。手臂離開水平後會自動乘 cos(角度)：水平最吃
// 力、垂直不吃力。設 0＝不用前饋，跟以前一模一樣。
extern float ARM_KG;

// The arm angle (same degrees as the ARM_*_DEG presets) at which the arm is
// physically HORIZONTAL. Only used to aim the cos() above. Measure it once:
// move the arm level, read "arm_angle" on the dashboard, put that number here.
// 中文：手臂「水平」時的角度讀數（跟預設位置同一套度數），只給上面的 cos() 用。
// 量法：把手臂擺平，看 dashboard 的 arm_angle 是多少，填進來。
extern float ARM_HORIZONTAL_DEG;

// Separate, lower voltage cap applied whenever the PID output is driving the
// arm downward (e.g. heading to DOWN), so it descends gently instead of
// dropping at full speed. See arm_task() in arm.cpp.
extern const int ARM_DOWN_MAX_VOLTAGE;

// Max error (arm degrees) to be considered "arrived".
extern float ARM_SETTLE_ERROR_DEG;

// True once the arm is within ARM_SETTLE_ERROR_DEG of arm_target -- poll
// this to wait for the arm to actually reach its target. Forced false while
// the rotation sensor is unplugged, since position is unknown then.
extern bool arm_settled;

// True while arm_rotation is not reporting a valid position (unplugged or
// failed). The arm is held at 0 V for as long as this is set.
extern bool arm_sensor_ok;

// --- vexdash live telemetry (streamed to the web dashboard) ---
extern float tele_arm_angle;   // current arm angle (deg)
extern float tele_arm_target;  // target arm angle (deg)
extern float tele_arm_error;   // target - current (deg)
extern float tele_arm_output;  // arm PID output (volts)
extern float tele_arm_ff;      // gravity feedforward part of the output (volts)
                               // 中文：輸出裡屬於重力前饋的那一份

// Current arm angle in degrees from the rotation sensor. Returns the last
// valid reading if the sensor is unplugged.
float arm_get_position_deg();

// Target angle in arm degrees for a preset, already clamped to the soft limits.
float arm_target_degrees(ArmPosition pos);

void arm_set_position(ArmPosition pos);
void arm_task();
void start_arm_task();

#ifdef PID_TUNE_PROGRAM
// PID-TUNING BUILD ONLY (-DPID_TUNE_PROGRAM). Not compiled into the competition
// build at all, so the arm controller the team already signed off on is
// byte-for-byte unchanged there.
//
// Freezes the arm at whatever angle it is at right now, so the tuning program's
// abort key can stop an arm test in place instead of letting it carry on to a
// preset. The next arm_set_position() call cancels it.
// 中文：只有調參版才編得到這一段，比賽版完全沒有這段程式碼，所以已經驗收過的
// 手臂控制器在比賽版是一個位元組都沒變。
// 功能：把手臂鎖在「現在這個角度」，讓調參程式的中止鍵能把手臂停在原地，而不是
// 讓它繼續跑到 preset。下一次呼叫 arm_set_position() 就自動解除。
void arm_hold_here();

// --- trapezoidal motion profile (tuning build only) -------------------------
//
// arm_move_profiled() sends the arm to the same presets arm_set_position()
// does, but the angle handed to the PID is walked there along an
// accelerate / cruise / decelerate ramp instead of jumping straight to the end.
// Only the tuning program's four D-pad position keys use it; every existing
// caller still goes through arm_set_position() and is unaffected -- and
// arm_set_position() cancels a profile in flight, so the two can never fight.
// 中文：arm_move_profiled() 送手臂去的位置跟 arm_set_position() 完全一樣，差別在
// 「餵給 PID 的角度」是沿著加速→等速→減速的斜坡走過去，不是一步跳到終點。只有調參
// 版的四顆方向鍵會用它；其他既有呼叫者一律還是走 arm_set_position()，行為不變——而且
// arm_set_position() 會取消還在跑的梯形，兩邊不可能打架。
//
// The shape is three dashboard sliders ("arm/profile" group). Tune them AFTER
// the gains are sane: the profile decides how fast the arm is ASKED to move,
// the gains decide how well it obeys.
// 中文：形狀是 dashboard 上三顆滑桿（arm/profile 群組）。等增益調好再調它們：梯形
// 決定「叫手臂跑多快」，增益決定「它跟得多好」。
extern float ARM_PROFILE_VEL_DPS;   // cruise speed 巡航速度 (arm deg/s)
extern float ARM_PROFILE_ACC_DPS2;  // acceleration 加速度 (arm deg/s^2)
extern float ARM_PROFILE_DEC_DPS2;  // deceleration 減速度 (arm deg/s^2)

// The angle the profile is commanding right now -- graph it against arm_angle,
// the gap between the two lines is the tracking error.
// 中文：梯形當下要求的角度。跟 arm_angle 疊起來看，兩條線的差距就是追蹤誤差。
extern float tele_arm_setpoint;

void arm_move_profiled(ArmPosition pos);

// True while the trapezoid is still walking the commanded angle to the goal.
// 中文：梯形還在把命令角度往終點走的期間為 true。
bool arm_profile_running();

// True once a profiled move has been GIVEN UP ON because the arm could not keep
// up (jammed, or the gains cannot carry it). By then the arm has already been
// parked on its own current angle, so nothing is being pushed -- the flag exists
// so the tuning program can say so on the controller instead of leaving a move
// silently unfinished. Cleared by arm_hold_here() (which the abort path calls)
// and by the next arm_move_profiled().
// 中文：某次梯形動作因為手臂跟不上（卡住，或增益根本拉不動）而被「放棄」之後為 true。
// 那個時候手臂已經被停在自己現在的角度了，沒有在頂任何東西——這個旗標只是要讓調參程式
// 能在遙控器上講一聲，而不是讓一次動作無聲無息地沒下文。arm_hold_here()（中止流程會
// 呼叫它）與下一次 arm_move_profiled() 都會把它清掉。
bool arm_profile_stalled();

// Hand the arm motor over to something else (the feedforward ramp test) and take
// it back. While disabled, arm_task() does not write to the motor at all --
// exactly like cascade_control_set_enabled() for the lift -- so the two can never
// fight over one motor. Re-enabling parks the arm where it is (see arm.cpp), so
// control comes back with zero error instead of a jump to a stale target.
// 中文：把手臂馬達交給別人（前饋斜坡測試），以及收回來。停用期間 arm_task() 完全不寫
// 那顆馬達——跟滑軌的 cascade_control_set_enabled() 一樣——所以兩邊不會搶同一顆馬達。
// 重新啟用時會把手臂就地停住（見 arm.cpp），控制權回來的瞬間誤差是 0，不會往一個過期
// 的目標跳過去。
void arm_control_set_enabled(bool enabled);
bool arm_control_enabled();
#endif
