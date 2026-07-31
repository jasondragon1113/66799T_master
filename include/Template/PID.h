#pragma once

/**
 * General-use PID class for drivetrains. It includes both
 * control calculation and settling calculation. The default
 * update period is 10ms or 100Hz.
 */

class PID
{
public:
  float error = 0;
  float kp = 0;
  float ki = 0;
  float kd = 0;
  float starti = 0;
  float settle_error = 0;
  float settle_time = 0;
  float timeout = 0;
  float accumulated_error = 0;
  float previous_error = 0;

  // First compute() of this controller's life. The D term is
  // kd*(error - previous_error) and previous_error starts at 0, so on the very
  // first tick of every movement the "change in error" is the WHOLE error and D
  // reports a step that never happened. On a 50 cm drive that is kd*19.7 = 246
  // units of derivative on tick one, which pins the output at its voltage clamp
  // before the robot has moved at all -- and because it saturates for ANY sane
  // kd, changing kd changes nothing you can see. That is exactly what made kD
  // look "dead" on the dashboard.
  // Seeding previous_error from the first real error makes the first derivative
  // exactly 0, which is the truth: nothing has changed yet because nothing has
  // happened yet.
  // (This class's zero-crossing reset tests previous_error < 0 / > 0, and 0 is
  // neither, so unlike some ports it never wiped the integral on tick one --
  // that half of the bug does not exist here.)
  // 中文：D 項是 kd×（本次誤差 − 上次誤差），而「上次誤差」初值是 0——所以每個動作的
  // 第一圈，D 會把「整個誤差」當成一瞬間的變化量。50 公分的直走就是 kd×19.7≈246，車
  // 都還沒動輸出就先被頂到電壓上限；而且不管 kd 填多少都一樣飽和，所以「調 kD 沒感覺」
  // 正是這樣來的。改成第一圈把 previous_error 種成當下的誤差，第一圈的微分就是 0——
  // 這才是事實：什麼都還沒發生。
  // （這個類別的過零重置判斷的是 previous_error < 0／> 0，而 0 兩者皆非，所以它不像
  // 某些移植版會在第一圈把積分清掉——那半個 bug 在這裡不存在。）
  bool first_update = true;
  float output = 0;
  float time_spent_settled = 0;
  float time_spent_running = 0;
  float update_period = 10;

  PID(float error, float kp, float ki, float kd, float starti);

  PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout);

  PID(float error, float kp, float ki, float kd, float starti, float settle_error, float settle_time, float timeout, float update_period);

  float compute(float error);

  bool is_settled();
};