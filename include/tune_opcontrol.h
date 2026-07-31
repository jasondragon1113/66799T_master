#pragma once

#include <cstdint>   // std::int32_t, for the tune_seg channel below

// PID-tuning teleop program -- a SEPARATE PROGRAM, not a mode.
//
// This is not a switch you flip on the field. It is a second build of the same
// project, produced by defining PID_TUNE_PROGRAM at compile time:
//
//     pros make tune                                  (or, the long way:)
//     pros make clean && pros make EXTRA_CXXFLAGS=-DPID_TUNE_PROGRAM
//     pros upload --slot 2 --name "66799T TUNE"
//
// With PID_TUNE_PROGRAM defined, opcontrol() calls tune_opcontrol() instead of
// the normal driving loop. With it undefined -- which is every ordinary build,
// including the one that goes in slot 1 for a match -- src/tune_opcontrol.cpp
// compiles to nothing at all and the driving code is byte-for-byte unchanged.
// There is no mode flag, no button combo, and therefore no way for a tuning
// action to fire during a match.
//
// 中文：調參用的「另一支遙控程式」，不是一個模式開關。
// 它跟正常程式是同一份原始碼、不同的編譯旗標：定義了 PID_TUNE_PROGRAM，
// opcontrol() 就改叫 tune_opcontrol()；沒定義（＝比賽要燒的那一版）的話，
// src/tune_opcontrol.cpp 整個檔案編出來是空的，駕駛程式一個位元組都沒變。
// 沒有模式旗標、沒有組合鍵，所以比賽中不可能誤觸發測試動作。
//
// 比賽版燒 slot 1、調參版燒 slot 2；比賽只用 slot 1。
//
// What the buttons do, how to abort, and the recommended tuning flow are all
// documented in PR_DESCRIPTION.md (章節「調參模式」) and at the top of
// src/tune_opcontrol.cpp.

// Never returns. Only defined when PID_TUNE_PROGRAM is set.
void tune_opcontrol();

#ifdef PID_TUNE_PROGRAM
// Cascade four-level height targets (N1): B=LEVEL0, Y=LEVEL1, X=LEVEL2,
// A=LEVEL3. Defined in tune_opcontrol.cpp; registered as dashboard sliders
// ("cascade/presets" group) in main.cpp's initialize(), same convention as the
// arm presets -- so the heights are tuned on the dashboard, not hardcoded.
// Only exists in the tuning build, like everything else in this program.
// 中文：調參版 B/Y/X/A 四段滑軌高度的目標值。定義在 tune_opcontrol.cpp，
// 在 main.cpp 的 initialize() 登記成 dashboard 滑桿（cascade/presets 群組），
// 跟手臂 preset 同一套慣例——高度在 dashboard 上調，不寫死。只存在於調參版。
extern double CASCADE_LV0_DEG;
extern double CASCADE_LV1_DEG;
extern double CASCADE_LV2_DEG;
extern double CASCADE_LV3_DEG;

// --- feedforward ramp test: the dashboard's "量前饋" (TuningFlow) panel --------
//
// That panel does not look for a mechanism or a group -- it looks for FIVE
// channels by these EXACT names (tuningFlow.ts accepts either the exact name or
// an `xxx_tune_*` prefixed one) and stays blank until all five arrive:
//
//   tune_ms    ms since the ramp started
//   tune_volt  the command being applied RIGHT NOW
//   tune_pos   mechanism position
//   tune_vel   mechanism velocity
//   tune_seg   which leg: 0 = not running, 1 = forward, 2 = reverse
//
// Names and semantics follow the reference implementation in DPLIB
// (dplib/src/telemetry.cpp:84-88 registers them, :284-287 feeds them from a
// TuneSample), so the same panel works against both robots.
//
// UNITS: tune_volt is in PROS move() command units (-127..127), NOT volts. That
// is deliberate -- it is the same scale as ARM_KG / CASCADE_KG, so a feedforward
// number worked out from this data can be typed straight into those sliders with
// no conversion step for anyone to get wrong.
// 中文：dashboard 的「量前饋」面板不是照機構或群組找資料，而是照上面那五個**固定名字**
// 找頻道（tuningFlow.ts 接受精準名或 `xxx_tune_*` 前綴），五條沒到齊就整頁空白。
// 名字與語意照 DPLIB 的參考實作（telemetry.cpp:84-88 登記、:284-287 餵值）。
// 單位注意：tune_volt 用的是 PROS move() 的指令刻度（-127~127），不是伏特。這是故意的
// ——那正是 ARM_KG／CASCADE_KG 的刻度，從這批資料算出來的前饋值可以直接填進那兩顆滑桿，
// 中間不會有一個換算步驟讓人搞錯。
extern double TUNE_CH_MS;
extern double TUNE_CH_VOLT;
extern double TUNE_CH_POS;
extern double TUNE_CH_VEL;
extern std::int32_t TUNE_CH_SEG;

// Dashboard command handlers (registered with requires_confirm, so the panel asks
// before the robot moves). They only RAISE A REQUEST: they are called on the
// vexdash pump task, and starting a motion from there would be driving the robot
// from a background thread with none of the tuning program's safety state around
// it. The main loop picks the request up on its next 10 ms pass and runs the ramp
// itself -- the same handshake the chassis test moves already use.
// 中文：dashboard 按鈕的處理函式（登記時帶 requires_confirm，所以面板會先跳確認才動）。
// 它們只負責「舉手登記」：它們是在 vexdash 的背景 task 上被呼叫的，直接在那裡讓機器人
// 動起來，等於用一條背景執行緒開車，而且完全繞過調參程式的安全狀態。主迴圈會在下一個
// 10ms 迴圈接手、由主迴圈自己跑斜坡——跟底盤測試動作用的是同一套握手。
void tune_ff_ramp_arm_command(void* user_data);
void tune_ff_ramp_cascade_command(void* user_data);

// Manual capture (TOGGLE: press to start the run, press again to end it -- the
// end is what makes the panel fit). While a capture runs, the five channels
// above are published from ordinary movement, so the coach can move the arm with
// the D-pad keys and still get a feedforward fit. Mutually exclusive with the
// ramp: starting a ramp closes an open capture first.
// 中文：手動擷取（切換式：按一下開始、再按一下結束——「結束」才會讓面板做擬合）。擷取
// 期間上面那五條頻道會照實反映當下的動作，所以教練用方向鍵把手臂動一動也能擬合出前饋。
// 與斜坡互斥：啟動斜坡會先把還開著的擷取收掉。
void tune_ff_capture_arm_command(void* user_data);
void tune_ff_capture_cascade_command(void* user_data);
#endif
