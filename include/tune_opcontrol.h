#pragma once

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
#endif
