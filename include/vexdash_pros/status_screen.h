#pragma once

// StatusScreen -- draws the vexdash ConnectionPump status onto the V5 Brain
// screen so the user can debug while plugged in. CONTAINS PROS HEADERS:
// compiled ONLY inside a PROS project, never in the host build (see
// lib-pros/CMakeLists.txt isolation). The interesting logic (the Status
// snapshot + format_status_lines) lives in the PROS-free, host-tested
// ConnectionPump; this class only pushes the formatted lines to the screen.
//
// [NEEDS-HW-VERIFICATION] All PROS screen calls below are unverified on
// hardware (no PROS toolchain / no Brain in this environment).
//
// Why LLEMU (pros::lcd) rather than pros::screen:
//   * The HUD is pure text (7 short lines). LLEMU gives exactly 8 text rows
//     on the 480x272 screen with a one-call-per-line API
//     (pros::lcd::set_text(row, str)) -- a direct fit for
//     format_status_lines() output, no manual font/coordinate math.
//   * pros::screen would require computing text baselines/rows by hand.
//   * Tradeoff: LLEMU text is a single foreground color, so UP/DOWN is
//     distinguished in TEXT ("LINK:UP"/"LINK:DOWN") rather than by color.
//     If colored UP=green/DOWN=red is wanted later, switch this file to
//     pros::screen::set_pen + print (the ConnectionPump/format layer does
//     not change). See docs/pros-bringup-checklist.md.
//
// Screen drawing and serial are independent resources and do not interfere;
// still, redraw at a modest rate (5-10 Hz) off the hot path, not every tick.

#include <cstdint>

#include "vexdash_pros/connection_pump.h"

namespace vexdash {

class StatusScreen {
 public:
  // `pump` must outlive this object. `redraw_period_ms` throttles redraws.
  explicit StatusScreen(ConnectionPump& pump, std::uint32_t redraw_period_ms = 150);

  // Initializes the LLEMU display (idempotent-safe). MUST be called from the
  // main/user context (e.g. initialize()) BEFORE the pump task starts:
  // pros::lcd::initialize() from a background task deadlocks that task
  // (observed on hardware, WORKLOG 2026-07-08). The façade does this for you.
  void init();

  // If at least redraw_period_ms have elapsed since the last redraw, pulls
  // a fresh Status snapshot, formats it, and writes the lines to the Brain
  // screen. `now_ms` is pros::millis(). Cheap to call every task cycle.
  void update(std::uint32_t now_ms);

 private:
  ConnectionPump& pump_;
  std::uint32_t redraw_period_ms_;
  std::uint32_t last_redraw_ms_ = 0;
  bool initialized_ = false;
  bool first_ = true;
};

}  // namespace vexdash
