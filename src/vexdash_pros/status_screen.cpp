// [NEEDS-HW-VERIFICATION] Compiled only inside a PROS project. See header
// and docs/pros-bringup-checklist.md for the mechanism and sources.

#include "vexdash_pros/status_screen.h"

#include "api.h"          // set up pros namespaces first
#include "pros/llemu.hpp"  // pros::lcd (LLEMU)

namespace vexdash {

namespace {
// LLEMU provides 8 text rows (0..7). Each row comfortably holds the short
// status lines; kLineCap bounds our formatting buffer.
constexpr int kMaxLines = 8;
constexpr int kLineCap = 40;  // chars incl. NUL; well within LLEMU row width
}  // namespace

StatusScreen::StatusScreen(ConnectionPump& pump, std::uint32_t redraw_period_ms)
    : pump_(pump), redraw_period_ms_(redraw_period_ms) {}

void StatusScreen::init() {
  if (initialized_) return;
  pros::lcd::initialize();  // safe to call once; sets up the 8-line text UI
  initialized_ = true;
}

void StatusScreen::update(std::uint32_t now_ms) {
  // Never lazy-init from here: update() runs on the pump task, and calling
  // pros::lcd::initialize() from a background task deadlocks it. init() must
  // have been called from the main context; until then, draw nothing.
  if (!initialized_) return;

  if (!first_ && (now_ms - last_redraw_ms_) < redraw_period_ms_) {
    return;  // throttle: don't redraw every cycle
  }
  first_ = false;
  last_redraw_ms_ = now_ms;

  // Pull + format the status (all PROS-free, host-tested logic).
  char lines[kMaxLines * kLineCap];
  Status st = pump_.status();
  int n = format_status_lines(st, lines, kMaxLines, kLineCap);

  for (int i = 0; i < n; ++i) {
    pros::lcd::set_text(i, lines + static_cast<std::size_t>(i) * kLineCap);
  }
  // Clear any leftover rows below the ones we wrote (in case a previous
  // frame had more lines).
  for (int i = n; i < kMaxLines; ++i) {
    pros::lcd::clear_line(i);
  }
}

}  // namespace vexdash
