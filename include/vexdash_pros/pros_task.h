#pragma once

// ProsTask -- thin pros::Task wrapper that runs a ConnectionPump on a fixed
// period. CONTAINS PROS HEADERS: compiled only inside a PROS project.
//
// All the interesting logic (heartbeat/reconnect/rate-limit) lives in the
// host-testable ConnectionPump; this class only owns the background task
// and calls pump.tick(pros::millis()) every `tick_period_ms`.
//
// [NEEDS-HW-VERIFICATION] pros::Task scheduling behavior on hardware.

#include <cstdint>

#include "api.h"  // pros::Task/delay/millis via umbrella
#include "vexdash_pros/connection_pump.h"
#include "vexdash_pros/status_screen.h"

namespace vexdash {

class ProsTask {
 public:
  // `pump` must outlive this task. `tick_period_ms` is the background loop
  // cadence; keep it <= the pump's smallest configured period (default
  // telemetry_period_ms=20 -> a 10ms tick is a good default). If `screen`
  // is non-null, the same task also drives the Brain status HUD (the screen
  // self-throttles its redraw, so it never dominates the loop).
  ProsTask(ConnectionPump& pump, StatusScreen* screen = nullptr,
           std::uint32_t tick_period_ms = 10);
  ~ProsTask();

  ProsTask(const ProsTask&) = delete;
  ProsTask& operator=(const ProsTask&) = delete;

 private:
  static void trampoline(void* self);
  void run();

  ConnectionPump& pump_;
  StatusScreen* screen_;
  std::uint32_t tick_period_ms_;
  bool stop_ = false;
  pros::Task task_;
};

}  // namespace vexdash
