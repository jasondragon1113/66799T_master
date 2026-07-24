// [NEEDS-HW-VERIFICATION] Compiled only inside a PROS project.

#include "vexdash_pros/pros_task.h"

#include "api.h"  // pros::delay, pros::millis

namespace vexdash {

ProsTask::ProsTask(ConnectionPump& pump, StatusScreen* screen, std::uint32_t tick_period_ms)
    : pump_(pump),
      screen_(screen),
      tick_period_ms_(tick_period_ms),
      task_(&ProsTask::trampoline, this, "vexdash_pump") {}

ProsTask::~ProsTask() {
  stop_ = true;
  // Give the loop one period to observe stop_ and return, then reclaim.
  pros::delay(tick_period_ms_ + 5);
  task_.remove();
}

void ProsTask::trampoline(void* self) { static_cast<ProsTask*>(self)->run(); }

void ProsTask::run() {
  // NOTE: no screen_->init() here on purpose -- pros::lcd::initialize() from
  // a background task deadlocks it. The façade init()s the screen in the
  // caller's context before constructing this task.
  while (!stop_) {
    std::uint32_t now = static_cast<std::uint32_t>(pros::millis());
    pump_.tick(now);
    if (screen_ != nullptr) screen_->update(now);  // self-throttled redraw
    pros::delay(tick_period_ms_);
  }
}

}  // namespace vexdash
