// [NEEDS-HW-VERIFICATION] Compiled only inside a PROS project. See header
// and docs/pros-bringup-checklist.md for the mechanism and sources.

#include "vexdash_pros/smartport_transport.h"

#include "api.h"  // pros::millis, pros::delay
#include "vexdash_pros/bounded_write.h"

namespace vexdash {

namespace {

// Hook adapters wiring the platform-independent bounded_retry_write() (see
// bounded_write.h for the WHY -- FIFO-full short writes truncating the
// link-up registration burst) to this pros::Serial instance.
// 中文：把 host 可測的 bounded_retry_write() 接到這個 pros::Serial 實例上的
// hook 轉接函式（為什麼需要見 bounded_write.h：FIFO 滿→短寫→截斷註冊 burst）。

std::int32_t sp_raw_write(void* ctx, const std::uint8_t* data, std::int32_t len) {
  auto* serial = static_cast<pros::Serial*>(ctx);
  // pros::Serial::write takes a non-const uint8_t*; the bytes are not
  // modified. Returns count written or PROS_ERR (-1) on failure.
  return serial->write(const_cast<std::uint8_t*>(data), len);
}

std::int32_t sp_write_free(void* ctx) {
  auto* serial = static_cast<pros::Serial*>(ctx);
  return serial->get_write_free();
}

void sp_delay(void*, std::uint32_t ms) { pros::delay(ms); }

}  // namespace

ProsSmartPortTransport::ProsSmartPortTransport(std::uint8_t smart_port, std::int32_t baudrate)
    : serial_(smart_port, baudrate) {}

std::size_t ProsSmartPortTransport::write(const std::uint8_t* data, std::size_t len) {
  // Bounded retry (was: a single unchecked serial_.write() call -- see
  // bounded_write.h for why that truncated the link-up registration burst
  // and killed telemetry for good on 66799T). get_write_free() gates each
  // attempt to the FIFO's actual free space; a full FIFO waits pros::delay(1)
  // for it to drain instead of immediately truncating; the retry budget
  // (default 50ms of cumulative stall) is bounded so a permanently-stuck
  // link still returns (a short write) instead of hanging the pump task.
  BoundedWriteHooks hooks;
  hooks.write = &sp_raw_write;
  hooks.write_free = &sp_write_free;
  hooks.delay = &sp_delay;
  hooks.ctx = &serial_;
  return bounded_retry_write(hooks, data, len);
}

std::size_t ProsSmartPortTransport::bytes_available() {
  std::int32_t n = serial_.get_read_avail();
  if (n <= 0) return 0;
  return static_cast<std::size_t>(n);
}

std::size_t ProsSmartPortTransport::read(std::uint8_t* out, std::size_t len) {
  // pros::Serial::read is non-blocking: returns only currently-available
  // bytes, or PROS_ERR. Honors the ITransport no-block contract.
  std::int32_t n = serial_.read(out, static_cast<std::int32_t>(len));
  if (n <= 0) return 0;
  return static_cast<std::size_t>(n);
}

std::uint64_t ProsSmartPortTransport::millis() {
  return static_cast<std::uint64_t>(pros::millis());
}

}  // namespace vexdash
