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

void ProsSmartPortTransport::pace_begin(std::uint32_t budget_bytes, std::uint32_t window_ms,
                                        PaceStallFn meanwhile, void* meanwhile_ctx) {
  // A zero budget or window would mean "never allowed to send", which would
  // hang the pump; treat it as "pacing off" instead of deadlocking.
  // 中文：預算或視窗填 0 等於「永遠不准送」，那會把 pump 卡死——當成關閉節流處理。
  if (budget_bytes == 0 || window_ms == 0) {
    pacing_ = false;
    return;
  }
  pacing_ = true;
  pacing_stall_ = false;
  pace_budget_ = budget_bytes;
  pace_window_ms_ = window_ms;
  pace_spent_ = 0;
  pace_window_start_ = millis();
  pace_meanwhile_ = meanwhile;
  pace_ctx_ = meanwhile_ctx;
}

void ProsSmartPortTransport::pace_end() {
  pacing_ = false;
  pacing_stall_ = false;
  pace_meanwhile_ = nullptr;
  pace_ctx_ = nullptr;
}

void ProsSmartPortTransport::pace_gate(std::size_t len) {
  // len is not consulted: the bucket is spent AFTER the write, against the
  // count actually sent, so a short write cannot over-charge the budget.
  // 中文：不看 len——預算是在寫入之後按「實際送出的數量」扣的，短寫就不會多扣。
  (void)len;
  // Roll the window forward, then spend. A frame larger than a whole window's
  // budget is still sent (it just consumes the next window outright) -- the
  // alternative is refusing to send it at all, and a truncated registration is
  // exactly the failure this whole mechanism exists to prevent.
  // 中文：先把視窗往前滾，再扣預算。單幀比整個視窗預算還大時照送（等於直接吃掉下一個
  // 視窗）——否則就變成「這一幀永遠送不出去」，而註冊送不完正是這套機制要防的事。
  for (;;) {
    const std::uint64_t now = millis();
    if (now - pace_window_start_ >= pace_window_ms_) {
      pace_window_start_ = now;
      pace_spent_ = 0;
      return;
    }
    if (pace_spent_ < pace_budget_) {
      return;  // room left in this window
    }
    // Bucket empty: give the caller a chance to move live data, then wait for
    // the window to roll. pacing_stall_ exempts whatever it writes, so the
    // callback cannot recurse back into this gate.
    // Called once per 1 ms wait, so up to ~9 times in a 10 ms window once the
    // budget is spent. That is cheap by construction, not by luck: the hook is
    // a telemetry flush(), and flush() with nothing buffered sends nothing and
    // returns. dashboard_update() only put()s every 25 ms, so at most one of
    // those ~9 calls has samples to ship; the rest are a compare and a return.
    // 中文：預算用完之後，每 1ms 等待呼叫一次，一個 10ms 視窗最多約 9 次。它便宜是設計
    // 使然、不是運氣：這個鉤子就是一次 telemetry flush()，而 flush() 在沒有緩衝樣本時
    // 什麼都不送、直接返回。dashboard_update() 每 25ms 才 put 一輪，所以那約 9 次裡
    // 最多只有一次真的有東西要送，其餘就是比一下然後 return。
    if (pace_meanwhile_ != nullptr) {
      pacing_stall_ = true;
      pace_meanwhile_(pace_ctx_);
      pacing_stall_ = false;
    }
    pros::delay(1);
  }
}

std::size_t ProsSmartPortTransport::write(const std::uint8_t* data, std::size_t len) {
  // Bounded retry (was: a single unchecked serial_.write() call -- see
  // bounded_write.h for why that truncated the link-up registration burst
  // and killed telemetry for good on 66799T). get_write_free() gates each
  // attempt to the FIFO's actual free space; a full FIFO waits pros::delay(1)
  // for it to drain instead of immediately truncating; the retry budget
  // (default 50ms of cumulative stall) is bounded so a permanently-stuck
  // link still returns (a short write) instead of hanging the pump task.
  // Registration trickle (see smartport_transport.h). Only active while the
  // registration burst is being replayed, and never for writes made from
  // inside the stall callback.
  // Gate only outside the stall callback (recursion), but CHARGE every write
  // made while pacing is on, including the callback's -- see the header note.
  // 中文：閘門只擋 stall callback 以外的寫入（避免遞迴），但只要涓流開著，**每一筆**
  // 寫入都要記帳，包含 callback 自己送的——理由見標頭檔。
  const bool gated = pacing_ && !pacing_stall_;
  const bool charged = pacing_;
  if (gated) {
    pace_gate(len);
  }
  BoundedWriteHooks hooks;
  hooks.write = &sp_raw_write;
  hooks.write_free = &sp_write_free;
  hooks.delay = &sp_delay;
  hooks.ctx = &serial_;
  const std::size_t sent = bounded_retry_write(hooks, data, len);
  if (charged) {
    pace_spent_ += static_cast<std::uint32_t>(sent);
  }
  return sent;
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
