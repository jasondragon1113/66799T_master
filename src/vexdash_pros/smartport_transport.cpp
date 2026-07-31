// [NEEDS-HW-VERIFICATION] Compiled only inside a PROS project. See header
// and docs/pros-bringup-checklist.md for the mechanism and sources.

#include "vexdash_pros/smartport_transport.h"

#include <cstring>  // std::memcpy for the capture queue

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

void ProsSmartPortTransport::capture_begin() {
  // Always starts from empty. A reconnect landing mid-playback restarts the
  // catalogue from the top, which is exactly right -- the dashboard that just
  // reconnected has nothing, so replaying the tail alone would leave it with a
  // half-registry it could never complete.
  // 中文：一律從空的開始。重連剛好落在播放中途時，目錄會從頭重播——這正是對的：
  // 剛連上的 dashboard 手上什麼都沒有，只補後半段等於讓它永遠停在半份目錄。
  capturing_ = true;
  queue_len_ = 0;
  queue_read_ = 0;
  frame_count_ = 0;
  frame_next_ = 0;
  capture_overflowed_ = false;
}

void ProsSmartPortTransport::capture_end() { capturing_ = false; }

bool ProsSmartPortTransport::playback_step(std::size_t max_frames) {
  for (std::size_t i = 0; i < max_frames && frame_next_ < frame_count_; ++i) {
    const std::size_t len = frame_len_[frame_next_];
    // Straight to the wire: this is the ordinary send path, flow-controlled by
    // bounded_retry_write like any other frame. The spacing is the pump's job,
    // not this function's.
    // 中文：直接送上線，跟其他幀一樣走 bounded_retry_write 流控。間隔是 pump 的職責，
    // 不是這個函式的事。
    write(queue_ + queue_read_, len);
    queue_read_ += len;
    ++frame_next_;
  }
  return frame_next_ < frame_count_;
}

std::size_t ProsSmartPortTransport::write(const std::uint8_t* data, std::size_t len) {
  // Bounded retry (was: a single unchecked serial_.write() call -- see
  // bounded_write.h for why that truncated the link-up registration burst
  // and killed telemetry for good on 66799T). get_write_free() gates each
  // attempt to the FIFO's actual free space; a full FIFO waits pros::delay(1)
  // for it to drain instead of immediately truncating; the retry budget
  // (default 50ms of cumulative stall) is bounded so a permanently-stuck
  // link still returns (a short write) instead of hanging the pump task.
  // Capture mode (see smartport_transport.h): buffer whole frames instead of
  // sending them, so the registration callback never touches the wire.
  // playback_step() replays them one at a time from the pump's tick.
  // ONE write() IS ONE COMPLETE FRAME (Session encodes, then writes once),
  // which is what makes a byte queue plus a length table enough to rebuild
  // the frame boundaries on the way out.
  // 中文：錄製模式（見標頭）——整幀存起來、不送出，註冊回呼因此完全不碰線路，
  // 之後由 playback_step() 在 pump 的 tick 裡一幀一幀播。**一次 write() 就是一個完整的幀**
  // （Session 先編碼再一次寫出），所以「位元組佇列＋長度表」就足以還原幀邊界。
  if (capturing_) {
    if (frame_count_ < kRegQueueFrames && queue_len_ + len <= kRegQueueBytes &&
        len <= 0xFFFF) {
      std::memcpy(queue_ + queue_len_, data, len);
      frame_len_[frame_count_++] = static_cast<std::uint16_t>(len);
      queue_len_ += len;
      return len;
    }
    // Queue full: stop capturing and let this frame and everything after it
    // go out normally. Tighter on the wire for the tail, but never truncated.
    // 中文：佇列滿了就停止錄製，這一幀與之後的照常直接送出——尾巴會比較擠，
    // 但絕不會被截斷。
    capturing_ = false;
    capture_overflowed_ = true;
  }
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
