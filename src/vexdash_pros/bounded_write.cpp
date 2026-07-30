#include "vexdash_pros/bounded_write.h"

// 平台無關（無 PROS header），納入 host build 與單元測試。見 bounded_write.h。

namespace vexdash {

namespace {

// Shared bound check + delay for both stall paths below (free==0 and
// write()==0). Returns false if either budget is exhausted (caller must stop
// retrying), true if it delayed and the caller may retry.
// 中文：兩個 stall 路徑（free==0 與 write()==0）共用的上限檢查＋delay。任一
// 預算耗盡回 false（呼叫端該停手），delay 過後仍有預算則回 true 可以再試。
bool tick_or_bail(const BoundedWriteHooks& hooks, std::uint32_t max_stall_ms,
                   std::uint32_t max_total_ms, std::uint32_t& stalled_ms, std::uint32_t& total_ms) {
  if (hooks.delay == nullptr) return false;
  if (stalled_ms >= max_stall_ms) return false;   // no-progress budget exhausted
  if (total_ms >= max_total_ms) return false;     // absolute wall-clock-equivalent budget exhausted (P2-1)
  hooks.delay(hooks.ctx, 1);
  ++stalled_ms;
  ++total_ms;
  return true;
}

}  // namespace

std::size_t bounded_retry_write(const BoundedWriteHooks& hooks, const std::uint8_t* data,
                                 std::size_t len, std::uint32_t max_stall_ms,
                                 std::uint32_t max_total_ms) {
  if (hooks.write == nullptr) return 0;

  std::size_t total = 0;
  std::uint32_t stalled_ms = 0;  // resets on any progress -- catches "truly stuck"
  std::uint32_t total_ms = 0;    // NEVER resets -- catches "drips forever" (P2-1)

  while (total < len) {
    std::int32_t remain = static_cast<std::int32_t>(len - total);
    std::int32_t room = remain;  // default: attempt the whole remainder

    if (hooks.write_free != nullptr) {
      std::int32_t free = hooks.write_free(hooks.ctx);
      if (free == 0) {
        // FIFO reports zero room right now -- don't even attempt the write,
        // just wait a tick for it to drain (bounded, both budgets checked).
        if (!tick_or_bail(hooks, max_stall_ms, max_total_ms, stalled_ms, total_ms)) break;
        continue;
      }
      if (free > 0) {
        room = free < remain ? free : remain;
      }
      // free < 0 (error/unknown): fall through and just try the full
      // remainder, same as if no write_free hook were installed.
    }

    std::int32_t n = hooks.write(hooks.ctx, data + total, room);
    if (n > 0) {
      total += static_cast<std::size_t>(n);
      stalled_ms = 0;  // made progress -> reset the NO-PROGRESS budget only;
                        // total_ms is intentionally left untouched (P2-1: an
                        // absolute cap that repeated tiny progress cannot dodge)
    } else if (n < 0) {
      // Hard error (e.g. PROS_ERR from a bad port number) -- not transient,
      // retrying cannot help. Give up immediately instead of burning the
      // stall budget on a call that can never succeed (P3-1).
      // 中文：硬錯誤（如埠號錯的 PROS_ERR）——不是暫時性，重試沒有意義，直接
      // 放棄，不要把重試預算燒在一個永遠不會成功的呼叫上。
      break;
    } else {
      // n == 0: transient short write (room was offered but nothing landed) -- bounded retry.
      if (!tick_or_bail(hooks, max_stall_ms, max_total_ms, stalled_ms, total_ms)) break;
    }
  }

  return total;
}

}  // namespace vexdash
