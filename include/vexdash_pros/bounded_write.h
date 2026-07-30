#pragma once

#include <cstddef>
#include <cstdint>

// bounded_retry_write -- the platform-INDEPENDENT bounded-retry write loop
// shared by the Smart Port transport (and usable by any future non-blocking
// transport). Deliberately contains NO PROS headers so it can be unit-tested
// on the host with mock hooks (see lib-pros/tests/test_bounded_write.cpp).
// The PROS layer (ProsSmartPortTransport::write, smartport_transport.cpp)
// wires the hooks to pros::Serial::write / get_write_free / pros::delay.
//
// WHY this exists (root cause of the 66799T "telemetry dead after handshake"
// bug): the link-up registration burst is ~1.5KB of CHANNEL_DEF/CONFIG_SCHEMA
// frames sent back-to-back. Smart Port's pros::Serial::write() is
// non-blocking and can return fewer bytes than requested when the TX FIFO is
// full -- the old code (a single unchecked write() call) silently truncated
// the frame in that case. A truncated frame fails CRC on the dashboard side,
// which permanently marks that channel's registration as failed; telemetry
// for it never recovers for the rest of the session. This loop retries
// (checking get_write_free()/delaying 1ms to let the FIFO drain) until the
// whole buffer is sent, OR a bounded retry budget is exhausted -- at which
// point it gives up and returns the short count instead of retrying forever.
//
// 中文：為什麼要流控（66799T「握手後遙測全死」的根因）——link-up 首波註冊 burst
// 約 1.5KB 的 CHANNEL_DEF/CONFIG_SCHEMA 幀連續送出。Smart Port 的
// pros::Serial::write() 是非阻塞的，TX FIFO 滿時可能只送出部分位元組——舊碼
// （單次無檢查的 write() 呼叫）在這種情況下會悄悄截斷幀。截斷幀在 dashboard 端
// CRC 驗證失敗，導致該頻道的註冊被永久標記失敗，該頻道遙測整個 session 都救不
// 回來。這個迴圈會不斷重試（查 get_write_free()／delay(1) 讓 FIFO 排空），直到
// 整包送完，或重試預算耗盡才放棄回傳短寫計數——絕不無限重試卡死。

namespace vexdash {

// Raw non-blocking write: writes up to `len` bytes starting at `data`,
// returns the count actually written (may be 0..len), or a negative value
// (e.g. PROS_ERR) on a HARD error such as an invalid port number. A negative
// return is NOT retried -- bounded_retry_write() gives up immediately on
// that call instead of burning the retry budget on a call that can never
// succeed (e.g. a misconfigured port would otherwise cost ~max_stall_ms of
// delay() EVERY frame, forever). ctx is opaque hook context (e.g. a
// pros::Serial*).
// 中文：raw 非阻塞寫入：回傳實際寫入數（可以是 0..len），或負值（如
// PROS_ERR）代表硬錯誤（例如埠號無效）。負值不會重試——bounded_retry_write()
// 該次呼叫直接放棄，不會把重試預算燒在一個永遠不可能成功的呼叫上（否則埠號設
// 錯時，每一幀都要白白燒掉 ~max_stall_ms 的 delay()，永無止盡）。
using RawWriteFn = std::int32_t (*)(void* ctx, const std::uint8_t* data, std::int32_t len);

// Returns bytes currently free in the underlying FIFO, or a negative value if
// unknown/error. May be nullptr if the transport can't report free space --
// the loop then skips the space gate and just retries the write itself.
using WriteFreeFn = std::int32_t (*)(void* ctx);

// Delays approximately `ms` milliseconds (a scheduler yield point), letting
// the FIFO drain before the next retry.
using DelayFn = void (*)(void* ctx, std::uint32_t ms);

struct BoundedWriteHooks {
  RawWriteFn write = nullptr;       // required
  WriteFreeFn write_free = nullptr;  // optional (nullable)
  DelayFn delay = nullptr;          // required for retries to make progress
  void* ctx = nullptr;
};

// Writes `len` bytes from `data` via hooks.write, retrying bounded on short
// writes / a full FIFO.
//
// Two independent bounds, both counted in delay(1) ticks (~ms):
//   - `max_stall_ms`: consecutive stalled ticks with NO progress at all.
//     Resets to 0 the moment any byte gets written -- this alone only
//     catches a truly stuck/permanently-congested sink, NOT a slow drip.
//   - `max_total_ms`: an ABSOLUTE budget for the whole call, never reset by
//     progress. Why this is needed on top of max_stall_ms: an adversarial or
//     just very slow sink that trickles in a handful of bytes every stall
//     tick (e.g. 1 byte every ~49ms) keeps "making progress", so
//     max_stall_ms alone never trips -- a 512-byte frame at that drip rate
//     would block for ~25 seconds before finishing. max_total_ms caps the
//     WORST CASE regardless of how progress is distributed, so a single
//     write() call can never eat more than this much wall-clock-equivalent
//     time; default 250ms comfortably covers a real, recovering FIFO
//     (observed on hardware: ~137ms for a realistic draining burst) while
//     still bounding the pathological drip case.
// 中文：兩個獨立的上限，都以 delay(1) 的 tick 數（約當 ms）計算：
//   - `max_stall_ms`：連續無進度的 tick 數，只要送出任何一個 byte 就歸零——
//     單獨這個只能擋住「真的卡死、永久壅塞」的情況，擋不住慢慢滴的壅塞。
//   - `max_total_ms`：整次呼叫的「絕對」總預算，不會因為有進度而重置。為什麼
//     需要它：對手式或單純很慢的接收端，只要每個 stall tick 都滴進幾個
//     byte（例如每 ~49ms 滴 1 byte），就會一直「有進度」，讓 max_stall_ms
//     永遠不會觸發——這樣一個 512-byte 的幀在這種滴速下會卡住約 25 秒才送完。
//     max_total_ms 不管進度怎麼分布，都幫最糟情況設一個天花板：單次 write()
//     呼叫絕不會吃掉超過這麼多「約當時間」；預設 250ms 在實機觀察到的正常
//     排空（約 137ms）留有餘裕，同時仍能擋住病態的滴水情境。
//
// Once either bound is exhausted, returns whatever was written so far (may
// be < len: a short write, reported to the caller instead of hanging).
std::size_t bounded_retry_write(const BoundedWriteHooks& hooks, const std::uint8_t* data,
                                 std::size_t len, std::uint32_t max_stall_ms = 50,
                                 std::uint32_t max_total_ms = 250);

}  // namespace vexdash
