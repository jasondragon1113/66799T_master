#pragma once

// ProsSmartPortTransport -- vexdash ITransport over a V5 Smart Port using
// PROS generic serial (pros::Serial). For the wireless path (ESP32/RS-485
// bridge) or a USB-RS485 dongle wired to a Smart Port. CONTAINS PROS
// HEADERS: compiled only inside a PROS project (host build excludes it).
//
// [NEEDS-HW-VERIFICATION] Not run on hardware from this environment.
//
// pros::Serial (pros/serial.hpp) API used (verified against PROS docs, see
// docs/pros-bringup-checklist.md for URLs):
//   Serial(uint8_t port, int32_t baudrate)     -- port 1..21
//   int32_t write(uint8_t* buf, int32_t len)   -- returns count or PROS_ERR
//   int32_t read(uint8_t* buf, int32_t len)    -- non-blocking, count/PROS_ERR
//   int32_t get_read_avail()                   -- available input bytes/PROS_ERR

#include <cstddef>
#include <cstdint>

#include "pros/apix.h"  // pros::Serial (extended API; includes api.h in correct order, avoids literals clash)
#include "vexdash/transport.h"

namespace vexdash {

class ProsSmartPortTransport : public ITransport {
 public:
  // `smart_port` is 1..21 (Brain smart port number). `baudrate` must match
  // the far end (e.g. the ESP32 bridge). 921600 is the project target;
  // 115200 is the safe fallback (protocol.md §7).
  ProsSmartPortTransport(std::uint8_t smart_port, std::int32_t baudrate);

  std::size_t write(const std::uint8_t* data, std::size_t len) override;
  std::size_t bytes_available() override;
  std::size_t read(std::uint8_t* out, std::size_t len) override;
  std::uint64_t millis() override;

  // ---- registration trickle (2026-07-31, 66994V) --------------------------
  //
  // THE BUG THIS EXISTS FOR, measured on a stationary robot: the dashboard's
  // link quality score dropped to 24 on a regular ~5 second beat while nothing
  // was moving. 5 s is registration_resend_period_ms, and the resend replays
  // the WHOLE registry in one synchronous burst -- around 120 frames / 8 KB
  // with no gap between frames. The Brain's Smart Port FIFO drains fine; the
  // ESP32 on the far end does not, so its receive buffer overruns partway
  // through and everything still in flight is lost mid-COBS-frame. The link
  // does not go down, it just shreds one packet in every burst, forever.
  //
  // bounded_retry_write() cannot see this: it flow-controls against the LOCAL
  // FIFO, which the Brain empties happily. The far end has no back-pressure
  // channel at all, so the only fix is to not send faster than it can absorb.
  //
  // pace_begin() puts the transport into a token-bucket mode: at most
  // `budget_bytes` per `window_ms`, sleeping when the bucket is empty. Used
  // ONLY around the registration burst (link-up AND periodic resend) -- normal
  // telemetry is nowhere near the rate that causes this and stays unpaced.
  //
  // `meanwhile` is called each time the bucket runs dry, before the sleep, so
  // the caller can keep live data moving instead of the graph freezing for the
  // length of the trickle. Writes made from inside it SKIP THE GATE but are
  // still CHARGED to the same bucket. Both halves matter:
  //   * skipping the gate is what stops the callback recursing into the very
  //     wait that invoked it (it would deadlock against its own budget);
  //   * charging it is what keeps the budget honest. Steady-state telemetry on
  //     this robot is around 14 KB/s, which is the SAME ORDER as the 16 KB/s
  //     registration budget -- not "negligible". Exempting it from the count
  //     would put ~30 KB/s on the wire during a trickle, double the configured
  //     figure, which is the rate the ESP32 could not absorb in the first place.
  // 中文：桶空時會先呼叫 meanwhile 再睡，讓呼叫端趁空檔把即時資料送出去。從 meanwhile
  // 發出的寫入**不經過閘門、但一樣要記帳**，兩件事都必要：
  //   * 不經過閘門，callback 才不會遞迴回叫出它的那個等待（會對著自己的預算死鎖）；
  //   * 要記帳，預算才誠實。本車穩態遙測約 14KB/s，跟註冊預算 16KB/s **是同一個量級**，
  //     不是「小到可以忽略」。不記帳的話，涓流期間線上實際會有約 30KB/s，是設定值的兩倍，
  //     而那正是 ESP32 一開始吃不下的速率。
  //
  // 中文：**這段程式是為了一個實測到的 bug**：車子完全靜止時，dashboard 的連線品質分數
  // 每隔約 5 秒就規律掉到 24。5 秒正是 registration_resend_period_ms，而重送會把整份
  // 登記表在一個同步迴圈裡一次打出去——約 120 幀、8KB，幀與幀之間完全沒有間隔。
  // Brain 這端的 FIFO 排得掉，ESP32 那端排不掉：接收緩衝在半途溢位，還在路上的位元組
  // 整段掉在 COBS 幀中間。連線不會斷，只是每一輪爆發都固定撕掉一個封包，永遠如此。
  // bounded_retry_write() 看不到這件事——它是對**本地** FIFO 做流控，而本地根本不塞；
  // 對端沒有任何反壓通道，所以唯一的解法就是「不要送得比對方吃得下還快」。
  // pace_begin() 讓 transport 進入權杖桶模式：每 window_ms 最多送 budget_bytes，桶空就睡。
  // **只用在註冊爆發上**（開機首次註冊與週期重送都算），一般遙測遠低於這個速率，不受影響。
  // 桶空時會先呼叫 meanwhile 再睡，讓呼叫端可以趁空檔把即時資料送出去，圖表不會整段凍住；
  // 從 meanwhile 裡發出的寫入**不受節流**（見 pacing_stall_），否則它會遞迴回自己這道閘。
  using PaceStallFn = void (*)(void* ctx);
  void pace_begin(std::uint32_t budget_bytes, std::uint32_t window_ms, PaceStallFn meanwhile,
                  void* meanwhile_ctx);
  void pace_end();
  // True while a trickle is in progress. Re-entrancy guard for the façade:
  // a second pace_begin() must not restart the bucket underneath the first.
  // 中文：涓流進行中為 true。門面用它擋重入——第二次 pace_begin() 不可以把第一次的桶重置。
  bool pacing() const { return pacing_; }

 private:
  // Blocks until `len` more bytes fit in this window's budget.
  void pace_gate(std::size_t len);

  pros::Serial serial_;

  bool pacing_ = false;
  bool pacing_stall_ = false;  // inside meanwhile(): nested writes bypass the gate
  std::uint32_t pace_budget_ = 0;
  std::uint32_t pace_window_ms_ = 0;
  std::uint32_t pace_spent_ = 0;
  std::uint64_t pace_window_start_ = 0;
  PaceStallFn pace_meanwhile_ = nullptr;
  void* pace_ctx_ = nullptr;
};

}  // namespace vexdash
