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

  // ---- registration playback (trickle v2, 2026-07-31, 66994V) -------------
  //
  // SYMPTOM HISTORY, because the second attempt only makes sense against it:
  //  * v0, no pacing: the whole registry went out back to back, ~90-120 frames
  //    / ~8 KB at ~92 KB/s. The dashboard's link quality score sat around 65
  //    and collapsed to ~24 on the 5 s resend beat with the robot stationary.
  //    The Brain's FIFO drains fine; the ESP32's small RX buffer does not, and
  //    what overruns is lost mid-COBS-frame.
  //  * v1, blocking token bucket at 320 B / 10 ms: baseline rose to 77-82, but
  //    the 5 s dips remained (22-45). A 0.25 s sprint at 32 KB/s still leaves
  //    the far end only ~8 ms of slack; one millisecond-scale WiFi stall inside
  //    that window and the buffer is over again. Making the sprint slower ran
  //    into the opposite wall: the pump blocks inside the pacing loop, so a
  //    long burst stops sending PINGs and approaches link_timeout_ms (3000).
  //
  // v2 removes the sprint instead of resizing it. There is no burst to survive
  // a stall if the registry is never sent as a burst.
  //
  // HOW: capture, then play back. capture_begin() switches write() from "send
  // now" to "append to a frame queue"; the registration callback then runs to
  // completion in microseconds, touching no wire at all. The pump afterwards
  // calls drain() once per tick and sends ONE frame, alongside its normal
  // PING / poll / telemetry work. Nothing blocks, ever.
  //
  // LOAD MATHS, which is the whole point. Telemetry publishes on a 25 ms period
  // (~33 Hz measured), so at rest the wire sees a ~350 B frame every ~30 ms:
  // ~11 KB/s AVERAGE with a ~350 B PEAK in any one 10 ms window. Playback adds
  // one ~70 B definition frame every 20 ms:
  //   average  ~11 -> ~15 KB/s   (1.3x)
  //   peak     measured 1.11x of the resting peak (model bound 1.20x, i.e. the
  //            worst case where a definition frame shares a window with a
  //            telemetry frame)
  // Compare v1: 32 KB/s sustained for 250 ms, ~2.8x the average, all of it in
  // one burst. The catalogue is ~132 frames, so playback takes ~2.6 s and the
  // whole cycle (playback + the 5 s idle period) is ~7.6 s. That latency is
  // the trade v2 deliberately makes: a slower catalogue for a flat wire.
  // Both figures are peak-vs-peak and average-vs-average -- do not mix them.
  //
  // 中文：**先講症狀史，第二版才有意義**：
  //  * v0 完全不節流：整份登記表連續打完（約 90-120 幀 / 8KB，瞬時約 92KB/s）。品質分基線
  //    約 65，每 5 秒重送時掉到約 24。Brain 的 FIFO 排得掉，ESP32 的小 RX 緩衝排不掉，
  //    溢出的部分掉在 COBS 幀中間。
  //  * v1 阻塞式權杖桶 320B/10ms：基線升到 77-82，**但 5 秒週期仍掉到 22-45**。0.25 秒、
  //    32KB/s 的衝刺只留給對端約 8ms 餘裕，WiFi 只要在那個窗口裡卡頓一下就又爆。而把衝刺
  //    放慢會撞到另一面牆：pump 卡在節流迴圈裡不送 PING，逼近 link_timeout_ms（3000）。
  // **v2 不是把衝刺調小，是把衝刺拿掉**——只要不是用爆發送的，就沒有「窗口」可以被打爆。
  // 做法：**先錄再播**。capture_begin() 讓 write() 從「立刻送」改成「存進幀佇列」，註冊回呼
  // 因此在幾微秒內跑完、完全不碰線路；之後由 pump 每個 tick 呼叫 drain() 送**一幀**，
  // 跟它平常的 PING／收包／遙測一起做。全程不阻塞。
  // **負載數學（重點）**：遙測發布週期 25ms（實測約 33Hz），所以靜止時線上是「每約 30ms
  // 一個 350B 的幀」：**平均**約 11KB/s、**峰值**任一 10ms 窗約 350B。輪播每 20ms 多一幀約 70B：
  //   平均  約 11 → 約 15 KB/s（1.3 倍）
  //   峰值  實測 是靜止峰值的 1.11 倍（模型保守估 1.20 倍，即定義幀剛好跟遙測幀擠同一個窗）
  // 對照 v1：32KB/s 持續 250ms，平均約 **2.8 倍**而且全集中在一波。目錄約 **132 幀**，
  // 所以輪播約 **2.6 秒**，加上 5 秒間隔整個循環約 **7.6 秒**。這個延遲就是 v2 刻意做的
  // 取捨：用目錄慢一點換一條平坦的線路。**峰值對峰值、平均對平均，兩組數字不要混用。**
  //
  // The queue is sized for the whole catalogue with headroom. If it ever fills
  // anyway, capture switches OFF for the remainder so the rest of the
  // registration goes straight out rather than being silently dropped -- a
  // truncated registry is the exact failure all of this exists to prevent.
  // 中文：佇列容量按整份目錄加餘裕抓。萬一真的滿了，剩下的部分會**關掉錄製直接送出**，
  // 而不是安靜丟掉——註冊被截斷正是這整套機制要防的事。
  static constexpr std::size_t kRegQueueBytes = 12288;
  // 256, not 192: the FRAME TABLE is the binding limit, not the byte pool.
  // With both registries at their 96 cap the catalogue is ~200 frames
  // (96 channels + 96 configs + commands + HELLO + device map) against ~12 KB
  // of payload, so the byte pool has room to spare while 192 slots would not.
  // 中文：256 而不是 192——卡住的是**幀數表**，不是位元組池。兩張登記表都到
  // 96 上限時，目錄約 200 幀（96 频道＋96 參數＋命令＋HELLO＋埠地圖）、但負載只有
  // 約 12KB，所以位元組池還有餘裕而 192 格不夠。
  static constexpr std::size_t kRegQueueFrames = 256;

  // Start/stop buffering whole frames instead of sending them.
  void capture_begin();
  void capture_end();
  // True while captured frames are still waiting to be played out.
  bool playback_pending() const { return frame_next_ < frame_count_; }
  // Send up to `max_frames` queued frames. Returns true if more remain.
  bool playback_step(std::size_t max_frames);
  // True if a capture ever hit the queue limit and fell back to sending the
  // remainder directly. NOT a count of dropped frames -- nothing is ever
  // dropped; the fallback sends the tail unbuffered, which is slower on the
  // wire but complete. A count would also be meaningless here, since the same
  // branch turns capturing off and so can only ever be taken once per capture.
  // 中文：錄製曾經撞到佇列上限、改成直接送出尾巴時為 true。**不是被丟弃的幀數**
  // ——一幀都不會丟，只是尾巴不經緩衝直接上線，線上比較擠但完整。計數也沒意義：
  // 同一個分支會把錄製關掉，所以每次錄製最多只走得到一次。
  bool capture_overflowed() const { return capture_overflowed_; }

 private:
  pros::Serial serial_;

  bool capturing_ = false;
  std::size_t queue_len_ = 0;      // bytes buffered
  std::size_t queue_read_ = 0;     // byte cursor for playback
  std::size_t frame_count_ = 0;    // frames buffered
  std::size_t frame_next_ = 0;     // frame cursor for playback
  bool capture_overflowed_ = false;
  std::uint8_t queue_[kRegQueueBytes];
  std::uint16_t frame_len_[kRegQueueFrames];
};

}  // namespace vexdash
