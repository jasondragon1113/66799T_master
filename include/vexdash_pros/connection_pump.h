#pragma once

#include <cstdint>

#include "vexdash/session.h"
#include "vexdash/transport.h"

// ConnectionPump: the platform-INDEPENDENT pump/heartbeat/reconnect/
// rate-limit logic for a robot-side vexdash connection. Deliberately
// contains NO PROS headers so it can be unit-tested on the host with a
// fake ITransport (see lib-pros/tests/). The PROS layer (ProsTask) is a
// thin wrapper that calls tick() from a pros::Task on a fixed period.
//
// Responsibilities, all driven by tick(now_ms):
//   1. poll(): drain incoming bytes, dispatch CONFIG_SET / COMMAND / PONG.
//   2. Rate-limited telemetry flush: flush the caller's buffered samples at
//      most once per `telemetry_period_ms` (the caller put()s samples into
//      session.telemetry() between ticks).
//   3. Heartbeat PING at `ping_period_ms` (doubles as liveness probe).
//   4. Disconnect detection: if no PONG has been seen for
//      `link_timeout_ms`, the link is considered DOWN.
//   5. Reconnect / re-registration: on the DOWN->UP edge (first PONG after
//      a down period, or the very first PONG), invoke the caller's
//      `on_register` callback so it re-sends its CHANNEL_DEF / CONFIG_SCHEMA
//      / CMD_DEF / DEVICE_MAP registrations (protocol.md: the dashboard may
//      have (re)opened and needs the registry replayed).
//   6. Periodic registration resend (self-heal): while the link is UP, every
//      `registration_resend_period_ms` the pump re-invokes `on_register`
//      again WITHOUT re-sending HELLO. This is for lossy transports (e.g. an
//      ESP32 WiFi bridge) that can drop a frame in the middle of the
//      link-up/reconnect registration burst -- without a resend, that def
//      would be lost forever and the dashboard would show a fallback name
//      (channel_N) for that channel/config/command. Re-running the caller's
//      `on_register` is safe because it is expected to be idempotent
//      (declare_* re-declares reuse the existing id and callback, see
//      lib-core's name-based de-dup). HELLO is deliberately NOT re-sent here:
//      the dashboard clears and rebuilds its registry when it sees a HELLO
//      from an already-known robot, so periodic HELLOs would cause visible
//      flicker. Set `registration_resend_period_ms = 0` to disable.
//
// The pump does NOT own the transport or session lifetime; the caller
// constructs a Session over a transport and hands a reference in. No
// dynamic allocation.

namespace vexdash {

// Callback the pump invokes to (re)send all registration-type frames
// (CHANNEL_DEF/CONFIG_SCHEMA/CMD_DEF/DEVICE_MAP). Called once at first
// successful link-up and again after every reconnect. `user_data` is the
// context pointer supplied to the pump.
using RegisterCallback = void (*)(Session& session, void* user_data);

// Callback the pump invokes right BEFORE each telemetry flush (方案 A：watch
// 自動上報). The watch-registry façade installs this so every registered
// variable pointer is sampled into session.telemetry() just before the frame
// goes out -- meaning the user never writes a per-loop put(). Default nullptr
// keeps the pump's original behaviour (it only ships what the caller manually
// put()), so this is fully backward compatible and host-testable.
using SampleCallback = void (*)(Session& session, void* user_data);

// Callback the pump invokes on a slow cadence (device_scan_period_ms) to poll
// the smart ports for hotplug and re-send DEVICE_MAP (0x0D) only when the
// plugged-in set changed. The façade installs one that drives DeviceScanner;
// default nullptr = no auto device scan (backward compatible, host-testable).
// 中文：週期性掃埠鉤子。pump 每 device_scan_period_ms 呼叫一次，讓門面掃 21 個
// 智慧埠、偵測到插拔才重送 DEVICE_MAP——不是每 tick 洗訊息。預設 nullptr＝不啟用。
using DeviceScanCallback = void (*)(Session& session, void* user_data);

struct PumpConfig {
  // Minimum interval between telemetry flushes (ms). The caller's telemetry
  // put() rate can be higher; the pump coalesces to this cadence.
  std::uint32_t telemetry_period_ms = 20;  // 50 Hz default (protocol.md §7)

  // Heartbeat PING interval (ms). protocol.md §5.8 suggests ~1 Hz.
  std::uint32_t ping_period_ms = 1000;

  // If no PONG is received within this window, the link is considered down.
  // Must be a small multiple of ping_period_ms so a couple of dropped
  // heartbeats don't false-trip. protocol.md leaves this to the impl.
  std::uint32_t link_timeout_ms = 3000;

  // If true, the pump sends an initial HELLO + registration immediately on
  // the first tick without waiting for a PONG (Robot should stream even if
  // no dashboard is listening yet, protocol.md §6.2). The link is still
  // reported DOWN until the first PONG arrives.
  bool register_on_start = true;

  // While the link is UP, resend the registration burst (on_register only --
  // no HELLO, see the class comment / tick() point 6) every this many ms.
  // Self-heals CHANNEL_DEF/CONFIG_SCHEMA/CMD_DEF/DEVICE_MAP frames dropped
  // mid-burst by a lossy bridge (e.g. ESP32 WiFi): without this, a dropped
  // def is lost for the rest of the session and the dashboard falls back to
  // a generic name (channel_N) for that item. Set to 0 to disable.
  // Was 2000ms; raised to 5000ms (2026-07-26, ticket V-15) -- the resend burst
  // is ~843B sent all at once and was the single biggest periodic spike on the
  // RS-485/WiFi link every 2s. 5s is still short enough to self-heal a dropped
  // CHANNEL_DEF well within a normal debugging/pairing session, so the
  // self-heal guarantee holds while cutting the burst frequency by 2.5x.
  // 中文：原本 2000ms，V-15 精算後改 5000ms——重送整批約 843B 是線路上唯一的
  // 週期性壓力尖峰，每 2 秒炸一次；拉到 5 秒仍能在合理時間內自癒掉幀的
  // CHANNEL_DEF（沿用 tick() 第 6 點的自癒機制），但把突發頻率降到 2.5 分之一。
  std::uint32_t registration_resend_period_ms = 5000;

  // ---- registration playback (trickle v2) -----------------------------
  //
  // WHAT THIS REPLACED, and why the first fix was not enough. Measured with the
  // robot stationary, dashboard link quality out of 100:
  //   * no pacing         baseline ~65, dropping to ~24 every 5 s
  //   * blocking pacer    baseline 77-82, still dropping to 22-45 every 5 s
  // The resend replays the whole registry, ~90-120 frames / ~8 KB. Sent flat
  // out that is ~92 KB/s; throttled to 320 B / 10 ms it is a 0.25 s sprint at
  // 32 KB/s. The ESP32's RX buffer has roughly 8 ms of slack at that rate, so a
  // single millisecond-scale WiFi stall inside the sprint still overruns it --
  // which is what the remaining dips were. Slowing the sprint further was not
  // available either: the pacer BLOCKED the pump, so a longer burst meant no
  // PINGs for that whole time and link_timeout_ms (3000) started to loom.
  //
  // v2 removes the burst rather than resizing it. The transport captures the
  // registration frames instead of sending them (smartport_transport.h), and
  // the pump plays them back ONE FRAME at a time from its ordinary tick, in
  // between the PING, the RX poll and the telemetry flush. Nothing blocks and
  // there is no window to be unlucky inside.
  //
  // THE NUMBERS THAT MATTER, and keep peak and average apart. Telemetry
  // publishes every 25 ms (~33 Hz measured): at rest that is ~11 KB/s AVERAGE
  // with a ~350 B PEAK in any single 10 ms window. Playback adds one ~70 B
  // frame per 20 ms -> ~15 KB/s average (1.3x) and a measured 1.11x peak
  // (model bound 1.20x, the case where a definition frame lands in the same
  // window as a telemetry frame). v1 by comparison was 32 KB/s for 250 ms,
  // ~2.8x the average and all of it in one burst. The catalogue is ~132 frames
  // so playback runs ~2.6 s, and the full cycle (playback + the 5 s idle
  // period) is ~7.6 s.
  //
  // 中文：**取代了什麼、以及第一版為什麼不夠**（車靜止實測，品質分滿分 100）：
  //   完全不節流：基線約 65，每 5 秒掉到約 24；阻塞式節流：基線 77-82，**每 5 秒仍掉到
  //   22-45**。重送要打完整份登記表（約 90-120 幀 / 8KB）：不節流是瞬時約 92KB/s，
  //   節流成 320B/10ms 是「0.25 秒、32KB/s 的衝刺」。ESP32 的 RX 緩衝在那個速率下只剩
  //   約 8ms 餘裕，WiFi 在衝刺窗口裡卡頓一毫秒就又爆——剩下的那些掉分就是這樣來的。
  //   而把衝刺再放慢也不行：舊的節流是**阻塞**的，爆發拉長等於那段時間完全不送 PING，
  //   逼近 link_timeout_ms（3000）。
  // **v2 不是把衝刺調小，是把衝刺拿掉**：transport 先把註冊的幀錄下來而不送出
  // （見 smartport_transport.h），再由 pump 在自己平常的 tick 裡**一次播一幀**，
  // 夾在 PING、收包、遙測之間。全程不阻塞，也就沒有「窗口」可以倒楣。
  // **關鍵數字**：穩態每 10ms 約 140B；每 20ms 多一幀約 70B ＝每 tick 多約 35B，
  // 任何 10ms 窗都不超過穩態的約 1.25 倍。整份目錄約 90 幀 × 20ms ≒ 1.8 秒播完——
  // 這個延遲就是刻意的取捨：讓 dashboard 的清單花一兩秒長出來，換線路不被捶。

  // Sends up to `max_frames` buffered definition frames. Returns true if more
  // remain. nullptr = no playback (frames go out as they are declared, the
  // pre-v2 behaviour, which is still correct for USB).
  // 中文：送出最多 max_frames 個已錄好的定義幀，還有剩就回 true。nullptr＝不做輪播
  // （定義幀在宣告當下就送出，也就是 v2 之前的行為——USB 路徑維持這樣是對的）。
  using RegistrationDrainFn = bool (*)(void* user_data, std::size_t max_frames);
  RegistrationDrainFn registration_drain = nullptr;
  void* registration_drain_user_data = nullptr;

  // One definition frame every this many ms, and how many to send each time.
  // 20 ms / 1 frame is the ~1.25x figure worked out above; raising the frame
  // count is what you change if a much larger registry ever needs to fit in
  // the same wall-clock, and it is a direct multiplier on the load.
  // 中文：每幾毫秒播一次、一次播幾幀。20ms／1 幀就是上面算出來的約 1.25 倍；
  // 之後若登記表大很多、要在同樣的時間內播完，就是調幀數——它直接等比放大負載。
  std::uint32_t registration_drain_period_ms = 20;
  std::uint32_t registration_drain_frames = 1;

  // 方案 A（watch 自動上報）鉤子。非 nullptr 時，pump 在每次 telemetry flush 前
  // 先呼叫它一次，讓 watch 登記表把所有登記變數取樣進 telemetry()。預設 nullptr
  // ＝維持舊行為（pump 只送呼叫端手動 put() 的樣本），既有呼叫者不受影響。放在
  // flush 前呼叫，確保同一 frame 就帶著最新的 watch 值。host 可測。
  SampleCallback pre_flush = nullptr;
  void* pre_flush_user_data = nullptr;

  // 熱插拔掃描鉤子（DEVICE_MAP 自動化）。非 nullptr 時，pump 每 device_scan_period_ms
  // 呼叫它一次；該回呼負責掃埠並「只在內容變更時」重送 DEVICE_MAP。DEVICE_MAP 不佔
  // watch 遙測槽（§5.12），與 64 槽上限無關。獨立於 pre_flush 之外用「慢週期」是因為
  // 掃 21 個埠不需要跟著 50Hz 遙測跑；插拔本來就是低頻事件。預設 nullptr＝維持舊行為
  // （不掃、不自動送），既有呼叫者完全不受影響，host 可測。
  DeviceScanCallback device_scan = nullptr;
  void* device_scan_user_data = nullptr;

  // 掃描週期（ms）。~1 秒級：插拔是人手操作的低頻事件，1s 內看到埠亮起夠即時，又
  // 不會拿掃 21 個埠的成本去洗高頻路徑。0＝停用週期掃描（初始快照仍由註冊路徑送出）。
  std::uint32_t device_scan_period_ms = 1000;
};

enum class LinkState : std::uint8_t {
  kInitializing,  // before the first registration has been sent
  kDown,          // registered, but no live dashboard confirmed (no recent PONG)
  kUp,            // a PONG was seen within link_timeout_ms
};

// Fixed-size string capacity for the human-readable status fields (no
// dynamic allocation anywhere in the pump / status snapshot).
constexpr std::size_t kStatusStrLen = 48;

// A snapshot of the connection state for on-Brain display / logging. Built
// by ConnectionPump; formatted to text by format_status_lines() (both are
// PROS-free and host-tested). All strings are NUL-terminated, fixed size.
struct Status {
  LinkState link = LinkState::kInitializing;
  std::uint32_t uptime_ms = 0;

  std::uint32_t tx_frames = 0;   // frames the pump has emitted (see note below)
  std::uint32_t rx_frames = 0;   // valid frames decoded from the dashboard
  std::uint32_t tx_fps = 0;      // recent TX frame rate (frames/second)

  std::uint32_t reconnects = 0;

  // Last CONFIG_SET applied, as reported by the user's config callback via
  // ConnectionPump::note_config_set(). Empty until the first one.
  char last_config[kStatusStrLen] = {0};
  // Last COMMAND triggered, as reported via note_command(). Empty initially.
  char last_command[kStatusStrLen] = {0};
  // Last warning/error string, as reported via note_warning(). Empty init.
  char last_warning[kStatusStrLen] = {0};
};

// Pure formatting: renders `status` into up to `max_lines` fixed-width text
// lines (each up to line_cap-1 chars + NUL). Returns the number of lines
// written. No allocation, no PROS. Host-tested. Lines are intended for a
// 480x272 Brain screen (LLEMU: 8 lines) or pros::screen text rows.
//
// `lines` is a caller-owned 2D buffer: lines[i] must hold >= line_cap chars.
int format_status_lines(const Status& status, char* lines, int max_lines, int line_cap);

class ConnectionPump {
 public:
  ConnectionPump(Session& session, const PumpConfig& config, RegisterCallback on_register,
                 void* user_data);

  // Advances the pump. `now_ms` is a monotonic millisecond clock supplied
  // by the caller (on PROS this is pros::millis()); the pump does not read
  // any clock itself so it stays host-testable with a virtual clock.
  // Call this at least as often as the smallest configured period.
  void tick(std::uint32_t now_ms);

  LinkState link_state() const { return link_state_; }
  bool is_up() const { return link_state_ == LinkState::kUp; }

  // ---- Status snapshot (for the Brain HUD; see format_status_lines) ------
  // Returns the current status snapshot, rebuilt cheaply each call from the
  // pump's counters. Safe to call from a display task between ticks.
  Status status() const;

  // Report the last CONFIG_SET / COMMAND / warning into the status snapshot.
  // These are meant to be called from the user's Config/Command callbacks
  // (which the pump cannot see into) and from error paths. Strings are
  // copied into fixed buffers (truncated to kStatusStrLen-1). Since stdout
  // is the vexdash data channel and printf is unavailable (see checklist),
  // these are how you surface "what just happened" onto the Brain screen.
  void note_config_set(const char* param_name, const char* value_str);
  void note_command(const char* command_name);
  void note_warning(const char* msg);

  // Diagnostics (not wire-protocol; useful for a status LED / logging).
  struct Stats {
    std::uint32_t registrations = 0;   // how many times on_register fired
    std::uint32_t reconnects = 0;      // DOWN->UP edges after the first UP
    std::uint32_t telemetry_flushes = 0;
    std::uint32_t pings_sent = 0;
  };
  const Stats& stats() const { return stats_; }

 private:
  // Full registration burst: HELLO + on_register_(). Used at startup and on
  // DOWN->UP reconnect.
  void do_register();
  // Periodic self-heal resend: on_register_() only, no HELLO (see class
  // comment point 6 / PumpConfig::registration_resend_period_ms).
  void resend_registration();
  // Copies `src` into `dst` (size kStatusStrLen), NUL-terminated & truncated.
  static void copy_status_str(char* dst, const char* src);

  // Approx. TX frame accounting. The pump directly emits pings + telemetry
  // flushes and knows when a registration burst happened, but registration
  // frames go out through Session (HELLO + N CHANNEL_DEF/CONFIG_SCHEMA/
  // CMD_DEF/DEVICE_MAP) and the pump cannot count them individually without
  // touching lib-core. So tx_frames counts: pings + telemetry-flush frames +
  // 1 per registration burst (an honest lower bound; documented as such).

  Session& session_;
  PumpConfig config_;
  RegisterCallback on_register_;
  void* user_data_;

  LinkState link_state_ = LinkState::kInitializing;

  bool started_ = false;
  bool ever_up_ = false;

  std::uint32_t last_telemetry_ms_ = 0;
  std::uint32_t last_ping_ms_ = 0;
  // Tracks the PONG count seen so we can detect "a new PONG arrived since
  // last tick" without the pump needing its own clock -- combined with
  // last_pong_seen_ms_ for the timeout.
  bool last_pong_received_ = false;
  std::uint64_t last_pong_original_time_ = 0;
  std::uint32_t last_pong_seen_ms_ = 0;
  bool have_seen_any_pong_ = false;

  // Timestamp (now_ms) of the most recent registration event (startup,
  // reconnect replay, or periodic resend) -- baseline for the periodic
  // registration_resend_period_ms self-heal timer.
  std::uint32_t last_registration_ms_ = 0;
  // Playback in flight: the registry has been captured and is being sent one
  // frame at a time. The resend timer does not restart until this clears, so
  // two playbacks can never overlap however slow one gets.
  // 中文：輪播進行中——登記表已經錄好、正在一幀一幀送。重送計時要等它結束才重新起算，
  // 所以不管播多慢，兩輪都不可能重疊。
  bool registration_playing_ = false;
  std::uint32_t last_drain_ms_ = 0;

  // Timestamp (now_ms) of the most recent device-scan poll -- baseline for the
  // device_scan_period_ms cadence. 中文：上次掃埠的時間，週期掃描的計時基準。
  std::uint32_t last_device_scan_ms_ = 0;

  // Status / HUD tracking.
  std::uint32_t start_ms_ = 0;          // now_ms of the first tick (for uptime)
  std::uint32_t tx_frames_ = 0;         // see note above class' private section
  std::uint32_t tx_rate_window_start_ms_ = 0;  // sliding-window anchor
  std::uint32_t tx_rate_window_base_ = 0;      // tx_frames_ at window start
  std::uint32_t tx_fps_ = 0;            // last computed frames/second
  char last_config_[kStatusStrLen] = {0};
  char last_command_[kStatusStrLen] = {0};
  char last_warning_[kStatusStrLen] = {0};

  Stats stats_;
};

}  // namespace vexdash
