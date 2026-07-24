#include "vexdash_pros/connection_pump.h"

#include <cstdio>
#include <cstring>

// This file has no PROS headers (host-testable, see lib-pros/CLAUDE.md) and
// is also #included verbatim into the VEXcode build (lib-vexcode reuses
// ConnectionPump's platform-neutral logic as-is -- see lib-vexcode/CLAUDE.md
// and dashboard/scripts/pack-vexcode-download.ts, which ships this file at
// examples/vexcode-pid-tuning/src/vexdash_pros/connection_pump.cpp). Calls
// below use global (not std::) snprintf because VEXcode's newlib toolchain
// only declares it in the global namespace -- same reasoning as
// lib-core/src/session.cpp. <cstdio> makes the global name available under
// the host/PROS toolchains too, so this stays portable across all targets.

namespace vexdash {

ConnectionPump::ConnectionPump(Session& session, const PumpConfig& config,
                                RegisterCallback on_register, void* user_data)
    : session_(session), config_(config), on_register_(on_register), user_data_(user_data) {}

void ConnectionPump::copy_status_str(char* dst, const char* src) {
  if (src == nullptr) {
    dst[0] = '\0';
    return;
  }
  std::size_t i = 0;
  for (; i < kStatusStrLen - 1 && src[i] != '\0'; ++i) dst[i] = src[i];
  dst[i] = '\0';
}

void ConnectionPump::note_config_set(const char* param_name, const char* value_str) {
  // Compose "name=value" into the fixed buffer without allocation.
  char buf[kStatusStrLen];
  snprintf(buf, sizeof(buf), "%s=%s", param_name ? param_name : "?",
                value_str ? value_str : "?");
  copy_status_str(last_config_, buf);
}

void ConnectionPump::note_command(const char* command_name) {
  copy_status_str(last_command_, command_name);
}

void ConnectionPump::note_warning(const char* msg) { copy_status_str(last_warning_, msg); }

void ConnectionPump::do_register() {
  // protocol.md §6.2: (re)announce ourselves and replay the registry so a
  // freshly-opened dashboard sees channel/config/command/device metadata.
  session_.send_hello();
  if (on_register_ != nullptr) {
    on_register_(session_, user_data_);
  }
  ++stats_.registrations;
  // Count the registration burst as (at least) one TX frame (honest lower
  // bound; the burst is actually HELLO + many frames, see header note).
  ++tx_frames_;
}

void ConnectionPump::resend_registration() {
  // Periodic self-heal replay: re-run the caller's registration callback
  // (idempotent, see class comment point 6) WITHOUT send_hello(). Re-sending
  // HELLO here would make the dashboard clear-on-HELLO its already-populated
  // registry every period, causing visible flicker -- so this path only ever
  // replays on_register_().
  if (on_register_ != nullptr) {
    on_register_(session_, user_data_);
  }
  ++stats_.registrations;
  // Same honest-lower-bound accounting as do_register().
  ++tx_frames_;
}

Status ConnectionPump::status() const {
  Status s;
  s.link = link_state_;
  // uptime is computed from the last tick's now vs start; the pump doesn't
  // hold "now", so we derive elapsed from the most recent timestamp we saw.
  // start_ms_ is set on the first tick; the latest now_ms observed is
  // max(last_ping_ms_, last_telemetry_ms_, last_pong_seen_ms_).
  std::uint32_t latest = last_ping_ms_;
  if (last_telemetry_ms_ > latest) latest = last_telemetry_ms_;
  if (last_pong_seen_ms_ > latest) latest = last_pong_seen_ms_;
  s.uptime_ms = started_ ? (latest - start_ms_) : 0u;

  s.tx_frames = tx_frames_;
  s.rx_frames = session_.decoder_stats().frames_ok;
  s.tx_fps = tx_fps_;
  s.reconnects = stats_.reconnects;
  copy_status_str(s.last_config, last_config_);
  copy_status_str(s.last_command, last_command_);
  copy_status_str(s.last_warning, last_warning_);
  return s;
}

void ConnectionPump::tick(std::uint32_t now_ms) {
  // 1. First-tick startup: send HELLO + registration immediately so the
  //    robot streams even if no dashboard is listening yet (§6.2).
  if (!started_) {
    started_ = true;
    start_ms_ = now_ms;
    last_telemetry_ms_ = now_ms;
    last_ping_ms_ = now_ms;
    last_pong_seen_ms_ = now_ms;  // grace period before first timeout
    tx_rate_window_start_ms_ = now_ms;
    tx_rate_window_base_ = tx_frames_;
    // Baseline for the periodic resend timer even if register_on_start is
    // false (no registration sent yet) -- the timer counts from "now" either
    // way so it doesn't fire early once the link comes up.
    last_registration_ms_ = now_ms;
    // Baseline the device-scan cadence too, so the first hotplug poll fires one
    // full period from now -- the initial DEVICE_MAP snapshot is already sent by
    // do_register() (the register callback emits it), so the poll only needs to
    // catch subsequent plug/unplug. 中文：掃埠計時從現在起算；初始快照由註冊路徑
    // 送出，週期掃描只負責之後的插拔。
    last_device_scan_ms_ = now_ms;
    if (config_.register_on_start) {
      do_register();
    }
    link_state_ = LinkState::kDown;  // registered but not yet confirmed live
  }

  // 2. Drain incoming bytes; this dispatches CONFIG_SET/COMMAND and updates
  //    session_.last_pong() if a PONG arrived.
  session_.poll();

  // 3. Detect a newly-arrived PONG. Session overwrites last_pong() on each
  //    PONG but never clears `received`, so we detect "new" by a change in
  //    the echoed original_sender_time_ms (which the pump varies every PING
  //    via now_ms) OR the first time we ever see received==true.
  const PongInfo& pong = session_.last_pong();
  if (pong.received) {
    bool is_new_pong = !have_seen_any_pong_ ||
                       pong.original_sender_time_ms != last_pong_original_time_;
    if (is_new_pong) {
      have_seen_any_pong_ = true;
      last_pong_original_time_ = pong.original_sender_time_ms;
      last_pong_seen_ms_ = now_ms;

      // DOWN->UP edge: link just (re)established.
      if (link_state_ != LinkState::kUp) {
        if (ever_up_) {
          // This is a reconnect: replay the registry so the (possibly new)
          // dashboard instance has the full metadata again.
          ++stats_.reconnects;
          do_register();
          last_registration_ms_ = now_ms;
        }
        ever_up_ = true;
        link_state_ = LinkState::kUp;
      }
    }
  }

  // 4. Link timeout: no fresh PONG within link_timeout_ms -> DOWN.
  if (link_state_ == LinkState::kUp &&
      (now_ms - last_pong_seen_ms_) >= config_.link_timeout_ms) {
    link_state_ = LinkState::kDown;
  }

  // 5. Heartbeat PING (also the liveness probe that drives PONG detection).
  if ((now_ms - last_ping_ms_) >= config_.ping_period_ms) {
    session_.ping(now_ms);
    last_ping_ms_ = now_ms;
    ++stats_.pings_sent;
    ++tx_frames_;
  }

  // 6. Rate-limited telemetry flush. The caller has been put()ing samples
  //    into session_.telemetry() since the last tick; coalesce to cadence.
  if ((now_ms - last_telemetry_ms_) >= config_.telemetry_period_ms) {
    // 方案 A：flush 前讓 watch 登記表取樣（若門面掛了鉤）。nullptr 時這一段完全
    // 不執行，等同舊行為。放在 flush() 之前，確保本 frame 就帶著最新的 watch 值。
    if (config_.pre_flush != nullptr) {
      config_.pre_flush(session_, config_.pre_flush_user_data);
    }
    session_.telemetry().flush(now_ms);
    last_telemetry_ms_ = now_ms;
    ++stats_.telemetry_flushes;
    // A flush emits >=1 TELEMETRY frame (more if samples spill past one
    // frame, protocol.md §5.2); count 1 as an honest lower bound.
    ++tx_frames_;
  }

  // 7. Periodic registration resend (self-heal for lossy bridges, see class
  //    comment point 6). Only while UP -- no point replaying defs nobody can
  //    hear -- and never sends HELLO (would flicker the dashboard's
  //    already-populated registry). Disabled when the period is 0.
  if (link_state_ == LinkState::kUp && config_.registration_resend_period_ms > 0 &&
      (now_ms - last_registration_ms_) >= config_.registration_resend_period_ms) {
    resend_registration();
    last_registration_ms_ = now_ms;
  }

  // 7b. Periodic device-map hotplug poll. The callback scans the smart ports
  //     and re-sends DEVICE_MAP (0x0D) ONLY when the plugged-in set changed
  //     (change-on-event, protocol.md §5.12) -- so a mid-session plug/unplug
  //     shows on the dashboard within ~device_scan_period_ms without washing
  //     the wire every tick. Runs regardless of link state (like telemetry, the
  //     robot may stream before a dashboard connects, §6.2). The periodic
  //     registration resend (step 7) additionally re-sends the full snapshot
  //     unconditionally, which is DEVICE_MAP's self-heal against a mid-burst
  //     frame drop -- exactly how CHANNEL_DEF avoids the "link-up sends once,
  //     then a dropped frame is lost forever" bug. Disabled when the callback
  //     is null or the period is 0.
  //     中文：慢週期掃埠，只在插拔變化時重送 DEVICE_MAP（不是每 tick 洗）；掉幀
  //     的自癒交給步驟 7 的週期重送（無條件整批重送），跟 CHANNEL_DEF 同款解法。
  if (config_.device_scan != nullptr && config_.device_scan_period_ms > 0 &&
      (now_ms - last_device_scan_ms_) >= config_.device_scan_period_ms) {
    config_.device_scan(session_, config_.device_scan_user_data);
    last_device_scan_ms_ = now_ms;
  }

  // 8. Recompute the TX frame rate over a ~1s sliding window.
  std::uint32_t win = now_ms - tx_rate_window_start_ms_;
  if (win >= 1000) {
    std::uint32_t delta = tx_frames_ - tx_rate_window_base_;
    tx_fps_ = (delta * 1000u) / win;
    tx_rate_window_start_ms_ = now_ms;
    tx_rate_window_base_ = tx_frames_;
  }
}

namespace {
const char* link_text(LinkState s) {
  switch (s) {
    case LinkState::kInitializing:
      return "INIT";
    case LinkState::kDown:
      return "DOWN";
    case LinkState::kUp:
      return "UP";
  }
  return "?";
}
}  // namespace

int format_status_lines(const Status& status, char* lines, int max_lines, int line_cap) {
  if (lines == nullptr || max_lines <= 0 || line_cap <= 1) return 0;

  auto row = [&](int i) -> char* { return lines + static_cast<std::size_t>(i) * line_cap; };

  int n = 0;
  const int cap = line_cap;  // includes room for the NUL

  // Line 0: title + link state (the most important glanceable info).
  if (n < max_lines) {
    snprintf(row(n), cap, "vexdash  LINK:%s", link_text(status.link));
    ++n;
  }
  // Line 1: uptime + reconnects.
  if (n < max_lines) {
    std::uint32_t secs = status.uptime_ms / 1000u;
    snprintf(row(n), cap, "up %lus  reconn:%lu", static_cast<unsigned long>(secs),
                  static_cast<unsigned long>(status.reconnects));
    ++n;
  }
  // Line 2: TX/RX frame counters.
  if (n < max_lines) {
    snprintf(row(n), cap, "TX:%lu  RX:%lu", static_cast<unsigned long>(status.tx_frames),
                  static_cast<unsigned long>(status.rx_frames));
    ++n;
  }
  // Line 3: TX rate.
  if (n < max_lines) {
    snprintf(row(n), cap, "TX rate:%lu fps", static_cast<unsigned long>(status.tx_fps));
    ++n;
  }
  // Line 4: last CONFIG_SET.
  if (n < max_lines) {
    snprintf(row(n), cap, "cfg:%s", status.last_config[0] ? status.last_config : "-");
    ++n;
  }
  // Line 5: last COMMAND.
  if (n < max_lines) {
    snprintf(row(n), cap, "cmd:%s", status.last_command[0] ? status.last_command : "-");
    ++n;
  }
  // Line 6: last warning/error.
  if (n < max_lines) {
    snprintf(row(n), cap, "warn:%s", status.last_warning[0] ? status.last_warning : "-");
    ++n;
  }
  return n;
}

}  // namespace vexdash
