#pragma once

#include <cstdint>

#include "vexdash/protocol_types.h"
#include "vexdash/transport.h"

// Telemetry: CHANNEL_DEF (registration) + TELEMETRY (samples), per
// protocol.md §5.2/§5.3, plus v1.1 TELEMETRY_TEXT (§5.11) and the v1.1
// CHANNEL_DEF extensions (device_port, enum labels, STRING/ENUM types).
//
// Usage pattern (v1):
//   Telemetry telemetry(transport);
//   auto ch = telemetry.declare_channel("left_motor_temp", ValueType::kF64, "C");
//   telemetry.put(ch, 42.5);   // buffers one sample for the current frame
//   telemetry.flush(millis()); // encodes+sends one TELEMETRY frame with
//                               // all samples put() since the last flush
//
// Usage pattern (v1.1 device-aware / text / enum):
//   ChannelOptions opt;                        // opt-in v1.1/v1.4 extensions
//   opt.device_port = 3;                       // attribute to smart port 3
//   opt.path = "drive/pid";                    // v1.4: declare the mechanism
//   auto temp = telemetry.declare_channel_ex("temp", ValueType::kF64, "C", opt);
//
//   const char* labels[] = {"none", "motor", "rotation"};
//   auto state = telemetry.declare_enum_channel("port3_state", labels, 3, /*device_port=*/3);
//   telemetry.put_enum(state, 1);              // enum sample travels as i32
//
//   auto step = telemetry.declare_channel_ex("auton_step", ValueType::kString);
//   telemetry.put_text(step, "scoring preload");  // sends a TELEMETRY_TEXT frame
//
// No dynamic allocation: channel table and per-flush sample buffer are
// fixed-size arrays sized by compile-time constants below. declare_* keep
// no pointers to the caller's name/unit/label strings past the call (the
// CHANNEL_DEF wire bytes are the only persisted form).

namespace vexdash {

using ChannelId = std::uint16_t;

// protocol.md §2.2: a single TELEMETRY frame holds at most ~45 f64
// samples; kMaxSamplesPerFrame is a conservative cap under that bound that
// also serves as the fixed buffer size for put()-before-flush().
constexpr std::size_t kMaxSamplesPerFrame = 40;

// Fixed table size for registered channels (declare_channel() fails past
// this -- V5 has no dynamic allocation to grow it).
constexpr std::size_t kMaxChannels = 64;

// Opt-in CHANNEL_DEF extensions (protocol.md §5.3). Defaults reproduce
// a plain v1 CHANNEL_DEF (no v11_flags byte emitted at all) so existing
// callers are byte-for-byte unaffected.
struct ChannelOptions {
  // Smart port this channel belongs to (1-21 smart, 22-29 ADI A-H per
  // §5.12), or 0 for "no attribution". A negative sentinel (-1) means
  // "do not emit device_port at all" (keeps the CHANNEL_DEF v1-shaped
  // unless enum labels are present).
  int device_port = -1;

  // v1.4 (protocol.md §5.3/§6.7): the mechanism/group this channel belongs
  // to, e.g. "drive/pid". Shares one namespace with CONFIG_SCHEMA's `path`
  // (§5.6) -- the same string means the same group, so the dashboard shows
  // this channel's graph next to that group's tunable parameters. nullptr
  // or "" = not declared: no path field is emitted at all and the dashboard
  // falls back to its name-based grouping heuristic (exactly the pre-v1.4
  // behavior). Max kMaxPathLen bytes.
  // 中文：這條頻道屬於哪個機構／群組（如 "drive/pid"），與可調參數的 path 同一
  // 個命名空間。不填＝不宣告，wire 上一個 byte 都不多、前端退回名字啟發式。
  const char* path = nullptr;

  bool has_device_port() const { return device_port >= 0; }
  bool has_path() const { return path != nullptr && path[0] != '\0'; }
};

class Telemetry {
 public:
  explicit Telemetry(ITransport& transport);

  // v1 CHANNEL_DEF: registers a channel and immediately sends its
  // CHANNEL_DEF frame with NO v1.1 extension bytes (byte-identical to the
  // original v1 encoding). Returns the assigned ChannelId, or an
  // implementation-defined sentinel on failure (table full, name/unit too
  // long) -- check via is_valid_channel().
  ChannelId declare_channel(const char* name, ValueType value_type, const char* unit = "");

  // CHANNEL_DEF with optional device_port (v1.1) / path (v1.4) attribution.
  // If neither opt.has_device_port() nor opt.has_path() is set, emits a
  // plain v1-shaped CHANNEL_DEF (no v11_flags), identical to
  // declare_channel(); otherwise emits the v11_flags byte followed by the
  // set fields in bit order. value_type may be any ValueType incl.
  // kString/kEnum (for kEnum without labels, front-end shows raw indices).
  // Fails (kInvalidChannelId) if opt.path is longer than kMaxPathLen.
  ChannelId declare_channel_ex(const char* name, ValueType value_type, const char* unit = "",
                               const ChannelOptions& opt = ChannelOptions{});

  // v1.1 ENUM channel: registers a kEnum channel carrying enum_labels
  // (protocol.md §5.3). label index (declaration order) maps to the
  // displayed string. `device_port` < 0 omits the device_port field.
  // `path` (v1.4) is the mechanism/group path, nullptr/"" = not declared
  // (see ChannelOptions::path); it is the LAST parameter so every existing
  // call site keeps compiling unchanged.
  // Fails (kInvalidChannelId) if label_count > kMaxEnumLabels, any label
  // > kMaxEnumLabelLen bytes, or path > kMaxPathLen bytes.
  ChannelId declare_enum_channel(const char* name, const char* const* labels, std::size_t label_count,
                                 int device_port = -1, const char* unit = "",
                                 const char* path = nullptr);

  static bool is_valid_channel(ChannelId id) { return id != kInvalidChannelId; }

  // Buffers one numeric/bool sample for `channel` for the next flush().
  // If put() is called past the per-flush buffer cap (kMaxSamplesPerFrame)
  // before a flush(), the overflow sample is dropped -- callers driving a
  // fixed-rate telemetry loop should flush() at least as often as they
  // put() distinct channels to avoid silent drops.
  void put(ChannelId channel, double value);
  void put(ChannelId channel, std::int32_t value);
  void put(ChannelId channel, bool value);

  // v1.1: buffers an ENUM channel's current index as an i32 sample
  // (protocol.md §5.3: enum values travel as i32 on the wire). Convenience
  // wrapper over put(channel, (int32_t)index).
  void put_enum(ChannelId channel, std::int32_t index) { put(channel, index); }

  // Encodes and sends one TELEMETRY frame with all samples buffered since
  // the last flush(), tagged with `timestamp_ms`, then clears the buffer.
  // If more samples are pending than fit in one frame
  // (kMaxSamplesPerFrame), sends multiple TELEMETRY frames sharing the
  // same timestamp_ms (protocol.md §5.2 explicitly allows this).
  // Returns false if any underlying frame_encode/transport write fails.
  bool flush(std::uint64_t timestamp_ms);

  // v1.1: sends one TELEMETRY_TEXT frame (protocol.md §5.11) carrying a
  // single string value for a STRING channel. Change-on-write / low-freq
  // path (not batched, not the high-rate TELEMETRY path). Truncates text
  // to the frame budget if needed; returns false only on encode/transport
  // failure, not on truncation.
  bool put_text(ChannelId channel, const char* text);

  std::size_t channel_count() const { return channel_count_; }

 private:
  static constexpr ChannelId kInvalidChannelId = 0xFFFF;

  struct ChannelInfo {
    ValueType value_type;
    // Fixed-size name copy so re-declaring the same name is idempotent
    // (reuses the id + re-sends the def on reconnect) rather than allocating
    // a new id each time. No dynamic allocation.
    char name[kMaxNameLen + 1];
  };

  // Returns the id of an already-declared channel with this name, or
  // kInvalidChannelId if none. Fixed-size linear scan (kMaxChannels).
  ChannelId find_channel_by_name(const char* name) const;

  struct PendingSample {
    ChannelId channel;
    ValueType value_type;
    std::uint8_t bytes[8];  // largest value_type (f64) is 8 bytes
  };

  // Builds & sends a CHANNEL_DEF frame. `labels`==nullptr / label_count==0
  // means "no enum_labels field". device_port < 0 means "no device_port
  // field". `path`==nullptr/"" means "no path field" (v1.4). No v11_flags
  // byte at all when none of the three is present.
  bool send_channel_def(ChannelId id, const char* name, ValueType value_type, const char* unit,
                        int device_port, const char* const* labels, std::size_t label_count,
                        const char* path);

  ITransport& transport_;

  ChannelInfo channels_[kMaxChannels];
  std::size_t channel_count_ = 0;

  PendingSample pending_[kMaxSamplesPerFrame];
  std::size_t pending_count_ = 0;

  // protocol.md §5.2 (v1.2): per-frame monotonic sequence number appended as a
  // trailing u16 to every TELEMETRY frame. Increments once per emitted frame
  // (so a multi-frame flush yields consecutive seqs) and wraps at 0xFFFF. The
  // dashboard uses gaps in this to compute a real packet-loss rate instead of
  // guessing from arrival intervals.
  std::uint16_t tx_seq_ = 0;
};

}  // namespace vexdash
