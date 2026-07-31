#pragma once

#include <cstdint>

#include "vexdash/protocol_types.h"
#include "vexdash/transport.h"

// Config: CONFIG_SCHEMA (declare a tunable parameter) + dispatch of
// incoming CONFIG_SET frames to per-parameter callbacks, per
// protocol.md §5.6/§5.7.
//
// Usage pattern:
//   Config config(transport);
//   ConfigId kp_id = config.declare<double>("kP", "drive/pid", 1.2);
//   ...
//   config.set_callback(kp_id, [](double new_value, void* ctx) { ... });
//   ...
//   // whenever a CONFIG_SET frame's payload has been decoded by Session:
//   config.dispatch_config_set(payload, payload_len);
//
// No dynamic allocation: parameter table is a fixed-size array
// (kMaxConfigParams).

namespace vexdash {

using ConfigId = std::uint16_t;

// RAISED 64 -> 96 (2026-07-31, ported from 66994V) -- see the matching note on
// kMaxChannels in telemetry.h. ConfigId is uint16_t; the cost is ~76 bytes of
// static RAM per extra slot (~2.4 KB for +32) and a longer registration burst,
// which bounded_retry_write() flow-controls per frame.
// 中文：上限 64 → 96，理由同 telemetry.h 的 kMaxChannels。ConfigId 是 uint16_t，
// 代價是每格約 76 bytes 靜態記憶體（多 32 格約 2.4KB）與註冊 burst 變長，而 burst
// 由 bounded_retry_write 逐幀流控。
constexpr std::size_t kMaxConfigParams = 96;

// C-style callback (see frame_codec.h FrameCallback rationale: no
// <functional>/heap-backed closures assumed on embedded targets).
// `value_bytes` points at the raw little-endian value bytes
// (value_type_size(value_type) long); caller reinterprets per the
// ValueType recorded at declare<T>() time (dispatch already validated
// value_type matches, see .cpp).
using ConfigSetCallback = void (*)(ConfigId id, const std::uint8_t* value_bytes, void* user_data);

class Config {
 public:
  explicit Config(ITransport& transport);

  // Declares a parameter and sends its CONFIG_SCHEMA frame immediately.
  // `path` groups params into a UI tree (protocol.md §5.6), "" = root.
  // Returns kInvalidConfigId on failure (table full, name/path too long).
  ConfigId declare_f64(const char* name, const char* path, double default_value);
  ConfigId declare_i32(const char* name, const char* path, std::int32_t default_value);
  ConfigId declare_bool(const char* name, const char* path, bool default_value);

  static bool is_valid_config(ConfigId id) { return id != kInvalidConfigId; }

  // Registers the callback invoked when a CONFIG_SET for `id` is
  // dispatched. Only one callback per id (last registration wins).
  void set_callback(ConfigId id, ConfigSetCallback callback, void* user_data);

  // Parses a CONFIG_SET payload (protocol.md §5.7: config_id(2) +
  // value_type(1) + value) and, if config_id is known and value_type
  // matches the declared type, invokes the registered callback.
  // Unknown config_id or mismatched value_type: silently ignored (per
  // protocol.md §5.7's documented v1 behavior) -- returns false in both
  // cases so callers/tests can distinguish "dispatched" from "ignored",
  // but this is a diagnostic return value, not a wire-format signal.
  bool dispatch_config_set(const std::uint8_t* payload, std::size_t payload_len);

  std::size_t config_count() const { return config_count_; }
  ValueType value_type_of(ConfigId id) const;

 private:
  static constexpr ConfigId kInvalidConfigId = 0xFFFF;

  struct ConfigInfo {
    ValueType value_type;
    ConfigSetCallback callback = nullptr;
    void* user_data = nullptr;
    // Fixed-size name copy so re-declaring the same name reuses the id
    // (idempotent registration on reconnect). No dynamic allocation.
    char name[kMaxNameLen + 1] = {0};
  };

  // Returns the id of an already-declared param with this name, or
  // kInvalidConfigId if none. Fixed-size linear scan (kMaxConfigParams).
  ConfigId find_config_by_name(const char* name) const;

  ConfigId declare_impl(const char* name, const char* path, ValueType value_type,
                        const std::uint8_t* default_value_bytes);

  ITransport& transport_;
  ConfigInfo configs_[kMaxConfigParams];
  std::size_t config_count_ = 0;
};

}  // namespace vexdash
