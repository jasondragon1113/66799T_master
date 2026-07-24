#include "vexdash/config.h"

#include <cstring>

#include "vexdash/byte_writer.h"
#include "vexdash/frame_codec.h"

namespace vexdash {

Config::Config(ITransport& transport) : transport_(transport) {}

ConfigId Config::find_config_by_name(const char* name) const {
  for (std::size_t i = 0; i < config_count_; ++i) {
    if (std::strcmp(configs_[i].name, name) == 0) {
      return static_cast<ConfigId>(i);
    }
  }
  return kInvalidConfigId;
}

ConfigId Config::declare_impl(const char* name, const char* path, ValueType value_type,
                               const std::uint8_t* default_value_bytes) {
  std::size_t name_len = std::strlen(name);
  std::size_t path_len = std::strlen(path);
  if (name_len > kMaxNameLen || path_len > kMaxNameLen) return kInvalidConfigId;

  std::size_t value_size = value_type_size(value_type);

  // Idempotent re-declaration: reuse an existing param's id and re-send its
  // CONFIG_SCHEMA (so a reconnecting dashboard gets the same id), rather
  // than allocating a new id. Preserves the registered callback (the user
  // re-registers it via set_callback after declare anyway, but keeping it
  // avoids a transient window where a CONFIG_SET would be dropped).
  ConfigId existing = find_config_by_name(name);
  bool reuse = existing != kInvalidConfigId;
  if (!reuse && config_count_ >= kMaxConfigParams) return kInvalidConfigId;

  ConfigId id = reuse ? existing : static_cast<ConfigId>(config_count_);

  std::uint8_t payload[2 + 1 + 1 + kMaxNameLen + 1 + kMaxNameLen + 8];
  ByteWriter w(payload, sizeof(payload));
  w.write_u16(id);
  w.write_u8(static_cast<std::uint8_t>(value_type));
  w.write_string_u8len(name, name_len);
  w.write_string_u8len(path, path_len);
  w.write_bytes(default_value_bytes, value_size);
  if (!w.ok()) return kInvalidConfigId;

  std::uint8_t wire[frame_max_wire_size(1 + sizeof(payload) + 2)];
  std::size_t wire_len = frame_encode(MsgType::kConfigSchema, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return kInvalidConfigId;
  if (transport_.write(wire, wire_len) != wire_len) return kInvalidConfigId;

  configs_[id].value_type = value_type;
  if (!reuse) {
    configs_[id].callback = nullptr;
    configs_[id].user_data = nullptr;
    std::strcpy(configs_[id].name, name);  // name_len already <= kMaxNameLen
    ++config_count_;
  }  // on reuse: keep existing callback/user_data/name
  return id;
}

ConfigId Config::declare_f64(const char* name, const char* path, double default_value) {
  std::uint8_t bytes[8];
  std::memcpy(bytes, &default_value, sizeof(default_value));
  return declare_impl(name, path, ValueType::kF64, bytes);
}

ConfigId Config::declare_i32(const char* name, const char* path, std::int32_t default_value) {
  std::uint8_t bytes[4];
  std::memcpy(bytes, &default_value, sizeof(default_value));
  return declare_impl(name, path, ValueType::kI32, bytes);
}

ConfigId Config::declare_bool(const char* name, const char* path, bool default_value) {
  std::uint8_t bytes[1] = {static_cast<std::uint8_t>(default_value ? 0x01 : 0x00)};
  return declare_impl(name, path, ValueType::kBool, bytes);
}

void Config::set_callback(ConfigId id, ConfigSetCallback callback, void* user_data) {
  if (id >= config_count_) return;
  configs_[id].callback = callback;
  configs_[id].user_data = user_data;
}

ValueType Config::value_type_of(ConfigId id) const {
  // Callers are expected to only call this for ids returned by declare_*;
  // out-of-range access here would be a programming error on the robot
  // side, not a wire-format concern, so we return a definite (if
  // arbitrary) value rather than adding an exception/abort dependency.
  if (id >= config_count_) return ValueType::kF64;
  return configs_[id].value_type;
}

bool Config::dispatch_config_set(const std::uint8_t* payload, std::size_t payload_len) {
  ByteReader r(payload, payload_len);
  std::uint16_t config_id;
  std::uint8_t value_type_raw;
  if (!r.read_u16(config_id)) return false;
  if (!r.read_u8(value_type_raw)) return false;

  if (config_id >= config_count_) {
    // protocol.md §5.7: unknown config_id -> silently ignore (v1 behavior).
    return false;
  }

  ConfigInfo& info = configs_[config_id];
  if (value_type_raw != static_cast<std::uint8_t>(info.value_type)) {
    // protocol.md §5.7: mismatched value_type -> silently ignore.
    return false;
  }

  const std::uint8_t* value_ptr;
  if (!r.read_value_bytes(info.value_type, value_ptr)) return false;

  if (info.callback != nullptr) {
    info.callback(config_id, value_ptr, info.user_data);
  }
  return true;
}

}  // namespace vexdash
