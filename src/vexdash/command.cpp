#include "vexdash/command.h"

#include <cstring>

#include "vexdash/byte_writer.h"
#include "vexdash/frame_codec.h"

namespace vexdash {

Command::Command(ITransport& transport) : transport_(transport) {}

namespace {

// Writes one CmdParamSpec's wire encoding (protocol.md §5.9 params[]
// item) into `w`. Returns false (and marks `w` failed) if name/unit
// exceed limits or the writer runs out of capacity.
bool write_param_spec(ByteWriter& w, const CmdParamSpec& spec) {
  std::size_t name_len = std::strlen(spec.name);
  std::size_t unit_len = std::strlen(spec.unit);
  if (name_len > kMaxNameLen || unit_len > kMaxUnitLen) return false;

  // protocol.md §5.9: CMD_DEF param value_type is restricted to the §5.2
  // numeric enumeration (f64/i32/bool). The v1.1 CHANNEL_DEF-only types
  // (kString/kEnum) are not valid command parameters -- reject them.
  if (spec.value_type != ValueType::kF64 && spec.value_type != ValueType::kI32 &&
      spec.value_type != ValueType::kBool) {
    return false;
  }

  w.write_string_u8len(spec.name, name_len);
  w.write_u8(static_cast<std::uint8_t>(spec.value_type));
  w.write_string_u8len(spec.unit, unit_len);
  w.write_u8(spec.flags);

  std::size_t value_size = value_type_size(spec.value_type);

  auto write_scalar = [&](double v) {
    switch (spec.value_type) {
      case ValueType::kF64: {
        double d = v;
        w.write_bytes(reinterpret_cast<const std::uint8_t*>(&d), value_size);
        break;
      }
      case ValueType::kI32: {
        std::int32_t i = static_cast<std::int32_t>(v);
        w.write_bytes(reinterpret_cast<const std::uint8_t*>(&i), value_size);
        break;
      }
      case ValueType::kBool: {
        std::uint8_t b = (v != 0) ? 0x01 : 0x00;
        w.write_bytes(&b, value_size);
        break;
      }
      case ValueType::kString:
      case ValueType::kEnum:
        // Rejected earlier in write_param_spec(); unreachable here.
        break;
    }
  };

  if (spec.flags & kCmdParamFlagHasDefault) write_scalar(spec.default_value);
  if (spec.flags & kCmdParamFlagHasMin) write_scalar(spec.min_value);
  if (spec.flags & kCmdParamFlagHasMax) write_scalar(spec.max_value);

  return w.ok();
}

}  // namespace

CommandId Command::find_command_by_name(const char* name) const {
  for (std::size_t i = 0; i < command_count_; ++i) {
    if (std::strcmp(commands_[i].name, name) == 0) {
      return static_cast<CommandId>(i);
    }
  }
  return kInvalidCommandId;
}

CommandId Command::declare(const char* name, const CmdParamSpec* params, std::size_t param_count,
                            bool requires_confirm) {
  if (param_count > kMaxParamsPerCommand) return kInvalidCommandId;

  std::size_t name_len = std::strlen(name);
  if (name_len > kMaxNameLen) return kInvalidCommandId;

  // Idempotent re-declaration: reuse an existing command's id and re-send
  // its CMD_DEF (so a reconnecting dashboard gets the same id + button),
  // rather than allocating a new id. Preserves the registered callback.
  CommandId existing = find_command_by_name(name);
  bool reuse = existing != kInvalidCommandId;
  if (!reuse && command_count_ >= kMaxCommands) return kInvalidCommandId;

  CommandId id = reuse ? existing : static_cast<CommandId>(command_count_);

  // Conservative payload buffer: header + name + up to kMaxParamsPerCommand
  // params, each up to name(63)+unit(15)+type/flags(2)+3*8 value bytes.
  constexpr std::size_t kMaxParamWireSize = 1 + kMaxNameLen + 1 + 1 + kMaxUnitLen + 1 + 3 * 8;
  std::uint8_t payload[2 + 1 + 1 + kMaxNameLen + 1 + kMaxParamsPerCommand * kMaxParamWireSize];
  ByteWriter w(payload, sizeof(payload));
  w.write_u16(id);
  w.write_u8(requires_confirm ? 0x01 : 0x00);
  w.write_string_u8len(name, name_len);
  w.write_u8(static_cast<std::uint8_t>(param_count));

  for (std::size_t i = 0; i < param_count; ++i) {
    if (!write_param_spec(w, params[i])) return kInvalidCommandId;
  }
  if (!w.ok()) return kInvalidCommandId;

  std::uint8_t wire[kMaxAccumBufferSize];
  std::size_t wire_len = frame_encode(MsgType::kCmdDef, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return kInvalidCommandId;
  if (transport_.write(wire, wire_len) != wire_len) return kInvalidCommandId;

  CommandInfo& info = commands_[id];
  info.param_count = param_count;
  for (std::size_t i = 0; i < param_count; ++i) {
    info.param_types[i] = params[i].value_type;
  }
  if (!reuse) {
    info.callback = nullptr;
    info.user_data = nullptr;
    std::strcpy(info.name, name);  // name_len already <= kMaxNameLen
    ++command_count_;
  }  // on reuse: keep existing callback/user_data/name
  return id;
}

void Command::set_callback(CommandId id, CommandCallback callback, void* user_data) {
  if (id >= command_count_) return;
  commands_[id].callback = callback;
  commands_[id].user_data = user_data;
}

bool Command::dispatch_command(const std::uint8_t* payload, std::size_t payload_len) {
  ByteReader r(payload, payload_len);
  std::uint16_t command_id;
  std::uint8_t param_count;
  if (!r.read_u16(command_id)) return false;
  if (!r.read_u8(param_count)) return false;

  // protocol.md §5.10 step 2: unknown command_id -> entire frame dropped,
  // do not attempt to parse param_values (we don't know each value's
  // width without the schema).
  if (command_id >= command_count_) return false;

  CommandInfo& info = commands_[command_id];

  // protocol.md §5.10 step 3: param_count mismatch -> drop, do not read
  // further (receiver may be in a stale/incompatible state).
  if (param_count != info.param_count) return false;

  CmdParamValue values[kMaxParamsPerCommand];
  for (std::size_t i = 0; i < param_count; ++i) {
    ValueType vt = info.param_types[i];
    const std::uint8_t* value_ptr;
    if (!r.read_value_bytes(vt, value_ptr)) return false;

    values[i].value_type = vt;
    switch (vt) {
      case ValueType::kF64:
        std::memcpy(&values[i].as_f64, value_ptr, sizeof(double));
        break;
      case ValueType::kI32:
        std::memcpy(&values[i].as_i32, value_ptr, sizeof(std::int32_t));
        break;
      case ValueType::kBool:
        values[i].as_bool = (value_ptr[0] != 0);
        break;
      case ValueType::kString:
      case ValueType::kEnum:
        // Not permitted as command params (rejected at declare time);
        // read_value_bytes would also have failed for kString (size 0).
        return false;
    }
  }

  if (info.callback != nullptr) {
    info.callback(command_id, values, param_count, info.user_data);
  }
  return true;
}

}  // namespace vexdash
