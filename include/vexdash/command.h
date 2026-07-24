#pragma once

#include <cstdint>

#include "vexdash/protocol_types.h"
#include "vexdash/transport.h"

// Command: CMD_DEF (declare a triggerable custom-button action with a
// parameter schema) + dispatch of incoming COMMAND frames, per
// protocol.md §5.9/§5.10.
//
// Usage pattern:
//   Command commands(transport);
//   CmdParamSpec params[] = {
//     CmdParamSpec::f64("x", "mm"),
//     CmdParamSpec::f64("y", "mm"),
//   };
//   CommandId id = commands.declare("Drive to point", params, 2, /*requires_confirm=*/false);
//   commands.set_callback(id, [](CommandId, const CmdParamValue* values, std::size_t count, void*) {
//     // values[0].as_f64, values[1].as_f64
//   });
//   ...
//   // whenever a COMMAND frame's payload has been decoded by Session:
//   commands.dispatch_command(payload, payload_len);
//
// No dynamic allocation: command table and per-command param schema table
// are fixed-size arrays (kMaxCommands, kMaxParamsPerCommand).

namespace vexdash {

using CommandId = std::uint16_t;

constexpr std::size_t kMaxCommands = 32;
constexpr std::size_t kMaxParamsPerCommand = 8;

// protocol.md §5.9 flags bits.
constexpr std::uint8_t kCmdParamFlagHasDefault = 0x01;
constexpr std::uint8_t kCmdParamFlagHasMin = 0x02;
constexpr std::uint8_t kCmdParamFlagHasMax = 0x04;

// Describes one parameter of a command, used at declare()-time to build
// the CMD_DEF wire payload. `name`/`unit` are borrowed pointers (must
// outlive the declare() call, not stored beyond it -- CMD_DEF is sent
// immediately and the wire bytes are the only persisted representation).
struct CmdParamSpec {
  const char* name;
  ValueType value_type;
  const char* unit = "";
  std::uint8_t flags = 0;
  double default_value = 0;  // interpreted per value_type; for bool, != 0 means true
  double min_value = 0;
  double max_value = 0;

  static CmdParamSpec plain(const char* name, ValueType vt, const char* unit = "") {
    CmdParamSpec s;
    s.name = name;
    s.value_type = vt;
    s.unit = unit;
    return s;
  }
};

// A parsed parameter value from an incoming COMMAND frame, exposed to the
// callback in CMD_DEF-declared order.
struct CmdParamValue {
  ValueType value_type;
  union {
    double as_f64;
    std::int32_t as_i32;
    bool as_bool;
  };
};

// See frame_codec.h FrameCallback rationale re: C-style callbacks.
using CommandCallback = void (*)(CommandId id, const CmdParamValue* values, std::size_t value_count,
                                  void* user_data);

class Command {
 public:
  explicit Command(ITransport& transport);

  // Declares a command and sends its CMD_DEF frame immediately. Returns
  // kInvalidCommandId on failure (table full, too many params, name/unit
  // too long).
  CommandId declare(const char* name, const CmdParamSpec* params, std::size_t param_count,
                    bool requires_confirm = false);

  static bool is_valid_command(CommandId id) { return id != kInvalidCommandId; }

  void set_callback(CommandId id, CommandCallback callback, void* user_data);

  // Parses a COMMAND payload (protocol.md §5.10) and, if command_id is
  // known and param_count matches the declared schema, invokes the
  // registered callback with decoded values in declaration order.
  // Unknown command_id or param_count mismatch: frame is safely ignored
  // (protocol.md §5.10 "若對照不到...丟棄" rule) -- returns false in both
  // cases, a diagnostic signal for callers/tests, not a wire-format one.
  bool dispatch_command(const std::uint8_t* payload, std::size_t payload_len);

  std::size_t command_count() const { return command_count_; }

 private:
  static constexpr CommandId kInvalidCommandId = 0xFFFF;

  struct CommandInfo {
    ValueType param_types[kMaxParamsPerCommand];
    std::size_t param_count = 0;
    CommandCallback callback = nullptr;
    void* user_data = nullptr;
    // Fixed-size name copy so re-declaring the same name reuses the id
    // (idempotent registration on reconnect). No dynamic allocation.
    char name[kMaxNameLen + 1] = {0};
  };

  // Returns the id of an already-declared command with this name, or
  // kInvalidCommandId if none. Fixed-size linear scan (kMaxCommands).
  CommandId find_command_by_name(const char* name) const;

  ITransport& transport_;
  CommandInfo commands_[kMaxCommands];
  std::size_t command_count_ = 0;
};

}  // namespace vexdash
