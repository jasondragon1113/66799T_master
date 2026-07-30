#pragma once

#include <cstdint>

#include "vexdash/command.h"
#include "vexdash/config.h"
#include "vexdash/device_map.h"
#include "vexdash/device_status.h"
#include "vexdash/field_view.h"
#include "vexdash/frame_codec.h"
#include "vexdash/protocol_types.h"
#include "vexdash/telemetry.h"
#include "vexdash/transport.h"

// Session: top-level orchestrator that owns a transport binding and wires
// together Telemetry/FieldView/Config/Command with the frame codec.
//
// Responsibilities (protocol.md §6):
//   - Sends HELLO on construction with this side's role (Robot) and the
//     v1 protocol version (1, 0).
//   - poll() drains available bytes from the transport into an
//     incremental FrameDecoder and dispatches decoded frames:
//       * HELLO       -> records remote (major, minor, role)
//       * CONFIG_SET  -> forwarded to the owned Config
//       * COMMAND     -> forwarded to the owned Command
//       * PING        -> auto-replies with PONG (protocol.md §5.8)
//       * PONG        -> recorded (rtt/offset calc left to caller, needs
//                        its own "now" reading which Session doesn't own)
//       * others      -> ignored (protocol.md §4 unknown-type policy)
//   - log() sends a LOG frame (protocol.md §5.5).
//   - ping() sends a PING frame with the caller-supplied timestamp.
//
// Session does not itself drive a scheduling loop or own a thread --
// callers (adapter layers) call poll() from their own task/loop.
// No dynamic allocation: everything is fixed-size members.

namespace vexdash {

struct RemoteInfo {
  bool received = false;
  std::uint8_t version_major = 0;
  std::uint8_t version_minor = 0;
  Role role = Role::kRobot;
};

struct PongInfo {
  bool received = false;
  std::uint64_t original_sender_time_ms = 0;
  std::uint64_t responder_time_ms = 0;
};

// protocol.md §6.1: v1.4 corresponds to (major=1, minor=4). Announcing the
// minor in HELLO lets the peer's UI decide whether to enable newer panels (e.g.
// the v1.3 DEVICE_STATUS live-value panel) or word a hint honestly (e.g. "this
// firmware is pre-v1.4, mechanism grouping is a guess"); it is NOT a
// precondition for decoding v1.1 CHANNEL_DEF fields, the v1.2 TELEMETRY sequence
// number, v1.3 DEVICE_STATUS, or the v1.4 CHANNEL_DEF path
// (§6.4/§6.5/§6.6/§6.7: all detected structurally / by msg_type / by flag bit,
// not gated on the version, so an old peer that never sent HELLO still interoperates).
constexpr std::uint8_t kProtocolVersionMajor = 1;
constexpr std::uint8_t kProtocolVersionMinor = 4;

class Session {
 public:
  // `role` is this side's role to announce in HELLO (protocol.md §5.1);
  // lib-core is robot-side so this is normally Role::kRobot, but the
  // parameter exists so host-side tests can exercise both directions.
  Session(ITransport& transport, Role role = Role::kRobot);

  Telemetry& telemetry() { return telemetry_; }
  FieldView& field() { return field_; }
  Config& config() { return config_; }
  Command& command() { return command_; }
  DeviceMap& device_map() { return device_map_; }        // v1.1 (protocol.md §5.12)
  DeviceStatus& device_status() { return device_status_; }  // v1.3 (protocol.md §5.13)

  // Sends this side's HELLO frame (protocol.md §6.2: send proactively,
  // don't wait for the remote's HELLO). Safe to call more than once
  // (e.g. on reconnect) though v1 doesn't require repeated HELLOs.
  bool send_hello();

  // Sends a LOG frame. Truncates `message` to fit the frame budget if
  // needed (protocol.md §5.5 encoder-side suggested behavior); returns
  // false only on transport/encode failure, not on truncation.
  bool log(LogLevel level, const char* message);

  // Sends a PING frame carrying `sender_time_ms` (caller's own clock
  // reading -- Session doesn't call transport.millis() itself so the
  // caller controls which clock/units are used consistently with its
  // own RTT bookkeeping).
  bool ping(std::uint64_t sender_time_ms);

  // Drains all currently-available bytes from the transport and feeds
  // them through the frame decoder, dispatching any complete frames.
  // Returns the number of bytes read from the transport this call (0 if
  // none were available). Never blocks beyond what ITransport::read()
  // itself does (which per its contract must not block).
  std::size_t poll();

  const RemoteInfo& remote_info() const { return remote_info_; }
  const PongInfo& last_pong() const { return last_pong_; }
  const FrameDecoder::Stats& decoder_stats() const { return decoder_.stats(); }

 private:
  static void on_frame(const DecodedFrame& frame, void* user_data);
  void handle_frame(const DecodedFrame& frame);
  void handle_hello(const std::uint8_t* payload, std::size_t len);
  void handle_ping(const std::uint8_t* payload, std::size_t len);
  void handle_pong(const std::uint8_t* payload, std::size_t len);

  ITransport& transport_;
  Role role_;

  Telemetry telemetry_;
  FieldView field_;
  Config config_;
  Command command_;
  DeviceMap device_map_;
  DeviceStatus device_status_;

  FrameDecoder decoder_;

  RemoteInfo remote_info_;
  PongInfo last_pong_;

  // Scratch read buffer for poll(); fixed size, no allocation.
  static constexpr std::size_t kPollChunkSize = 256;
  std::uint8_t poll_buffer_[kPollChunkSize];
};

}  // namespace vexdash
