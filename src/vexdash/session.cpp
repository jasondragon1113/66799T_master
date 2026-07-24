#include "vexdash/session.h"

#include <cstdio>
#include <cstring>

#include "vexdash/byte_writer.h"

namespace vexdash {

Session::Session(ITransport& transport, Role role)
    : transport_(transport),
      role_(role),
      telemetry_(transport),
      field_(transport),
      config_(transport),
      command_(transport),
      device_map_(transport),
      device_status_(transport),
      decoder_(&Session::on_frame, this) {
  // protocol.md §6.2: send HELLO proactively on connection, don't wait for
  // the remote's HELLO before starting normal operation.
  send_hello();
}

bool Session::send_hello() {
  std::uint8_t payload[3];
  ByteWriter w(payload, sizeof(payload));
  w.write_u8(kProtocolVersionMajor);
  w.write_u8(kProtocolVersionMinor);
  w.write_u8(static_cast<std::uint8_t>(role_));
  if (!w.ok()) return false;

  std::uint8_t wire[frame_max_wire_size(1 + sizeof(payload) + 2)];
  std::size_t wire_len = frame_encode(MsgType::kHello, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

bool Session::log(LogLevel level, const char* message) {
  std::size_t msg_len = std::strlen(message);
  // protocol.md §5.5: msg_len is u16, frame budget bounds it to
  // kMaxFrameSize - 1(msg_type) - 1(level) - 2(msg_len) - 2(crc16).
  constexpr std::size_t kMaxLogMsgLen = kMaxFrameSize - 1 - 1 - 2 - 2;
  if (msg_len > kMaxLogMsgLen) {
    msg_len = kMaxLogMsgLen;  // truncate, per protocol.md §5.5 suggested encoder behavior
  }

  std::uint8_t payload[1 + 2 + kMaxLogMsgLen];
  ByteWriter w(payload, sizeof(payload));
  w.write_u8(static_cast<std::uint8_t>(level));
  w.write_string_u16len(message, msg_len);
  if (!w.ok()) return false;

  std::uint8_t wire[kMaxAccumBufferSize];
  std::size_t wire_len = frame_encode(MsgType::kLog, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

bool Session::ping(std::uint64_t sender_time_ms) {
  std::uint8_t payload[8];
  ByteWriter w(payload, sizeof(payload));
  w.write_u64(sender_time_ms);
  if (!w.ok()) return false;

  std::uint8_t wire[frame_max_wire_size(1 + sizeof(payload) + 2)];
  std::size_t wire_len = frame_encode(MsgType::kPing, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

std::size_t Session::poll() {
  std::size_t total_read = 0;
  std::size_t n;
  while ((n = transport_.read(poll_buffer_, sizeof(poll_buffer_))) > 0) {
    decoder_.feed(poll_buffer_, n);
    total_read += n;
    if (n < sizeof(poll_buffer_)) break;  // drained what was available
  }
  return total_read;
}

void Session::on_frame(const DecodedFrame& frame, void* user_data) {
  static_cast<Session*>(user_data)->handle_frame(frame);
}

void Session::handle_frame(const DecodedFrame& frame) {
  switch (frame.msg_type_raw) {
    case static_cast<std::uint8_t>(MsgType::kHello):
      handle_hello(frame.payload, frame.payload_len);
      break;
    case static_cast<std::uint8_t>(MsgType::kConfigSet):
      config_.dispatch_config_set(frame.payload, frame.payload_len);
      break;
    case static_cast<std::uint8_t>(MsgType::kCommand):
      command_.dispatch_command(frame.payload, frame.payload_len);
      break;
    case static_cast<std::uint8_t>(MsgType::kPing):
      handle_ping(frame.payload, frame.payload_len);
      break;
    case static_cast<std::uint8_t>(MsgType::kPong):
      handle_pong(frame.payload, frame.payload_len);
      break;
    default:
      // protocol.md §4: unknown / not-relevant-to-robot-side message
      // types are ignored (e.g. TELEMETRY/CHANNEL_DEF/FIELD_OPS/LOG/
      // CONFIG_SCHEMA/CMD_DEF are Robot->PC only, a robot-side Session
      // should not normally receive them, but ignoring is still safe).
      break;
  }
}

void Session::handle_hello(const std::uint8_t* payload, std::size_t len) {
  ByteReader r(payload, len);
  std::uint8_t major, minor, role_raw;
  if (!r.read_u8(major)) return;
  if (!r.read_u8(minor)) return;
  if (!r.read_u8(role_raw)) return;

  remote_info_.received = true;
  remote_info_.version_major = major;
  remote_info_.version_minor = minor;
  remote_info_.role = static_cast<Role>(role_raw);

  if (major != kProtocolVersionMajor) {
    char msg[64];
    // Global (not std::) snprintf: VEXcode's newlib toolchain only declares
    // this in the global namespace, and this file is amalgamated verbatim
    // into lib-core/amalgam/vexdash.h for the VEXcode build (see
    // dashboard/scripts/gen-vexdash-amalgam.ts). <cstdio> also makes the
    // global name available under the host/PROS toolchains, so this stays
    // portable across all three targets.
    snprintf(msg, sizeof(msg), "protocol major version mismatch: local=%u remote=%u",
             kProtocolVersionMajor, major);
    log(LogLevel::kWarn, msg);
  }
}

void Session::handle_ping(const std::uint8_t* payload, std::size_t len) {
  ByteReader r(payload, len);
  std::uint64_t sender_time_ms;
  if (!r.read_u64(sender_time_ms)) return;

  std::uint8_t pong_payload[16];
  ByteWriter w(pong_payload, sizeof(pong_payload));
  w.write_u64(sender_time_ms);          // original_sender_time_ms, unchanged
  w.write_u64(transport_.millis());     // responder_time_ms
  if (!w.ok()) return;

  std::uint8_t wire[frame_max_wire_size(1 + sizeof(pong_payload) + 2)];
  std::size_t wire_len = frame_encode(MsgType::kPong, pong_payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return;
  transport_.write(wire, wire_len);
}

void Session::handle_pong(const std::uint8_t* payload, std::size_t len) {
  ByteReader r(payload, len);
  std::uint64_t original_sender_time_ms, responder_time_ms;
  if (!r.read_u64(original_sender_time_ms)) return;
  if (!r.read_u64(responder_time_ms)) return;

  last_pong_.received = true;
  last_pong_.original_sender_time_ms = original_sender_time_ms;
  last_pong_.responder_time_ms = responder_time_ms;
}

}  // namespace vexdash
