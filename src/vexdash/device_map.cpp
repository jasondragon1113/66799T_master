#include "vexdash/device_map.h"

#include <cstring>

#include "vexdash/byte_writer.h"
#include "vexdash/frame_codec.h"

namespace vexdash {

DeviceMap::DeviceMap(ITransport& transport) : transport_(transport) {}

void DeviceMap::begin() { entry_count_ = 0; }

bool DeviceMap::add_port(std::uint8_t port, DeviceType device_type, bool connected, const char* name) {
  if (entry_count_ >= kMaxDeviceEntries) return false;
  std::size_t name_len = std::strlen(name);
  if (name_len > kMaxDeviceNameLen) return false;

  Entry& e = entries_[entry_count_];
  e.port = port;
  e.device_type = device_type;
  e.connected = connected;
  e.name_len = static_cast<std::uint8_t>(name_len);
  std::memcpy(e.name, name, name_len);
  ++entry_count_;
  return true;
}

bool DeviceMap::send_entries(std::size_t begin_idx, std::size_t end_idx) {
  // protocol.md §5.12: payload = port_count(1) + entries[]. Each entry is
  // port(1) + device_type(1) + connected(1) + name_len(1) + name.
  std::uint8_t payload[kMaxFrameSize - 1 - 2];  // - msg_type - crc16
  ByteWriter w(payload, sizeof(payload));
  w.write_u8(static_cast<std::uint8_t>(end_idx - begin_idx));
  for (std::size_t i = begin_idx; i < end_idx; ++i) {
    const Entry& e = entries_[i];
    w.write_u8(e.port);
    w.write_u8(static_cast<std::uint8_t>(e.device_type));
    w.write_bool(e.connected);
    w.write_string_u8len(e.name, e.name_len);
  }
  if (!w.ok()) return false;

  std::uint8_t wire[kMaxAccumBufferSize];
  std::size_t wire_len = frame_encode(MsgType::kDeviceMap, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

bool DeviceMap::send() {
  if (entry_count_ == 0) {
    // Send an empty snapshot (port_count=0) -- a legitimate "all ports
    // empty / nothing detected" state, not an error.
    return send_entries(0, 0);
  }

  // Budget per entry: header(1) + port(1) + device_type(1) + connected(1) +
  // name_len(1) + name. Payload budget = kMaxFrameSize - msg_type - crc16.
  constexpr std::size_t kPayloadBudget = kMaxFrameSize - 1 - 2;

  std::size_t idx = 0;
  while (idx < entry_count_) {
    std::size_t used = 1;  // port_count byte
    std::size_t batch_end = idx;
    while (batch_end < entry_count_) {
      std::size_t entry_size = 1 + 1 + 1 + 1 + entries_[batch_end].name_len;
      if (used + entry_size > kPayloadBudget) break;
      used += entry_size;
      ++batch_end;
    }
    if (batch_end == idx) {
      // A single entry that cannot fit even alone -- shouldn't happen given
      // name is capped at 63 bytes (max entry 67 < budget), but guard
      // against an infinite loop just in case.
      return false;
    }
    if (!send_entries(idx, batch_end)) return false;
    idx = batch_end;
  }
  return true;
}

}  // namespace vexdash
