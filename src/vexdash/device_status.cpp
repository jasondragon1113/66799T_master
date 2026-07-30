#include "vexdash/device_status.h"

#include <cstring>

#include "vexdash/byte_writer.h"
#include "vexdash/frame_codec.h"

namespace vexdash {

DeviceStatus::DeviceStatus(ITransport& transport) : transport_(transport) {}

void DeviceStatus::begin() { entry_count_ = 0; }

bool DeviceStatus::add_entry(std::uint8_t port, DeviceType observed_type, const float* values,
                             std::uint8_t value_count) {
  if (entry_count_ >= kMaxDeviceStatusEntries) return false;
  // §5.13: only a type with a defined value set is ever emitted, and the
  // caller's value_count must match that set exactly. A type whose canonical
  // count is 0 (no standard values, e.g. VISION/GPS/EMPTY) is therefore never
  // staged. 中文：只有「有定義值集」的型別才發，且值數量須完全符合契約；契約為 0
  // 的型別（如 VISION/GPS/EMPTY）一律不發。
  const std::uint8_t expected = device_status_value_count(observed_type);
  if (expected == 0) return false;
  if (value_count != expected) return false;
  if (value_count > kMaxDeviceStatusValues) return false;  // defensive
  if (values == nullptr) return false;

  Entry& e = entries_[entry_count_];
  e.port = port;
  e.observed_type = observed_type;
  e.value_count = value_count;
  for (std::uint8_t i = 0; i < value_count; ++i) e.values[i] = values[i];
  ++entry_count_;
  return true;
}

bool DeviceStatus::send_entries(std::size_t begin_idx, std::size_t end_idx) {
  // protocol.md §5.13: payload = entry_count(1) + entries[]. Each entry is
  // port(1) + observed_type(1) + value_count(1) + value_count * f32(4).
  std::uint8_t payload[kMaxFrameSize - 1 - 2];  // - msg_type - crc16
  ByteWriter w(payload, sizeof(payload));
  w.write_u8(static_cast<std::uint8_t>(end_idx - begin_idx));
  for (std::size_t i = begin_idx; i < end_idx; ++i) {
    const Entry& e = entries_[i];
    w.write_u8(e.port);
    w.write_u8(static_cast<std::uint8_t>(e.observed_type));
    w.write_u8(e.value_count);
    for (std::uint8_t v = 0; v < e.value_count; ++v) w.write_f32(e.values[v]);
  }
  if (!w.ok()) return false;

  std::uint8_t wire[kMaxAccumBufferSize];
  std::size_t wire_len = frame_encode(MsgType::kDeviceStatus, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

bool DeviceStatus::send() {
  if (entry_count_ == 0) {
    // Empty snapshot (entry_count=0) -- a legitimate "nothing to report" state.
    return send_entries(0, 0);
  }

  // Payload budget = kMaxFrameSize - msg_type - crc16. Batch entries so each
  // frame stays within it (today always one frame; loop future-proofs bigger
  // value sets). 中文：分批塞進 512-byte frame，目前恆為單 frame。
  constexpr std::size_t kPayloadBudget = kMaxFrameSize - 1 - 2;

  std::size_t idx = 0;
  while (idx < entry_count_) {
    std::size_t used = 1;  // entry_count byte
    std::size_t batch_end = idx;
    while (batch_end < entry_count_) {
      std::size_t entry_size = 1 + 1 + 1 + entries_[batch_end].value_count * 4u;
      if (used + entry_size > kPayloadBudget) break;
      used += entry_size;
      ++batch_end;
    }
    if (batch_end == idx) {
      // A single entry too big to fit even alone -- impossible with today's
      // value sets (max entry 19 bytes: BATTERY or MOTOR [WS10-F], both
      // value_count=4), but guard against an infinite loop.
      return false;
    }
    if (!send_entries(idx, batch_end)) return false;
    idx = batch_end;
  }
  return true;
}

}  // namespace vexdash
