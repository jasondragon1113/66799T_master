#include "vexdash/telemetry.h"

#include <cstring>

#include "vexdash/byte_writer.h"
#include "vexdash/frame_codec.h"

namespace vexdash {

Telemetry::Telemetry(ITransport& transport) : transport_(transport) {}

bool Telemetry::send_channel_def(ChannelId id, const char* name, ValueType value_type, const char* unit,
                                  int device_port, const char* const* labels, std::size_t label_count) {
  std::size_t name_len = std::strlen(name);
  std::size_t unit_len = std::strlen(unit);
  if (name_len > kMaxNameLen || unit_len > kMaxUnitLen) {
    return false;
  }

  bool has_device_port = device_port >= 0;
  bool has_enum_labels = labels != nullptr && label_count > 0;

  // Validate v1.1 enum-label bounds up front (protocol.md §5.3).
  if (has_enum_labels) {
    if (label_count > kMaxEnumLabels) return false;
    for (std::size_t i = 0; i < label_count; ++i) {
      if (std::strlen(labels[i]) > kMaxEnumLabelLen) return false;
    }
  }
  if (has_device_port && device_port > 0xFF) return false;

  // Payload is bounded by the 512-byte frame limit (protocol.md §2.2); size
  // the buffer accordingly and let ByteWriter/frame_encode reject overflow.
  std::uint8_t payload[kMaxFrameSize - 1 - 2];  // - msg_type - crc16
  ByteWriter w(payload, sizeof(payload));
  w.write_u16(id);
  w.write_u8(static_cast<std::uint8_t>(value_type));
  w.write_string_u8len(name, name_len);
  w.write_string_u8len(unit, unit_len);

  // v1.1 extension bytes (protocol.md §5.3) -- only emitted when needed, so
  // a plain channel with no device_port / no labels stays byte-identical to
  // the v1 CHANNEL_DEF encoding (backward compatible on the wire).
  if (has_device_port || has_enum_labels) {
    std::uint8_t v11_flags = 0;
    if (has_device_port) v11_flags |= kChannelFlagHasDevicePort;
    if (has_enum_labels) v11_flags |= kChannelFlagHasEnumLabels;
    w.write_u8(v11_flags);
    if (has_device_port) {
      w.write_u8(static_cast<std::uint8_t>(device_port));
    }
    if (has_enum_labels) {
      w.write_u8(static_cast<std::uint8_t>(label_count));
      for (std::size_t i = 0; i < label_count; ++i) {
        w.write_string_u8len(labels[i], std::strlen(labels[i]));
      }
    }
  }

  if (!w.ok()) return false;

  std::uint8_t wire[kMaxAccumBufferSize];
  std::size_t wire_len = frame_encode(MsgType::kChannelDef, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;

  return transport_.write(wire, wire_len) == wire_len;
}

ChannelId Telemetry::find_channel_by_name(const char* name) const {
  for (std::size_t i = 0; i < channel_count_; ++i) {
    if (std::strcmp(channels_[i].name, name) == 0) {
      return static_cast<ChannelId>(i);
    }
  }
  return kInvalidChannelId;
}

ChannelId Telemetry::declare_channel(const char* name, ValueType value_type, const char* unit) {
  // v1-shaped CHANNEL_DEF: no device_port, no enum labels -> no v11_flags.
  return declare_channel_ex(name, value_type, unit, ChannelOptions{});
}

ChannelId Telemetry::declare_channel_ex(const char* name, ValueType value_type, const char* unit,
                                         const ChannelOptions& opt) {
  if (std::strlen(name) > kMaxNameLen || std::strlen(unit) > kMaxUnitLen) {
    return kInvalidChannelId;
  }

  int device_port = opt.has_device_port() ? opt.device_port : -1;

  // Idempotent re-declaration: if a channel with this name already exists
  // (e.g. register_all() replayed on reconnect), reuse its id and re-send
  // the def so a freshly-connected dashboard gets the same id -- do NOT
  // allocate a new id (that caused duplicate channels on reconnect).
  ChannelId existing = find_channel_by_name(name);
  if (existing != kInvalidChannelId) {
    channels_[existing].value_type = value_type;  // update in case it changed
    if (!send_channel_def(existing, name, value_type, unit, device_port, nullptr, 0)) {
      return kInvalidChannelId;
    }
    return existing;
  }

  if (channel_count_ >= kMaxChannels) {
    return kInvalidChannelId;
  }
  ChannelId id = static_cast<ChannelId>(channel_count_);
  if (!send_channel_def(id, name, value_type, unit, device_port, nullptr, 0)) {
    return kInvalidChannelId;
  }

  channels_[channel_count_].value_type = value_type;
  std::strcpy(channels_[channel_count_].name, name);  // name_len already <= kMaxNameLen
  ++channel_count_;
  return id;
}

ChannelId Telemetry::declare_enum_channel(const char* name, const char* const* labels,
                                           std::size_t label_count, int device_port, const char* unit) {
  if (std::strlen(name) > kMaxNameLen || std::strlen(unit) > kMaxUnitLen) {
    return kInvalidChannelId;
  }

  // Idempotent re-declaration (see declare_channel_ex).
  ChannelId existing = find_channel_by_name(name);
  if (existing != kInvalidChannelId) {
    channels_[existing].value_type = ValueType::kEnum;
    if (!send_channel_def(existing, name, ValueType::kEnum, unit, device_port, labels, label_count)) {
      return kInvalidChannelId;
    }
    return existing;
  }

  if (channel_count_ >= kMaxChannels) {
    return kInvalidChannelId;
  }
  ChannelId id = static_cast<ChannelId>(channel_count_);
  if (!send_channel_def(id, name, ValueType::kEnum, unit, device_port, labels, label_count)) {
    return kInvalidChannelId;
  }

  channels_[channel_count_].value_type = ValueType::kEnum;
  std::strcpy(channels_[channel_count_].name, name);
  ++channel_count_;
  return id;
}

void Telemetry::put(ChannelId channel, double value) {
  if (pending_count_ >= kMaxSamplesPerFrame) return;  // silently dropped, see header note
  PendingSample& s = pending_[pending_count_++];
  s.channel = channel;
  s.value_type = ValueType::kF64;
  std::memcpy(s.bytes, &value, sizeof(value));
}

void Telemetry::put(ChannelId channel, std::int32_t value) {
  if (pending_count_ >= kMaxSamplesPerFrame) return;
  PendingSample& s = pending_[pending_count_++];
  s.channel = channel;
  s.value_type = ValueType::kI32;
  std::memcpy(s.bytes, &value, sizeof(value));
}

void Telemetry::put(ChannelId channel, bool value) {
  if (pending_count_ >= kMaxSamplesPerFrame) return;
  PendingSample& s = pending_[pending_count_++];
  s.channel = channel;
  s.value_type = ValueType::kBool;
  s.bytes[0] = value ? 0x01 : 0x00;
}

bool Telemetry::flush(std::uint64_t timestamp_ms) {
  if (pending_count_ == 0) {
    pending_count_ = 0;
    return true;  // nothing to send is not an error
  }

  bool all_ok = true;
  std::size_t sent = 0;

  while (sent < pending_count_) {
    // protocol.md §5.2: header is timestamp_ms(8) + sample_count(1); each
    // sample is channel_id(2) + value_type(1) + value(<=8).
    std::uint8_t payload[kMaxFrameSize - 1 - 2];  // - msg_type - crc16
    ByteWriter w(payload, sizeof(payload));
    w.write_u64(timestamp_ms);

    // Reserve the sample_count byte position; patch it after we know how
    // many samples actually fit in this frame's payload budget.
    std::size_t count_pos = w.size();
    w.write_u8(0);  // placeholder

    std::uint8_t count_this_frame = 0;
    while (sent < pending_count_ && count_this_frame < kMaxSamplesPerFrame) {
      const PendingSample& s = pending_[sent];
      std::size_t value_size = value_type_size(s.value_type);

      // Speculatively check whether this sample fits before committing
      // (ByteWriter would just mark itself failed past capacity, but we
      // want to *stop this frame* and start a new one rather than lose
      // the sample).
      // Reserve kTelemetrySeqLen bytes for the trailing v1.2 sequence number
      // so it always fits after the last sample in this frame.
      std::size_t needed = 2 + 1 + value_size;
      if (w.size() + needed + kTelemetrySeqLen > sizeof(payload)) {
        break;  // this sample goes into the next frame
      }

      w.write_u16(s.channel);
      w.write_u8(static_cast<std::uint8_t>(s.value_type));
      w.write_bytes(s.bytes, value_size);
      ++count_this_frame;
      ++sent;
    }

    // protocol.md §5.2/§6.5 (v1.2): append the per-frame u16 sequence number
    // AFTER the samples (like CHANNEL_DEF's v1.1 trailing fields) so a pre-v1.2
    // decoder that stops after sample_count samples ignores it. One seq per
    // emitted frame; wraps naturally at 0xFFFF for a std::uint16_t.
    w.write_u16(tx_seq_);
    ++tx_seq_;

    // Patch sample_count now that we know it.
    payload[count_pos] = count_this_frame;

    if (!w.ok() || count_this_frame == 0) {
      all_ok = false;
      break;
    }

    std::uint8_t wire[kMaxAccumBufferSize];
    std::size_t wire_len = frame_encode(MsgType::kTelemetry, payload, w.size(), wire, sizeof(wire));
    if (wire_len == 0 || transport_.write(wire, wire_len) != wire_len) {
      all_ok = false;
      break;
    }
  }

  pending_count_ = 0;
  return all_ok;
}

bool Telemetry::put_text(ChannelId channel, const char* text) {
  std::size_t text_len = std::strlen(text);
  // protocol.md §5.11: text_len is u16, frame budget bounds it to
  // kMaxFrameSize - 1(msg_type) - 2(channel_id) - 2(text_len) - 2(crc16).
  constexpr std::size_t kMaxTextLen = kMaxFrameSize - 1 - 2 - 2 - 2;
  if (text_len > kMaxTextLen) {
    text_len = kMaxTextLen;  // truncate, per §5.11 suggested encoder behavior
  }

  std::uint8_t payload[2 + 2 + kMaxTextLen];
  ByteWriter w(payload, sizeof(payload));
  w.write_u16(channel);
  w.write_string_u16len(text, text_len);
  if (!w.ok()) return false;

  std::uint8_t wire[kMaxAccumBufferSize];
  std::size_t wire_len = frame_encode(MsgType::kTelemetryText, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

}  // namespace vexdash
