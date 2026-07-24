#include "vexdash/frame_codec.h"

#include "vexdash/cobs.h"
#include "vexdash/crc16.h"

namespace vexdash {

std::size_t frame_encode(MsgType msg_type, const std::uint8_t* payload, std::size_t payload_len,
                          std::uint8_t* out_wire, std::size_t out_wire_capacity) {
  const std::size_t raw_len = 1 + payload_len + 2;
  if (raw_len > kMaxFrameSize) {
    return 0;
  }
  if (payload_len > 0 && payload == nullptr) {
    return 0;
  }

  // Build the raw frame (msg_type ++ payload ++ crc16_le) in a fixed-size
  // stack buffer sized to kMaxFrameSize (protocol.md §2.2 bound).
  std::uint8_t raw[kMaxFrameSize];
  raw[0] = static_cast<std::uint8_t>(msg_type);
  for (std::size_t i = 0; i < payload_len; ++i) {
    raw[1 + i] = payload[i];
  }

  std::uint16_t crc = crc16_ccitt_false(raw, 1 + payload_len);
  raw[1 + payload_len] = static_cast<std::uint8_t>(crc & 0xFF);          // low byte first (LE)
  raw[1 + payload_len + 1] = static_cast<std::uint8_t>((crc >> 8) & 0xFF);

  if (out_wire_capacity < frame_max_wire_size(raw_len)) {
    return 0;
  }

  std::size_t cobs_len = cobs_encode(raw, raw_len, out_wire, out_wire_capacity);
  if (cobs_len == 0) {
    return 0;
  }
  if (cobs_len + 1 > out_wire_capacity) {
    return 0;
  }
  out_wire[cobs_len] = 0x00;
  return cobs_len + 1;
}

FrameDecoder::FrameDecoder(FrameCallback callback, void* user_data)
    : callback_(callback), user_data_(user_data) {}

void FrameDecoder::reset_accum() { accum_len_ = 0; }

void FrameDecoder::on_delimiter() {
  if (accum_len_ == 0) {
    // Back-to-back delimiters (e.g. leading 0x00 at stream start, or a
    // resync) with nothing accumulated: nothing to decode, not an error.
    return;
  }

  std::size_t decoded_len = cobs_decode(accum_, accum_len_, decoded_, sizeof(decoded_));
  if (decoded_len == 0) {
    ++stats_.frames_cobs_failed;
    reset_accum();
    return;
  }

  // Need at least msg_type(1) + crc16(2).
  if (decoded_len < 3) {
    ++stats_.frames_too_short;
    reset_accum();
    return;
  }

  std::size_t payload_len = decoded_len - 3;
  std::uint8_t msg_type_raw = decoded_[0];
  const std::uint8_t* payload = decoded_ + 1;

  std::uint16_t received_crc = static_cast<std::uint16_t>(decoded_[1 + payload_len]) |
                                (static_cast<std::uint16_t>(decoded_[1 + payload_len + 1]) << 8);
  std::uint16_t computed_crc = crc16_ccitt_false(decoded_, 1 + payload_len);

  if (received_crc != computed_crc) {
    ++stats_.frames_crc_failed;
    reset_accum();
    return;
  }

  ++stats_.frames_ok;
  DecodedFrame frame{msg_type_raw, payload, payload_len};
  reset_accum();  // reset before invoking callback in case callback re-enters feed()
  if (callback_ != nullptr) {
    callback_(frame, user_data_);
  }
}

void FrameDecoder::feed(std::uint8_t byte) {
  if (byte == 0x00) {
    on_delimiter();
    return;
  }

  if (accum_len_ >= sizeof(accum_)) {
    // protocol.md §2.3 upper-bound protection: stream corruption, drop
    // accumulated bytes and keep scanning for the next 0x00. Never crash.
    // The byte that triggered the overflow is not itself discarded -- it
    // becomes the first byte of the next accumulation attempt, so the
    // buffer is guaranteed empty (not off-by-one) immediately after a
    // clean multiple of the buffer size.
    ++stats_.overflow_resyncs;
    reset_accum();
  }

  accum_[accum_len_++] = byte;
}

void FrameDecoder::feed(const std::uint8_t* data, std::size_t len) {
  for (std::size_t i = 0; i < len; ++i) {
    feed(data[i]);
  }
}

}  // namespace vexdash
