#pragma once

#include <cstdint>
#include <cstring>

#include "vexdash/protocol_types.h"

// Fixed-capacity, no-allocation little-endian byte writer/reader used by
// every message codec (TELEMETRY, CHANNEL_DEF, CONFIG_SCHEMA, CMD_DEF,
// COMMAND, ...). All multi-byte fields are little-endian per protocol.md §0.

namespace vexdash {

class ByteWriter {
 public:
  ByteWriter(std::uint8_t* buf, std::size_t capacity) : buf_(buf), capacity_(capacity) {}

  bool ok() const { return ok_; }
  std::size_t size() const { return pos_; }

  bool write_u8(std::uint8_t v) { return write_bytes(&v, 1); }

  bool write_u16(std::uint16_t v) {
    std::uint8_t b[2] = {static_cast<std::uint8_t>(v & 0xFF), static_cast<std::uint8_t>((v >> 8) & 0xFF)};
    return write_bytes(b, 2);
  }

  bool write_u32(std::uint32_t v) {
    std::uint8_t b[4] = {
        static_cast<std::uint8_t>(v & 0xFF),
        static_cast<std::uint8_t>((v >> 8) & 0xFF),
        static_cast<std::uint8_t>((v >> 16) & 0xFF),
        static_cast<std::uint8_t>((v >> 24) & 0xFF),
    };
    return write_bytes(b, 4);
  }

  bool write_u64(std::uint64_t v) {
    std::uint8_t b[8];
    for (int i = 0; i < 8; ++i) b[i] = static_cast<std::uint8_t>((v >> (8 * i)) & 0xFF);
    return write_bytes(b, 8);
  }

  bool write_i32(std::int32_t v) {
    std::uint32_t u;
    std::memcpy(&u, &v, sizeof(u));
    return write_u32(u);
  }

  bool write_f64(double v) {
    std::uint64_t u;
    std::memcpy(&u, &v, sizeof(u));
    return write_u64(u);
  }

  // IEEE-754 single precision, little-endian (protocol.md §5.13 DEVICE_STATUS
  // values). 中文：單精度浮點、小端序，DEVICE_STATUS 的裝置即時值用。
  bool write_f32(float v) {
    std::uint32_t u;
    std::memcpy(&u, &v, sizeof(u));
    return write_u32(u);
  }

  bool write_bool(bool v) { return write_u8(v ? 0x01 : 0x00); }

  // Writes a length-prefixed UTF-8 string with a u8 length prefix
  // (protocol.md §0 generic rule). Caller must ensure `len <= max_len`
  // (e.g. kMaxNameLen/kMaxUnitLen) before calling -- this writer only
  // enforces the wire-format u8 range (0-255) and buffer capacity.
  bool write_string_u8len(const char* str, std::size_t len) {
    if (len > 255) {
      ok_ = false;
      return false;
    }
    if (!write_u8(static_cast<std::uint8_t>(len))) return false;
    return write_bytes(reinterpret_cast<const std::uint8_t*>(str), len);
  }

  // LOG's msg_len uses u16 (protocol.md §5.5 exception).
  bool write_string_u16len(const char* str, std::size_t len) {
    if (len > 0xFFFF) {
      ok_ = false;
      return false;
    }
    if (!write_u16(static_cast<std::uint16_t>(len))) return false;
    return write_bytes(reinterpret_cast<const std::uint8_t*>(str), len);
  }

  bool write_value(ValueType vt, const std::uint8_t* value_bytes) {
    return write_bytes(value_bytes, value_type_size(vt));
  }

  bool write_bytes(const std::uint8_t* data, std::size_t len) {
    if (!ok_) return false;
    if (pos_ + len > capacity_) {
      ok_ = false;
      return false;
    }
    for (std::size_t i = 0; i < len; ++i) buf_[pos_ + i] = data[i];
    pos_ += len;
    return true;
  }

 private:
  std::uint8_t* buf_;
  std::size_t capacity_;
  std::size_t pos_ = 0;
  bool ok_ = true;
};

class ByteReader {
 public:
  ByteReader(const std::uint8_t* buf, std::size_t len) : buf_(buf), len_(len) {}

  bool ok() const { return ok_; }
  std::size_t remaining() const { return ok_ ? (len_ - pos_) : 0; }
  std::size_t pos() const { return pos_; }

  bool read_u8(std::uint8_t& out) {
    if (!ensure(1)) return false;
    out = buf_[pos_];
    pos_ += 1;
    return true;
  }

  bool read_u16(std::uint16_t& out) {
    if (!ensure(2)) return false;
    out = static_cast<std::uint16_t>(buf_[pos_]) | (static_cast<std::uint16_t>(buf_[pos_ + 1]) << 8);
    pos_ += 2;
    return true;
  }

  bool read_u32(std::uint32_t& out) {
    if (!ensure(4)) return false;
    out = static_cast<std::uint32_t>(buf_[pos_]) | (static_cast<std::uint32_t>(buf_[pos_ + 1]) << 8) |
          (static_cast<std::uint32_t>(buf_[pos_ + 2]) << 16) | (static_cast<std::uint32_t>(buf_[pos_ + 3]) << 24);
    pos_ += 4;
    return true;
  }

  bool read_u64(std::uint64_t& out) {
    if (!ensure(8)) return false;
    out = 0;
    for (int i = 0; i < 8; ++i) out |= static_cast<std::uint64_t>(buf_[pos_ + i]) << (8 * i);
    pos_ += 8;
    return true;
  }

  bool read_i32(std::int32_t& out) {
    std::uint32_t u;
    if (!read_u32(u)) return false;
    std::memcpy(&out, &u, sizeof(out));
    return true;
  }

  bool read_f64(double& out) {
    std::uint64_t u;
    if (!read_u64(u)) return false;
    std::memcpy(&out, &u, sizeof(out));
    return true;
  }

  bool read_f32(float& out) {
    std::uint32_t u;
    if (!read_u32(u)) return false;
    std::memcpy(&out, &u, sizeof(out));
    return true;
  }

  bool read_bool(bool& out) {
    std::uint8_t v;
    if (!read_u8(v)) return false;
    out = (v != 0);
    return true;
  }

  // Returns a pointer into the underlying buffer (no copy) plus length.
  // `out_len` is the length just read from the wire's length prefix.
  bool read_string_u8len(const char*& out_str, std::size_t& out_len) {
    std::uint8_t len;
    if (!read_u8(len)) return false;
    if (!ensure(len)) return false;
    out_str = reinterpret_cast<const char*>(buf_ + pos_);
    out_len = len;
    pos_ += len;
    return true;
  }

  bool read_string_u16len(const char*& out_str, std::size_t& out_len) {
    std::uint16_t len;
    if (!read_u16(len)) return false;
    if (!ensure(len)) return false;
    out_str = reinterpret_cast<const char*>(buf_ + pos_);
    out_len = len;
    pos_ += len;
    return true;
  }

  // Reads `value_type_size(vt)` bytes and returns a pointer to them (no
  // copy); caller interprets via write_value's inverse as needed.
  bool read_value_bytes(ValueType vt, const std::uint8_t*& out_ptr) {
    std::size_t n = value_type_size(vt);
    if (n == 0 || !ensure(n)) return false;
    out_ptr = buf_ + pos_;
    pos_ += n;
    return true;
  }

  bool skip(std::size_t n) {
    if (!ensure(n)) return false;
    pos_ += n;
    return true;
  }

 private:
  bool ensure(std::size_t n) {
    if (!ok_) return false;
    if (pos_ + n > len_) {
      ok_ = false;
      return false;
    }
    return true;
  }

  const std::uint8_t* buf_;
  std::size_t len_;
  std::size_t pos_ = 0;
  bool ok_ = true;
};

}  // namespace vexdash
