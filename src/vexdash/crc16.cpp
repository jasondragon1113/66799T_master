#include "vexdash/crc16.h"

namespace vexdash {

std::uint16_t crc16_ccitt_false_update(std::uint16_t crc, std::uint8_t byte) {
  crc ^= static_cast<std::uint16_t>(byte) << 8;
  for (int i = 0; i < 8; ++i) {
    if (crc & 0x8000) {
      crc = static_cast<std::uint16_t>((crc << 1) ^ 0x1021);
    } else {
      crc = static_cast<std::uint16_t>(crc << 1);
    }
  }
  return crc;
}

std::uint16_t crc16_ccitt_false_update(std::uint16_t crc, const std::uint8_t* data, std::size_t len) {
  for (std::size_t i = 0; i < len; ++i) {
    crc = crc16_ccitt_false_update(crc, data[i]);
  }
  return crc;
}

std::uint16_t crc16_ccitt_false(const std::uint8_t* data, std::size_t len) {
  return crc16_ccitt_false_update(kCrc16InitialValue, data, len);
}

}  // namespace vexdash
