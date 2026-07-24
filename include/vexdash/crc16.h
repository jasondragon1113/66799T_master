#pragma once

#include <cstddef>
#include <cstdint>

namespace vexdash {

// CRC-16/CCITT-FALSE as specified in docs/protocol.md §3.1:
//   poly    = 0x1021
//   init    = 0xFFFF
//   refin   = false
//   refout  = false
//   xorout  = 0x0000
//
// No dynamic allocation; pure function suitable for hot paths.
// Known-answer test per protocol.md: crc16_ccitt_false("123456789") == 0x29B1.
std::uint16_t crc16_ccitt_false(const std::uint8_t* data, std::size_t len);

// Incremental variant: allows feeding bytes as they arrive (e.g. while
// building a frame) without buffering the whole message up front.
// Start with `crc = 0xFFFF`, call crc16_ccitt_false_update() per byte or
// per chunk, final value is the CRC.
std::uint16_t crc16_ccitt_false_update(std::uint16_t crc, std::uint8_t byte);
std::uint16_t crc16_ccitt_false_update(std::uint16_t crc, const std::uint8_t* data, std::size_t len);

constexpr std::uint16_t kCrc16InitialValue = 0xFFFF;

}  // namespace vexdash
