#pragma once

#include <cstddef>
#include <cstdint>

// Consistent Overhead Byte Stuffing (COBS), per docs/protocol.md §2.
//
// Standard COBS algorithm (not the "zero-terminated variant"): guarantees
// the encoded output never contains a 0x00 byte, with worst-case overhead
// of ceil(len/254) bytes. No dynamic allocation: caller supplies output
// buffers; functions report how many bytes were written or needed.

namespace vexdash {

// Maximum encoded size for a given input length (standard COBS bound):
// input length + ceil(input length / 254) + input length == 0 special case.
constexpr std::size_t cobs_max_encoded_size(std::size_t input_len) {
  return input_len + (input_len / 254) + 1;
}

// Encodes `input` (length `input_len`) into `output` (capacity
// `output_capacity`). Does NOT append the trailing 0x00 frame delimiter --
// that is the caller's responsibility (kept orthogonal to encoding itself).
//
// Returns the number of bytes written to `output` on success, or 0 if
// `output_capacity` is insufficient. `output` must not overlap `input`.
std::size_t cobs_encode(const std::uint8_t* input, std::size_t input_len,
                         std::uint8_t* output, std::size_t output_capacity);

// Decodes a COBS-encoded block (`input`, length `input_len`, must NOT
// include the trailing 0x00 delimiter) into `output` (capacity
// `output_capacity`).
//
// Returns the number of bytes written to `output` on success, or 0 if the
// input is malformed (invalid overhead byte / length mismatch) or the
// output buffer is too small. Malformed input must never read or write out
// of bounds -- this is relied upon by the frame decoder's resync logic.
std::size_t cobs_decode(const std::uint8_t* input, std::size_t input_len,
                         std::uint8_t* output, std::size_t output_capacity);

}  // namespace vexdash
