#include "vexdash/cobs.h"

namespace vexdash {

std::size_t cobs_encode(const std::uint8_t* input, std::size_t input_len,
                         std::uint8_t* output, std::size_t output_capacity) {
  if (output_capacity < cobs_max_encoded_size(input_len)) {
    // Capacity check is conservative (uses the worst-case bound), so a
    // pass here guarantees no overrun below regardless of data content.
    return 0;
  }

  std::size_t read_idx = 0;
  std::size_t write_idx = 0;
  // Position in `output` where the next overhead (code) byte will be
  // written; reserved up front, patched once we know the run length.
  std::size_t code_pos = write_idx;
  ++write_idx;
  std::uint8_t code = 1;

  while (read_idx < input_len) {
    std::uint8_t byte = input[read_idx++];
    if (byte == 0x00) {
      output[code_pos] = code;
      code_pos = write_idx++;
      code = 1;
    } else {
      output[write_idx++] = byte;
      ++code;
      if (code == 0xFF) {
        output[code_pos] = code;
        code_pos = write_idx++;
        code = 1;
      }
    }
  }
  output[code_pos] = code;

  return write_idx;
}

std::size_t cobs_decode(const std::uint8_t* input, std::size_t input_len,
                         std::uint8_t* output, std::size_t output_capacity) {
  if (input_len == 0) {
    // A zero-length COBS block is malformed: even an empty original
    // message encodes to at least one code byte (0x01).
    return 0;
  }

  std::size_t read_idx = 0;
  std::size_t write_idx = 0;

  while (read_idx < input_len) {
    std::uint8_t code = input[read_idx];
    if (code == 0x00) {
      // 0x00 is never a valid COBS code byte inside an encoded block.
      return 0;
    }
    std::size_t block_len = static_cast<std::size_t>(code) - 1;

    if (read_idx + 1 + block_len > input_len) {
      // Overhead byte claims more data bytes than remain in the input:
      // malformed / truncated block.
      return 0;
    }
    ++read_idx;

    if (write_idx + block_len > output_capacity) {
      return 0;
    }
    for (std::size_t i = 0; i < block_len; ++i) {
      output[write_idx++] = input[read_idx++];
    }

    // A code of 0xFF means "254 data bytes, no implicit zero follows"
    // (standard COBS rule). Any other code implies an implicit zero
    // between blocks, unless this was the final block.
    bool more_input = read_idx < input_len;
    if (code != 0xFF && more_input) {
      if (write_idx >= output_capacity) {
        return 0;
      }
      output[write_idx++] = 0x00;
    }
  }

  return write_idx;
}

}  // namespace vexdash
