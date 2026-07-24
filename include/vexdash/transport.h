#pragma once

#include <cstddef>
#include <cstdint>

// Abstract transport: lib-core's core does not know where bytes come from
// or go to. Platform adapters (lib-pros/UsbSerialTransport,
// lib-vexcode/VexcodeSmartPortTransport, host test doubles, ...) implement
// this pure-virtual interface. No dynamic allocation, no blocking
// assumptions beyond what each method's contract documents below.

namespace vexdash {

class ITransport {
 public:
  virtual ~ITransport() = default;

  // Writes up to `len` bytes from `data`. Returns the number of bytes
  // actually written (may be less than `len` if the underlying transport
  // buffer is full -- callers must handle partial writes, e.g. by
  // retrying later; this call must not block indefinitely).
  virtual std::size_t write(const std::uint8_t* data, std::size_t len) = 0;

  // Returns the number of bytes currently available to read without
  // blocking.
  virtual std::size_t bytes_available() = 0;

  // Reads up to `len` bytes into `out`. Returns the number of bytes
  // actually read (0 if none available). Must not block.
  virtual std::size_t read(std::uint8_t* out, std::size_t len) = 0;

  // Monotonic milliseconds since an arbitrary epoch (e.g. boot). Used for
  // PING/PONG timestamps and Session housekeeping; each side's clock is
  // independent (protocol.md §5.8 -- not required to be synchronized).
  virtual std::uint64_t millis() = 0;
};

}  // namespace vexdash
