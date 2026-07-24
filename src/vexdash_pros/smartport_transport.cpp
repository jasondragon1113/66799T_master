// [NEEDS-HW-VERIFICATION] Compiled only inside a PROS project. See header
// and docs/pros-bringup-checklist.md for the mechanism and sources.

#include "vexdash_pros/smartport_transport.h"

#include "api.h"  // pros::millis

namespace vexdash {

ProsSmartPortTransport::ProsSmartPortTransport(std::uint8_t smart_port, std::int32_t baudrate)
    : serial_(smart_port, baudrate) {}

std::size_t ProsSmartPortTransport::write(const std::uint8_t* data, std::size_t len) {
  // pros::Serial::write takes a non-const uint8_t*; the bytes are not
  // modified. Returns count written or PROS_ERR (-1) on failure.
  std::int32_t n = serial_.write(const_cast<std::uint8_t*>(data), static_cast<std::int32_t>(len));
  if (n <= 0) return 0;
  return static_cast<std::size_t>(n);
}

std::size_t ProsSmartPortTransport::bytes_available() {
  std::int32_t n = serial_.get_read_avail();
  if (n <= 0) return 0;
  return static_cast<std::size_t>(n);
}

std::size_t ProsSmartPortTransport::read(std::uint8_t* out, std::size_t len) {
  // pros::Serial::read is non-blocking: returns only currently-available
  // bytes, or PROS_ERR. Honors the ITransport no-block contract.
  std::int32_t n = serial_.read(out, static_cast<std::int32_t>(len));
  if (n <= 0) return 0;
  return static_cast<std::size_t>(n);
}

std::uint64_t ProsSmartPortTransport::millis() {
  return static_cast<std::uint64_t>(pros::millis());
}

}  // namespace vexdash
