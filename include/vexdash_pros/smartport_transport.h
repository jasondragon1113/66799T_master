#pragma once

// ProsSmartPortTransport -- vexdash ITransport over a V5 Smart Port using
// PROS generic serial (pros::Serial). For the wireless path (ESP32/RS-485
// bridge) or a USB-RS485 dongle wired to a Smart Port. CONTAINS PROS
// HEADERS: compiled only inside a PROS project (host build excludes it).
//
// [NEEDS-HW-VERIFICATION] Not run on hardware from this environment.
//
// pros::Serial (pros/serial.hpp) API used (verified against PROS docs, see
// docs/pros-bringup-checklist.md for URLs):
//   Serial(uint8_t port, int32_t baudrate)     -- port 1..21
//   int32_t write(uint8_t* buf, int32_t len)   -- returns count or PROS_ERR
//   int32_t read(uint8_t* buf, int32_t len)    -- non-blocking, count/PROS_ERR
//   int32_t get_read_avail()                   -- available input bytes/PROS_ERR

#include <cstddef>
#include <cstdint>

#include "pros/apix.h"  // pros::Serial (extended API; includes api.h in correct order, avoids literals clash)
#include "vexdash/transport.h"

namespace vexdash {

class ProsSmartPortTransport : public ITransport {
 public:
  // `smart_port` is 1..21 (Brain smart port number). `baudrate` must match
  // the far end (e.g. the ESP32 bridge). 921600 is the project target;
  // 115200 is the safe fallback (protocol.md §7).
  ProsSmartPortTransport(std::uint8_t smart_port, std::int32_t baudrate);

  std::size_t write(const std::uint8_t* data, std::size_t len) override;
  std::size_t bytes_available() override;
  std::size_t read(std::uint8_t* out, std::size_t len) override;
  std::uint64_t millis() override;

 private:
  pros::Serial serial_;
};

}  // namespace vexdash
