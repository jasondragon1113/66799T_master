#pragma once

// UsbSerialTransport -- vexdash ITransport over the V5 Brain's micro-USB
// serial link (MVP path: Brain USB -> host, read in the browser via Web
// Serial). CONTAINS PROS HEADERS: this file is compiled ONLY inside a PROS
// project, never in the host build (see lib-pros/CMakeLists.txt isolation).
//
// [NEEDS-HW-VERIFICATION] All USB-serial behavior below is grounded in the
// PROS docs cited in docs/pros-bringup-checklist.md and validated on real
// hardware 2026-07-08 (see WORKLOG).
//
// Mechanism:
//   * PROS multiplexes+COBS-encodes its stdout over USB by default. To send
//     RAW bytes (our own COBS+CRC framing handles delimiting) we disable it:
//         pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
//   * Raw bytes go out over STDOUT_FILENO; incoming bytes arrive on
//     STDIN_FILENO (unistd.h POSIX fds, no stdio buffering).
//   * IMPORTANT: on PROS, ::read(STDIN_FILENO) BLOCKS until a byte arrives
//     (O_NONBLOCK is NOT honored on the serial fd). If the pump loop called
//     read() directly it would stall the whole loop at the host's send rate
//     (~2-3 Hz), throttling telemetry. So a DEDICATED RX TASK does the
//     blocking read and pushes bytes into a lock-free SPSC ring buffer; the
//     pump-side read() drains that ring non-blocking. TX (telemetry/ping)
//     therefore runs at full cadence regardless of incoming traffic.
//   * Only one host program may own the USB COM port -- don't run
//     `pros terminal` while the browser holds the port (see checklist).

#include <cstddef>
#include <cstdint>

#include "vexdash/transport.h"

namespace vexdash {

class UsbSerialTransport : public ITransport {
 public:
  UsbSerialTransport();
  ~UsbSerialTransport() override;

  UsbSerialTransport(const UsbSerialTransport&) = delete;
  UsbSerialTransport& operator=(const UsbSerialTransport&) = delete;

  std::size_t write(const std::uint8_t* data, std::size_t len) override;
  std::size_t bytes_available() override;
  std::size_t read(std::uint8_t* out, std::size_t len) override;
  std::uint64_t millis() override;

 private:
  static void rx_trampoline(void* self);
  void rx_loop();  // runs in the dedicated RX task: blocking read -> ring

  // Lock-free single-producer (RX task) / single-consumer (pump) ring.
  // Size MUST be a power of two (index masking). Incoming traffic is low
  // (PONG + occasional CONFIG_SET/COMMAND) so this is comfortably large.
  static constexpr std::size_t kRxBufSize = 4096;
  std::uint8_t rx_buf_[kRxBufSize];
  volatile std::uint32_t rx_head_ = 0;  // written only by the RX task
  volatile std::uint32_t rx_tail_ = 0;  // written only by the consumer (pump)

  volatile bool stop_ = false;
  bool cobs_disabled_ = false;
  void* rx_task_ = nullptr;  // pros::task_t (== void*); created in the ctor body
};

}  // namespace vexdash
