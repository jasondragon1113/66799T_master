// [NEEDS-HW-VERIFICATION] Compiled only inside a PROS project. See the
// header and docs/pros-bringup-checklist.md for the mechanism and sources.

#include "vexdash_pros/usb_serial_transport.h"

#include "api.h"       // PROS core (pros::c::task_create/delay/millis, TASK_* macros)
#include "pros/apix.h" // serctl, SERCTL_DISABLE_COBS

#include <fcntl.h>
#include <unistd.h>

namespace vexdash {

UsbSerialTransport::UsbSerialTransport() {
  // Disable PROS's stdout COBS multiplexing so our own framed bytes pass
  // through raw. extra_arg is unused for the COBS actions -> pass NULL.
  pros::c::serctl(SERCTL_DISABLE_COBS, NULL);
  cobs_disabled_ = true;

  // Make STDOUT non-blocking so write() never hangs when the host isn't
  // draining (write() loops with a bounded retry, see below). STDIN is left
  // blocking on purpose -- the dedicated RX task WANTS to block on read().
  int out_flags = fcntl(STDOUT_FILENO, F_GETFL, 0);
  if (out_flags != -1) {
    fcntl(STDOUT_FILENO, F_SETFL, out_flags | O_NONBLOCK);
  }

  // Start the dedicated RX task AFTER the buffers/indices are initialized
  // (they are, being members) and after COBS is disabled. It does the
  // blocking read and fills the ring; the pump-side read() drains it.
  rx_task_ = pros::c::task_create(&UsbSerialTransport::rx_trampoline, this,
                                  TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                                  "vexdash_rx");
}

UsbSerialTransport::~UsbSerialTransport() {
  stop_ = true;
  if (rx_task_ != nullptr) {
    pros::delay(5);  // let a blocked read() return / the loop observe stop_
    pros::c::task_delete(static_cast<pros::task_t>(rx_task_));
    rx_task_ = nullptr;
  }
}

void UsbSerialTransport::rx_trampoline(void* self) {
  static_cast<UsbSerialTransport*>(self)->rx_loop();
}

void UsbSerialTransport::rx_loop() {
  std::uint8_t tmp[256];
  while (!stop_) {
    // Blocking read: wakes as soon as >=1 byte arrives from the host. This
    // blocks THIS task only; the pump/TX loop is unaffected.
    ssize_t n = ::read(STDIN_FILENO, tmp, sizeof(tmp));
    if (n > 0) {
      for (ssize_t i = 0; i < n; ++i) {
        std::uint32_t next = (rx_head_ + 1u) & (kRxBufSize - 1u);
        if (next == rx_tail_) break;  // ring full (shouldn't happen) -> drop
        rx_buf_[rx_head_] = tmp[i];
        rx_head_ = next;
      }
    } else {
      pros::delay(1);  // EOF/error: don't busy-spin
    }
  }
}

std::size_t UsbSerialTransport::write(const std::uint8_t* data, std::size_t len) {
  // STDOUT is non-blocking, so a single ::write can return a short count when
  // the TX buffer is full. Loop to push the WHOLE frame -- a truncated frame
  // fails CRC on the PC and gets dropped (sparse/laggy telemetry). Bounded
  // retry: a connected, draining host gets full frames (smooth); if nobody is
  // reading (buffer never drains) we give up quickly so we can't hang.
  std::size_t total = 0;
  int stalls = 0;
  while (total < len) {
    ssize_t n = ::write(STDOUT_FILENO, data + total, len - total);
    if (n > 0) {
      total += static_cast<std::size_t>(n);
      stalls = 0;
    } else {
      if (++stalls > 5) break;  // ~5ms cap
      pros::delay(1);
    }
  }
  return total;
}

std::size_t UsbSerialTransport::bytes_available() {
  return static_cast<std::size_t>((rx_head_ - rx_tail_) & (kRxBufSize - 1u));
}

std::size_t UsbSerialTransport::read(std::uint8_t* out, std::size_t len) {
  // Non-blocking: drain up to `len` bytes from the ring the RX task fills.
  // Returns 0 when empty, so Session::poll()'s read loop never stalls.
  std::size_t count = 0;
  while (count < len && rx_tail_ != rx_head_) {
    out[count++] = rx_buf_[rx_tail_];
    rx_tail_ = (rx_tail_ + 1u) & (kRxBufSize - 1u);
  }
  return count;
}

std::uint64_t UsbSerialTransport::millis() {
  return static_cast<std::uint64_t>(pros::millis());
}

}  // namespace vexdash
