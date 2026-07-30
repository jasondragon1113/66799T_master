#pragma once

#include <cstdint>

#include "vexdash/protocol_types.h"
#include "vexdash/transport.h"

// DeviceStatus: builds and sends DEVICE_STATUS frames (0x0E, v1.3,
// protocol.md §5.13) -- the low-frequency (~1Hz) per-device live values of
// every declared-and-detected device (motor temp/power/current, optical
// hue/proximity/brightness, ...).
//
// 中文：DeviceStatus 負責組出並送出 DEVICE_STATUS（0x0E，§5.13）——每秒一次，把
// 「宣告過且真的偵測到」的裝置的 per-型別標準值集打包送給前端顯示在埠格子裡。
//
// Usage pattern (host-testable; PROS value reads happen upstream and are
// passed in as plain floats -- this class has NO PROS dependency):
//   DeviceStatus ds(transport);
//   ds.begin();
//   float m[4] = {41.5f, 3.2f, 850.0f, 127.0f};  // temp C, power W, current mA, rpm (WS10-F)
//   ds.add_entry(3, DeviceType::kMotor, m, 4);    // observed type = MOTOR
//   ds.send();   // encodes+sends, splitting across frames if ever needed
//
// The value order per observed_type is the §5.13 contract; the caller (a PROS
// value reader) is responsible for filling `values` in that order. This class
// only validates value_count against the type's canonical count and encodes.
// No dynamic allocation: the staging buffer is a fixed-size member.

namespace vexdash {

// Max entries stageable before send(). Covers all 21 smart ports (DEVICE_STATUS
// never emits ADI ports, §5.13); sized statically (no allocation).
// 中文：最多可暫存的 entry 數，足夠涵蓋 21 個 smart port。
constexpr std::size_t kMaxDeviceStatusEntries = 21;

class DeviceStatus {
 public:
  explicit DeviceStatus(ITransport& transport);

  // Clears any staged entries to start building a fresh ~1Hz snapshot.
  void begin();

  // Stages one device entry. `observed_type` is the ACTUALLY detected type
  // (§5.13). `values` points at `value_count` floats in the §5.13 canonical
  // order for that type. Returns false (and stages nothing) if:
  //   - the staging buffer is full (kMaxDeviceStatusEntries), or
  //   - `value_count` != device_status_value_count(observed_type) -- i.e. the
  //     caller's value set doesn't match the type contract (a type whose
  //     canonical count is 0 therefore can never be staged; §5.13 says such a
  //     device is simply not emitted).
  // 中文：暫存一筆 entry。observed_type＝實測型別；values 依 §5.13 該型別的順序帶
  // value_count 個 float。value_count 與該型別契約不符（含契約為 0）＝拒收回 false。
  bool add_entry(std::uint8_t port, DeviceType observed_type, const float* values,
                 std::uint8_t value_count);

  // Encodes and sends the staged snapshot. Splits across multiple
  // DEVICE_STATUS frames if the entries ever exceed one 512-byte frame (they
  // can't with today's value sets -- 21*15+1 < 512 -- but the split keeps the
  // codec correct if a future type carries more values). Returns false on any
  // encode/transport failure. Sending with zero staged entries sends an empty
  // snapshot (entry_count=0), a legitimate "nothing to report" state.
  // Does NOT clear staged entries (call begin() to reset).
  // 中文：把暫存快照編碼送出；恆為單 frame，但保留分批能力以防未來值集變大。
  bool send();

  std::size_t entry_count() const { return entry_count_; }

 private:
  struct Entry {
    std::uint8_t port;
    DeviceType observed_type;
    std::uint8_t value_count;
    float values[kMaxDeviceStatusValues];
  };

  bool send_entries(std::size_t begin_idx, std::size_t end_idx);

  ITransport& transport_;
  Entry entries_[kMaxDeviceStatusEntries];
  std::size_t entry_count_ = 0;
};

}  // namespace vexdash
