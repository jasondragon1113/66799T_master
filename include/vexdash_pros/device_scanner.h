#pragma once

#include <cstddef>
#include <cstdint>

#include "vexdash/device_map.h"
#include "vexdash/device_status.h"
#include "vexdash/protocol_types.h"

// DeviceScanner -- keeps the DEVICE_MAP frame (0x0D, protocol.md §5.12) in
// sync with reality on a competition robot. The mental model is
// DECLARED-vs-ACTUAL:
//
//   * The student DECLARES the devices their program uses, one line each:
//       scanner.declare(3, DeviceType::kMotor, "left_drive");
//     Declared devices are the PRIMARY data -- they carry the human name and
//     are ALWAYS sent, even when nothing is plugged in (connected=false), so
//     the dashboard can flag "declared but not wired / wrong type" anomalies.
//   * A background poll then fills in the ACTUAL state of each declared port
//     (is the right device really plugged in?) by scanning the smart ports.
//   * Devices detected but NOT declared are also sent, with an EMPTY name so
//     the front-end can tell them apart from declared ones (a surprise device
//     on a port nobody declared).
//   * Ports that were never declared AND have nothing plugged are omitted --
//     the panel only shows what matters.
//
// 中文：比賽場景的心智模型是「宣告 vs 實際」。學生一行一裝置宣告程式用到的埠（帶
// 名字）——宣告項是主資料，就算沒插到也照樣送（connected=false），讓 dashboard 標出
// 「宣告了卻沒插好／型別不對」的異常。背景掃描負責填每個宣告埠的實際狀態。掃到但
// 沒宣告的裝置也送，但名字留空讓前端能區分。從頭到尾沒宣告又沒插的埠就不送。
//
// DELIBERATELY NO PROS HEADERS here so this stays host-testable (same
// isolation rule as ConnectionPump / WatchRegistry, see lib-pros/CLAUDE.md).
// The real per-port PROS call (pros::c::registry_get_plugged_type) is injected
// through the PortReader seam so host tests stub it.

namespace vexdash {

class DeviceScanner {
 public:
  static constexpr std::uint8_t kSmartPorts = 21;   // V5 smart ports (wire 1..21)
  static constexpr std::uint8_t kMaxDecls = 29;     // one per possible wire port (1..29)

  // reader(pros_port_0based, user) returns a raw pros::c::v5_device_e_t value
  // (as int) for smart-port index 0..20. PROS ports are ZERO-indexed 0-20
  // (DPLIB pros/apix.h:624,647); the wire uses 1..21, so wire_port =
  // pros_port + 1. 中文：PROS 埠 0~20，協定線上 1~21，要 +1。
  using PortReader = int (*)(std::uint8_t pros_port_0based, void* user);

  // ValueReader seam for DEVICE_STATUS (protocol.md §5.13, ticket WS10-A). Reads
  // the per-type standard value set for the device on `wire_port` (1..21) whose
  // ACTUAL detected type is `observed_type`, writing up to `max_out` floats into
  // `out` in the §5.13 canonical order, and returns how many it wrote. Must
  // return device_status_value_count(observed_type) on success, or 0 on a read
  // failure / unsupported type (that device is then skipped). Kept as a plain
  // function-pointer seam with NO PROS types in the signature so send_status()
  // stays host-testable; the real reader (which calls pros::c::motor_get_* etc.)
  // lives in a PROS-only source. 中文：DEVICE_STATUS 的值讀取接縫——依實測型別讀
  // 該型別標準值集，回傳寫入幾個 float。純函式指標、簽名不含 PROS 型別，故 host 可測。
  using ValueReader = std::uint8_t (*)(std::uint8_t wire_port, DeviceType observed_type, float* out,
                                       std::uint8_t max_out, void* user);

  DeviceScanner() { reset(); }

  // Declares one device the program expects. `wire_port` is the protocol.md
  // §5.12 number: 1..21 for smart ports, 22..29 for ADI A..H (use adi_port()).
  // `type` is the EXPECTED device type; `name` is the display name (<=63 bytes,
  // "" allowed). Re-declaring the same port overwrites. Returns false if the
  // port is out of range (1..29), the name is too long, or the table is full.
  // 中文：宣告一個程式會用到的裝置（一行一個）。埠號 1~21 智慧埠、22~29 ADI；type 是
  // 期望型別、name 是顯示名。重複宣告同埠會覆蓋。回傳 false＝埠越界/名字過長/表滿。
  bool declare(std::uint8_t wire_port, DeviceType type, const char* name = "");

  // Scans all 21 smart ports through `reader`, refreshes the actual-state
  // snapshot, and returns true iff the actual plugged-in set changed since the
  // previous scan (change-on-event, §5.12 -- the caller only re-sends
  // DEVICE_MAP when something actually plugged/unplugged, never every tick).
  // The very first scan always reports changed (primes the snapshot).
  // 中文：掃 21 個智慧埠、更新實際狀態；有插拔才回 true（第一次一定回 true）。
  bool scan(PortReader reader, void* user);

  // Emits the current view as DEVICE_MAP: every declared device (always, with
  // its name and connected = "the declared device is actually plugged in with
  // the matching type") + every detected-but-undeclared smart port (name="").
  // Undeclared empty ports are skipped. Returns DeviceMap::send()'s result.
  // 中文：把宣告項（一律送，帶名字與「有沒有正確插好」）＋掃到但沒宣告的埠（名字空）
  // 送成 DEVICE_MAP；沒宣告又沒插的埠略過。
  bool send(DeviceMap& dm) const;

  // Emits DEVICE_STATUS (0x0E, protocol.md §5.13): the per-type live values of
  // every device that is DECLARED **and** actually DETECTED **and** whose actual
  // type has a defined value set. For each such smart port it emits one entry
  // with observed_type = the ACTUAL detected type (not the declared one, so a
  // "declared motor, plugged rotation" shows the rotation value set) and the
  // values read via `reader`. Declared-but-unplugged ports, ADI ports (can't be
  // read), undeclared surprise devices, and types with no value set are all
  // skipped (§5.13). Intended to run on the same ~1Hz hook as scan()/DEVICE_MAP,
  // but unlike DEVICE_MAP it sends every period (live values always change).
  // Returns DeviceStatus::send()'s result. Call scan() first so the actual-state
  // snapshot is fresh. 中文：把「宣告且偵測到且型別有值集」的裝置的即時值送成
  // DEVICE_STATUS；observed_type 用實測型別。沒插/ADI/未宣告/型別無值集都略過。
  //
  // WS10-D: `with_battery`. The Brain's battery (protocol.md §5.13, wire port 0)
  // is NOT a smart port and NOT declaration-gated -- it is always physically
  // present -- so when `with_battery` is true, one BATTERY entry (port 0) is
  // ALWAYS staged first, reading its 4-value set (voltage/current/capacity/temp)
  // through the SAME `reader` seam as reader(0, DeviceType::kBattery, ...). It is
  // emitted regardless of declarations; a reader that returns a wrong count for
  // battery skips just the battery entry. Default false keeps every pre-WS10-D
  // caller/test byte-identical. 中文：電池永遠在（不是 smart port、不靠宣告），
  // with_battery=true 時一律先送一筆 port 0 BATTERY entry，走同一個 reader seam
  // 以 reader(0, kBattery, ...) 讀 4 值；預設 false 讓舊呼叫端/測試位元不變。
  bool send_status(DeviceStatus& ds, ValueReader reader, void* user,
                   bool with_battery = false) const;

  // ---- Pure mapping helpers (host-tested) --------------------------------
  // Maps a raw pros::c::v5_device_e_t value (DPLIB pros/device.h:48-63) to the
  // wire DeviceType (protocol.md §5.12). Valid PROS types with no protocol
  // equivalent (radio / generic serial) honestly become kUnknown -- never
  // guessed. 中文：PROS 型別對映成協定型別；對不上的誠實標未知，不亂猜。
  static DeviceType map_pros_type(int pros_type);
  // True if a device is present on the port (anything other than NONE /
  // UNDEFINED). 中文：埠上有沒有東西。
  static bool pros_type_connected(int pros_type);

  // Clears declarations + snapshot; next scan reports changed. Test/re-init.
  void reset();

  std::size_t decl_count() const { return decl_count_; }

 private:
  struct Decl {
    std::uint8_t wire_port;  // 1..29
    DeviceType type;         // expected type
    char name[kMaxDeviceNameLen + 1];
  };
  struct Slot {
    DeviceType type;  // ACTUAL mapped type detected on this smart port
    bool connected;   // ACTUAL: something present on this smart port
  };

  bool is_declared(std::uint8_t wire_port) const;

  Decl decls_[kMaxDecls];
  std::size_t decl_count_ = 0;
  Slot slots_[kSmartPorts];  // actual smart-port state (index i == wire port i+1)
  bool primed_ = false;      // false until the first scan; first scan => changed
};

}  // namespace vexdash
