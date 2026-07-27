#include "vexdash_pros/device_scanner.h"

#include <cstring>

// Host-testable (NO PROS headers, see device_scanner.h / lib-pros/CLAUDE.md).
// Exercised by tests/test_device_scanner.cpp on the host.

namespace vexdash {

namespace {
// Mirror of pros::c::v5_device_e_t values (DPLIB pros/device.h:48-63). Kept as
// plain ints so this file needs no PROS headers. If PROS renumbers these on a
// kernel upgrade, re-verify against that header.
// 中文：PROS 裝置型別枚舉值的鏡像（來源見上行），用純 int 避免 include PROS 標頭。
constexpr int kProsNone = 0;         // E_DEVICE_NONE
constexpr int kProsMotor = 2;        // E_DEVICE_MOTOR
constexpr int kProsRotation = 4;     // E_DEVICE_ROTATION
constexpr int kProsImu = 6;          // E_DEVICE_IMU
constexpr int kProsDistance = 7;     // E_DEVICE_DISTANCE
constexpr int kProsRadio = 8;        // E_DEVICE_RADIO   (no protocol type)
constexpr int kProsVision = 11;      // E_DEVICE_VISION
constexpr int kProsAdi = 12;         // E_DEVICE_ADI     (ADI expander)
constexpr int kProsOptical = 16;     // E_DEVICE_OPTICAL
constexpr int kProsGps = 20;         // E_DEVICE_GPS
constexpr int kProsAiVision = 29;    // E_DEVICE_AIVISION
constexpr int kProsSerial = 129;     // E_DEVICE_SERIAL  (generic; no protocol type)
constexpr int kProsUndefined = 255;  // E_DEVICE_UNDEFINED
}  // namespace

DeviceType DeviceScanner::map_pros_type(int pros_type) {
  switch (pros_type) {
    case kProsNone:
      return DeviceType::kEmpty;
    case kProsMotor:
      return DeviceType::kMotor;
    case kProsRotation:
      return DeviceType::kRotation;
    case kProsImu:
      return DeviceType::kImu;
    case kProsDistance:
      return DeviceType::kDistance;
    case kProsVision:
      return DeviceType::kVision;
    case kProsAdi:
      return DeviceType::kAdiExpander;
    case kProsOptical:
      return DeviceType::kOptical;
    case kProsGps:
      return DeviceType::kGps;
    case kProsAiVision:
      // AI Vision has no dedicated protocol type; it IS a vision-family sensor,
      // so map to kVision (closest honest match; front-end shows "Vision").
      // 中文：AI Vision 協定沒有專屬型別，本質是視覺感測器，歸到 Vision 最誠實。
      return DeviceType::kVision;
    case kProsUndefined:
      // Not a valid device -> treat as empty. 中文：無效裝置，當成空。
      return DeviceType::kEmpty;
    case kProsRadio:   // VEXnet radio -- present but no protocol type.
    case kProsSerial:  // generic serial -- present but no protocol type.
    default:
      // Detected-but-unmapped: honestly kUnknown, never guessed (§5.12: 0xFE).
      // 中文：偵測到但對不上，誠實標未知，不亂猜。
      return DeviceType::kUnknown;
  }
}

bool DeviceScanner::pros_type_connected(int pros_type) {
  return pros_type != kProsNone && pros_type != kProsUndefined;
}

void DeviceScanner::reset() {
  decl_count_ = 0;
  for (std::uint8_t i = 0; i < kSmartPorts; ++i) {
    slots_[i].type = DeviceType::kEmpty;
    slots_[i].connected = false;
  }
  primed_ = false;
}

bool DeviceScanner::is_declared(std::uint8_t wire_port) const {
  for (std::size_t i = 0; i < decl_count_; ++i) {
    if (decls_[i].wire_port == wire_port) return true;
  }
  return false;
}

bool DeviceScanner::declare(std::uint8_t wire_port, DeviceType type, const char* name) {
  // protocol.md §5.12 port numbering: 1..21 smart, 22..29 ADI. Reject 0 and >29.
  if (wire_port < 1 || wire_port > 29) return false;
  if (name == nullptr) name = "";
  const std::size_t name_len = std::strlen(name);
  if (name_len > kMaxDeviceNameLen) return false;

  // Overwrite an existing declaration for the same port, else append.
  Decl* slot = nullptr;
  for (std::size_t i = 0; i < decl_count_; ++i) {
    if (decls_[i].wire_port == wire_port) {
      slot = &decls_[i];
      break;
    }
  }
  if (slot == nullptr) {
    if (decl_count_ >= kMaxDecls) return false;
    slot = &decls_[decl_count_++];
  }
  slot->wire_port = wire_port;
  slot->type = type;
  std::memcpy(slot->name, name, name_len);
  slot->name[name_len] = '\0';
  return true;
}

bool DeviceScanner::scan(PortReader reader, void* user) {
  bool changed = !primed_;  // first ever scan always emits an initial snapshot
  for (std::uint8_t i = 0; i < kSmartPorts; ++i) {
    const int raw = (reader != nullptr) ? reader(i, user) : kProsNone;
    const DeviceType t = map_pros_type(raw);
    const bool c = pros_type_connected(raw);
    if (slots_[i].type != t || slots_[i].connected != c) changed = true;
    slots_[i].type = t;
    slots_[i].connected = c;
  }
  // NOTE: ADI (3-wire) ports 22..29 are NOT scanned -- analog/digital lines
  // carry no device identity, so PROS cannot detect what (if anything) is on
  // them. A declared ADI device is therefore reported as present on trust (see
  // send()); we never claim to have physically verified an ADI plug.
  // 中文：ADI 三線埠物理上無法偵測，宣告的 ADI 裝置一律「照宣告當作在」（見 send），
  // 絕不假裝偵測得到——限制在協定與回報都寫明。
  primed_ = true;
  return changed;
}

bool DeviceScanner::send(DeviceMap& dm) const {
  dm.begin();

  // 1) Declared devices -- ALWAYS emitted (even when missing) so the dashboard
  //    can flag anomalies. device_type = the EXPECTED type; connected = "the
  //    right device is actually plugged in". For a smart port that means a
  //    device is present AND its detected type matches the declaration -- so
  //    both "nothing plugged" and "wrong device" collapse to connected=false
  //    (an anomaly the coach must fix). ADI ports can't be scanned, so a
  //    declared ADI device is reported connected=true on trust (unverifiable).
  //    中文：宣告項一律送；型別＝期望型別，connected＝「有沒有正確插好」（有插且
  //    型別對）。沒插或型別不對都是 connected=false（異常）。ADI 無法掃，照宣告當在。
  for (std::size_t i = 0; i < decl_count_; ++i) {
    const Decl& d = decls_[i];
    bool connected;
    if (d.wire_port >= 1 && d.wire_port <= kSmartPorts) {
      const Slot& s = slots_[d.wire_port - 1];
      connected = s.connected && (s.type == d.type);
    } else {
      connected = true;  // ADI: unverifiable, trust the declaration.
    }
    dm.add_port(d.wire_port, d.type, connected, d.name);
  }

  // 2) Detected-but-UNDECLARED smart ports -- emitted with an EMPTY name so the
  //    front-end can distinguish a surprise device from a declared one. Ports
  //    that are both undeclared and empty are skipped entirely (nothing to
  //    show). 中文：掃到但沒宣告的埠，名字留空讓前端能區分「意外裝置」；沒宣告又
  //    沒插的埠直接不送。
  for (std::uint8_t i = 0; i < kSmartPorts; ++i) {
    const std::uint8_t wire = static_cast<std::uint8_t>(i + 1);
    if (slots_[i].connected && !is_declared(wire)) {
      dm.add_port(wire, slots_[i].type, true, "");
    }
  }

  return dm.send();
}

bool DeviceScanner::send_status(DeviceStatus& ds, ValueReader reader, void* user,
                                bool with_battery) const {
  ds.begin();
  if (reader == nullptr) return ds.send();  // nothing to read -> empty snapshot

  // WS10-D: the Brain battery (§5.13, wire port 0) is always present and NOT
  // declaration-gated -- stage it first (before the declared smart-port loop) so
  // the dashboard's battery alerts (low-capacity / voltage-sag) always have real
  // data. Reuses the same value-reader seam. If the reader returns a count other
  // than battery's (4), the entry is simply skipped (add_entry validates it).
  // 中文：電池永遠在、不靠宣告，第一筆就送（走同一個讀值接縫）；讀不到正確數量就跳過。
  if (with_battery) {
    float batt[kMaxDeviceStatusValues];
    const std::uint8_t bn = reader(0, DeviceType::kBattery, batt, kMaxDeviceStatusValues, user);
    if (bn == device_status_value_count(DeviceType::kBattery)) {
      ds.add_entry(0, DeviceType::kBattery, batt, bn);
    }
  }

  // Only DECLARED devices are candidates (§5.13: declared AND detected). Walk
  // the declaration table; for each declared smart port that is actually
  // connected, emit its ACTUAL type's value set. 中文：只走宣告表；宣告的 smart
  // port 若真的偵測到，就用「實測型別」的值集送出。
  for (std::size_t i = 0; i < decl_count_; ++i) {
    const Decl& d = decls_[i];

    // ADI ports (22..29) can't be scanned/read for these value sets -> skip.
    // 中文：ADI（3-wire）讀不到這些型別的標準值，略過。
    if (d.wire_port < 1 || d.wire_port > kSmartPorts) continue;

    const Slot& s = slots_[d.wire_port - 1];
    // Must be actually detected (something present). "Declared but not plugged"
    // sends no value (§5.13). 中文：沒偵測到就不發值。
    if (!s.connected) continue;

    // observed_type = the ACTUAL detected type (may differ from the declared
    // type). If it has no defined value set, skip. 中文：實測型別；無值集則略過。
    const DeviceType observed = s.type;
    const std::uint8_t expected = device_status_value_count(observed);
    if (expected == 0) continue;

    float values[kMaxDeviceStatusValues];
    const std::uint8_t n = reader(d.wire_port, observed, values, kMaxDeviceStatusValues, user);
    // A read failure / type mismatch (n != expected) skips this device rather
    // than emitting a half-filled or contract-violating entry. 中文：讀取失敗或
    // 數量不符就跳過，不送半套值。
    if (n != expected) continue;

    ds.add_entry(d.wire_port, observed, values, n);
  }

  return ds.send();
}

}  // namespace vexdash
