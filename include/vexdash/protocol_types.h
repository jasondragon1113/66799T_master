#pragma once

#include <cstdint>

// Wire-level enums and constants shared across all vexdash message codecs.
// Values are pinned exactly to docs/protocol.md -- do not renumber.

namespace vexdash {

enum class MsgType : std::uint8_t {
  kHello = 0x01,
  kTelemetry = 0x02,
  kChannelDef = 0x03,
  kFieldOps = 0x04,
  kLog = 0x05,
  kConfigSchema = 0x06,
  kConfigSet = 0x07,
  kPing = 0x08,
  kPong = 0x09,
  kCmdDef = 0x0A,
  kCommand = 0x0B,
  kTelemetryText = 0x0C,  // v1.1: string live-value channel (protocol.md §5.11)
  kDeviceMap = 0x0D,      // v1.1: per-smart-port device map (protocol.md §5.12)
  kDeviceStatus = 0x0E,   // v1.3: per-device live values (protocol.md §5.13)
};

// protocol.md §5.2/§5.3/§5.6/§5.9/§5.10: value_type tag used by TELEMETRY
// samples, CHANNEL_DEF, CONFIG_SCHEMA, CMD_DEF params and CONFIG_SET.
//
// v1.1 (protocol.md §5.3) adds two CHANNEL_DEF-only value types:
//   kString: live value is text, carried by TELEMETRY_TEXT (§5.11), NOT by
//            the numeric TELEMETRY path -- it has no fixed inline wire size.
//   kEnum:   live value is an integer index carried in a numeric TELEMETRY
//            sample as i32 (0x02); the enum labels come from CHANNEL_DEF's
//            enum_labels. So an ENUM channel's *sample* value_type on the
//            wire is still kI32, and value_type_size(kEnum) is defined as 4
//            to reflect that inline-sample width.
enum class ValueType : std::uint8_t {
  kF64 = 0x01,
  kI32 = 0x02,
  kBool = 0x03,
  kString = 0x04,  // v1.1, CHANNEL_DEF only
  kEnum = 0x05,    // v1.1, CHANNEL_DEF only (samples travel as i32)
};

// protocol.md §5.4: FIELD_OPS op_type.
enum class FieldOpType : std::uint8_t {
  kSetPose = 0x01,
  kPolyline = 0x02,
  kCircle = 0x03,
  kClear = 0x04,
};

// protocol.md §5.5: LOG level.
enum class LogLevel : std::uint8_t {
  kDebug = 0x01,
  kInfo = 0x02,
  kWarn = 0x03,
  kError = 0x04,
};

// protocol.md §5.1: HELLO role.
enum class Role : std::uint8_t {
  kRobot = 0x01,
  kPc = 0x02,
};

// protocol.md §2.2: frame payload (msg_type + payload + crc16) upper bound
// before COBS encoding.
constexpr std::size_t kMaxFrameSize = 512;

// protocol.md §5.2 (v1.2): each TELEMETRY frame carries an optional trailing
// u16 sequence number (little-endian) appended AFTER the samples, so the
// dashboard can measure real packet loss. A pre-v1.2 decoder that stops after
// sample_count samples ignores it (frame boundary is COBS-delimited, §6.5).
constexpr std::size_t kTelemetrySeqLen = 2;

// protocol.md §2.3: accumulation buffer upper bound (COBS-encoded, before
// the trailing 0x00 delimiter is stripped), chosen conservatively above
// kMaxFrameSize to allow for COBS overhead.
constexpr std::size_t kMaxAccumBufferSize = 768;

// protocol.md §0: generic string length prefix is u8 (max 255 bytes) unless
// a specific message field overrides it (documented at each field).
constexpr std::size_t kMaxGenericStringLen = 255;

// protocol.md §5.3/§5.6/§5.9: `name`-class fields capped at 63 bytes.
constexpr std::size_t kMaxNameLen = 63;

// protocol.md §5.3/§5.9: `unit` field capped at 15 bytes.
constexpr std::size_t kMaxUnitLen = 15;

// protocol.md §5.3 (v1.1): enum_labels bounds.
constexpr std::size_t kMaxEnumLabels = 64;
constexpr std::size_t kMaxEnumLabelLen = 31;

// protocol.md §5.3 (v1.1): CHANNEL_DEF v11_flags bits.
constexpr std::uint8_t kChannelFlagHasDevicePort = 0x01;
constexpr std::uint8_t kChannelFlagHasEnumLabels = 0x02;

// protocol.md §5.12 (v1.1): DEVICE_MAP entry name cap (63 bytes, same as
// other name-class fields).
constexpr std::size_t kMaxDeviceNameLen = 63;

// protocol.md §5.12 (v1.1): device_type enumeration.
enum class DeviceType : std::uint8_t {
  kEmpty = 0x00,
  kMotor = 0x01,
  kRotation = 0x02,
  kDistance = 0x03,
  kOptical = 0x04,
  kImu = 0x05,
  kVision = 0x06,
  kGps = 0x07,
  kLed = 0x08,
  kAdiExpander = 0x09,
  kElectromagnet = 0x0A,
  kBattery = 0x0B,  // v1.3 (WS10-D): the Brain's internal battery (wire port 0, always present)
  kUnknown = 0xFE,  // detected but type unknown
};

// protocol.md §5.13 (v1.3): DEVICE_STATUS per-type standard value set. Returns
// how many f32 values a DEVICE_STATUS entry carries for a given observed
// DeviceType, in the canonical wire order documented in §5.13:
//   MOTOR   -> 3 : temperature(C), power(W), current(mA)
//   ROTATION-> 2 : angle(centideg, absolute), position(centideg, cumulative)
//   DISTANCE-> 2 : distance(mm), confidence(0-63)
//   OPTICAL -> 3 : hue(0-359.999 deg), proximity(0-255), brightness(0-1.0)
//   IMU     -> 3 : heading([0,360) deg), pitch((-180,180) deg), roll((-180,180) deg)
//   BATTERY -> 4 : voltage(mV), current(mA), capacity(%), temperature(C)  [v1.3, WS10-D]
// Any other type has no defined value set -> 0 (such a device is NOT emitted).
// 中文：某型別在 DEVICE_STATUS 裡帶幾個 f32 值（順序即 §5.13 契約）；對不上的型別回 0＝不發。
constexpr std::uint8_t device_status_value_count(DeviceType t) {
  switch (t) {
    case DeviceType::kMotor:
      return 3;  // temperature, power, current
    case DeviceType::kRotation:
      return 2;  // angle (absolute), position (cumulative)
    case DeviceType::kDistance:
      return 2;  // distance, confidence
    case DeviceType::kOptical:
      return 3;  // hue, proximity, brightness
    case DeviceType::kImu:
      return 3;  // heading, pitch, roll
    case DeviceType::kBattery:
      return 4;  // voltage(mV), current(mA), capacity(%), temperature(C)
    default:
      return 0;  // no standard value set for this type -> not reported
  }
}

// protocol.md §5.13: the largest per-type value_count above (BATTERY carries 4,
// the widest; MOTOR/OPTICAL/IMU carry 3). Lets the PROS-side value reader size
// its output buffer statically. 中文：值集最大長度（電池 4 為最寬），供讀取端靜態配置緩衝。
constexpr std::uint8_t kMaxDeviceStatusValues = 4;

// Returns the inline wire size in bytes of a value of the given ValueType
// as it appears in a numeric TELEMETRY sample / CONFIG value / CMD param
// (f64->8, i32->4, bool->1). kEnum travels as i32 (->4). kString has no
// inline value in those messages (its value goes via TELEMETRY_TEXT), so
// this returns 0 for kString -- callers that need an inline value must
// reject kString before calling.
constexpr std::size_t value_type_size(ValueType vt) {
  switch (vt) {
    case ValueType::kF64:
      return 8;
    case ValueType::kI32:
      return 4;
    case ValueType::kBool:
      return 1;
    case ValueType::kEnum:
      return 4;  // v1.1: enum sample value is an i32 index
    case ValueType::kString:
      return 0;  // v1.1: no inline value; carried by TELEMETRY_TEXT
  }
  return 0;
}

}  // namespace vexdash
