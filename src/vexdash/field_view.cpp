#include "vexdash/field_view.h"

#include "vexdash/byte_writer.h"
#include "vexdash/frame_codec.h"

namespace vexdash {

FieldView::FieldView(ITransport& transport) : transport_(transport) {}

bool FieldView::set_pose(double x_mm, double y_mm, double heading_rad) {
  std::uint8_t payload[1 + 8 + 8 + 8];
  ByteWriter w(payload, sizeof(payload));
  w.write_u8(static_cast<std::uint8_t>(FieldOpType::kSetPose));
  w.write_f64(x_mm);
  w.write_f64(y_mm);
  w.write_f64(heading_rad);
  if (!w.ok()) return false;

  std::uint8_t wire[frame_max_wire_size(1 + sizeof(payload) + 2)];
  std::size_t wire_len = frame_encode(MsgType::kFieldOps, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

bool FieldView::polyline(const FieldPoint* points, std::size_t point_count) {
  if (point_count > kMaxPolylinePoints) return false;
  if (point_count > 0 && points == nullptr) return false;

  std::uint8_t payload[1 + 2 + kMaxPolylinePoints * 16];
  ByteWriter w(payload, sizeof(payload));
  w.write_u8(static_cast<std::uint8_t>(FieldOpType::kPolyline));
  w.write_u16(static_cast<std::uint16_t>(point_count));
  for (std::size_t i = 0; i < point_count; ++i) {
    w.write_f64(points[i].x_mm);
    w.write_f64(points[i].y_mm);
  }
  if (!w.ok()) return false;

  std::uint8_t wire[frame_max_wire_size(1 + sizeof(payload) + 2)];
  std::size_t wire_len = frame_encode(MsgType::kFieldOps, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

bool FieldView::circle(double x_mm, double y_mm, double radius_mm) {
  std::uint8_t payload[1 + 8 + 8 + 8];
  ByteWriter w(payload, sizeof(payload));
  w.write_u8(static_cast<std::uint8_t>(FieldOpType::kCircle));
  w.write_f64(x_mm);
  w.write_f64(y_mm);
  w.write_f64(radius_mm);
  if (!w.ok()) return false;

  std::uint8_t wire[frame_max_wire_size(1 + sizeof(payload) + 2)];
  std::size_t wire_len = frame_encode(MsgType::kFieldOps, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

bool FieldView::clear() {
  std::uint8_t payload[1];
  ByteWriter w(payload, sizeof(payload));
  w.write_u8(static_cast<std::uint8_t>(FieldOpType::kClear));
  if (!w.ok()) return false;

  std::uint8_t wire[frame_max_wire_size(1 + sizeof(payload) + 2)];
  std::size_t wire_len = frame_encode(MsgType::kFieldOps, payload, w.size(), wire, sizeof(wire));
  if (wire_len == 0) return false;
  return transport_.write(wire, wire_len) == wire_len;
}

}  // namespace vexdash
