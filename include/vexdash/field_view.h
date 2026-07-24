#pragma once

#include <cstdint>

#include "vexdash/protocol_types.h"
#include "vexdash/transport.h"

// FIELD_OPS (0x04) field-map drawing ops, per protocol.md §5.4: SET_POSE,
// POLYLINE, CIRCLE, CLEAR. Each op is sent as its own frame (v1 does not
// batch multiple ops per frame).
//
// No dynamic allocation: polyline() takes a caller-owned point array and
// a compile-time-bounded max point count per call (kMaxPolylinePoints);
// longer paths must be sent across multiple polyline() calls by the
// caller (protocol.md §5.4 explicitly assigns this responsibility to the
// robot side, since v1 has no cross-frame append semantics).

namespace vexdash {

// protocol.md §5.4: (512 - 1 - 2 - 1 - 2) / 16 ~= 31 points/frame.
constexpr std::size_t kMaxPolylinePoints = 31;

struct FieldPoint {
  double x_mm;
  double y_mm;
};

class FieldView {
 public:
  explicit FieldView(ITransport& transport);

  // SET_POSE: robot's pose (position + heading) on the field map.
  bool set_pose(double x_mm, double y_mm, double heading_rad);

  // POLYLINE: a complete, independent path (not appended to any previous
  // polyline -- see protocol.md §5.4 [LOW-CONFIDENCE] note). `point_count`
  // must be <= kMaxPolylinePoints; returns false otherwise.
  bool polyline(const FieldPoint* points, std::size_t point_count);

  // CIRCLE: a circle (e.g. to highlight a target zone).
  bool circle(double x_mm, double y_mm, double radius_mm);

  // CLEAR: erase everything currently drawn on the front-end field map.
  bool clear();

 private:
  ITransport& transport_;
};

}  // namespace vexdash
