#pragma once

#include <cstddef>
#include <cstdint>

#include "vexdash/cobs.h"
#include "vexdash/protocol_types.h"

// Frame structure & framing, per docs/protocol.md §2-3:
//
//   raw frame  := msg_type(1) ++ payload(N) ++ crc16_le(2)
//   on wire    := COBS_encode(raw frame) ++ 0x00
//
// Both encoder and decoder are free of dynamic allocation: all buffers are
// fixed-size and supplied by the caller (or embedded as fixed members).

namespace vexdash {

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

// Upper bound on the on-wire size (COBS-encoded + trailing 0x00) for a raw
// frame whose un-encoded size is `raw_frame_len` bytes
// (= 1 + payload_len + 2).
constexpr std::size_t frame_max_wire_size(std::size_t raw_frame_len) {
  return cobs_max_encoded_size(raw_frame_len) + 1;
}

// Encodes one frame (msg_type + payload) into `out_wire` ready to push
// straight to a transport (COBS-encoded, CRC appended, trailing 0x00
// delimiter included). Returns the number of bytes written to `out_wire`,
// or 0 on failure (payload_len too large for kMaxFrameSize, or
// out_wire_capacity insufficient).
//
// `payload` may be nullptr only if payload_len == 0.
std::size_t frame_encode(MsgType msg_type, const std::uint8_t* payload, std::size_t payload_len,
                          std::uint8_t* out_wire, std::size_t out_wire_capacity);

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

// Result of a successfully decoded frame. `payload` points into the
// decoder's internal buffer and is only valid until the next byte is fed
// (i.e. consume it immediately in the callback, or copy it out).
struct DecodedFrame {
  std::uint8_t msg_type_raw;  // raw byte; may not correspond to a known MsgType enumerator
  const std::uint8_t* payload;
  std::size_t payload_len;
};

// Why a C-style callback: lib-core must not allocate, and must stay usable
// on embedded targets without assuming <functional>/std::function support
// or heap-backed closures. `user_data` lets callers attach context without
// a closure allocation.
using FrameCallback = void (*)(const DecodedFrame& frame, void* user_data);

// Incremental COBS+CRC frame decoder implementing docs/protocol.md §2.3
// resync semantics:
//   - Feed bytes one at a time (or in bulk via feed()).
//   - On 0x00, attempt COBS-decode + CRC check of the accumulated buffer;
//     valid frames are reported via the callback, invalid ones are
//     silently dropped (never crash, never propagate garbage).
//   - If the accumulation buffer would overflow before a 0x00 is seen, the
//     buffer is reset (data-stream-corruption case) and the decoder keeps
//     scanning for the next 0x00 -- this is the core resync guarantee, not
//     an error condition callers must handle specially.
//   - Unknown msg_type: CRC is still validated; if valid, the frame is
//     still reported to the callback with the raw msg_type byte (protocol.md
//     §4's "unknown message type -> ignore" policy is a decision for the
//     message-dispatch layer, e.g. Session, not this codec).
//
// No dynamic allocation: the accumulation buffer is a fixed-size member
// (kMaxAccumBufferSize, sized per protocol.md §2.3).
class FrameDecoder {
 public:
  FrameDecoder(FrameCallback callback, void* user_data);

  // Feeds a single byte from the transport into the decoder. May
  // synchronously invoke the callback zero or one time.
  //
  // NOT re-entrant: the callback MUST NOT call feed() again (directly or
  // indirectly). Doing so may overwrite the internal decode buffer that the
  // in-flight DecodedFrame::payload still points into. Callbacks should only
  // consume/copy the frame and return; drive all feeding from a single site
  // (e.g. Session::poll()).
  void feed(std::uint8_t byte);

  // Feeds multiple bytes; equivalent to calling feed() in a loop. May
  // synchronously invoke the callback zero or more times. Same non-re-entrancy
  // rule as the single-byte overload applies.
  void feed(const std::uint8_t* data, std::size_t len);

  // Diagnostics counters (not part of the wire protocol; useful for tests
  // and for a Session-layer health readout). Never reset automatically.
  struct Stats {
    std::uint32_t frames_ok = 0;
    std::uint32_t frames_crc_failed = 0;
    std::uint32_t frames_cobs_failed = 0;
    std::uint32_t frames_too_short = 0;
    std::uint32_t overflow_resyncs = 0;
  };
  const Stats& stats() const { return stats_; }

 private:
  void on_delimiter();
  void reset_accum();

  FrameCallback callback_;
  void* user_data_;

  std::uint8_t accum_[kMaxAccumBufferSize];
  std::size_t accum_len_ = 0;

  // Scratch buffer for the COBS-decoded raw frame (msg_type+payload+crc16).
  // Sized to accept anything a full accum_ buffer could decode to (COBS
  // decoding never expands, so kMaxAccumBufferSize is a safe upper bound).
  std::uint8_t decoded_[kMaxAccumBufferSize];

  Stats stats_;
};

}  // namespace vexdash
