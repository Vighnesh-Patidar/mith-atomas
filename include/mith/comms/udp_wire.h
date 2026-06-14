#pragma once

// udp_wire — see ARCHITECTURE.md §7.5
//
// On-the-wire encoding for UDPMulticastTransport (§7.5, v0.2). Pure
// serialisation — no socket calls, fully unit-testable. Cross-arch safe:
// all integers and floats are written little-endian via byte packing, no
// raw memcpy of structs.
//
// Frame format (every datagram):
//
//   offset  size  field
//   0       1     tag                     0x01 = beacon, 0x02 = message
//   1       4     payload_length (LE)     bytes following this header
//   5       N     payload                 schema below
//
// Beacon payload (StateVector):
//   2     swarm_id      LE
//   16    unit_id       raw bytes
//   12    position      x,y,z   float LE
//   12    velocity      vx,vy,vz float LE
//   1     health.value  u8
//   4     role          LE
//   4     state         LE
//   4     tick          LE
//   32    sender_pubkey raw bytes (zero in unsigned mode)
//   64    signature     raw bytes (zero in unsigned mode)
//   = 151 bytes
//
// Bytes 0..86 of the payload (everything up to and including
// sender_pubkey) are the SIGNED bytes — what sender Ed25519-signs and
// receiver verifies. See serialise_beacon_signed_payload() below.
//
// Message payload (Message):
//   2+16  sender    swarm_id LE + unit_id
//   2+16  recipient swarm_id LE + unit_id
//   4     type      LE
//   4     seq       LE
//   4     timestamp float LE
//   128   payload   raw bytes
//   = 176 bytes
//
// Magic / version: not in v0.2 — the tag byte doubles as a smoke check.
// Future v0.3 may prepend a 4-byte magic for cross-version detection.

#include "mith/comms/message.h"
#include "mith/comms/state_vector.h"

#include <cstdint>
#include <cstddef>
#include <optional>

namespace mith::udp_wire {

inline constexpr std::uint8_t TAG_BEACON  = 0x01;
inline constexpr std::uint8_t TAG_MESSAGE = 0x02;

inline constexpr std::size_t  FRAME_HEADER_BYTES        = 5;     // tag + length
inline constexpr std::size_t  BEACON_PAYLOAD_BYTES       = 151;
inline constexpr std::size_t  BEACON_SIGNED_PREFIX_BYTES = 87;    // bytes Ed25519-signs
inline constexpr std::size_t  MESSAGE_PAYLOAD_BYTES      = 176;
inline constexpr std::size_t  MAX_FRAME_BYTES =
    FRAME_HEADER_BYTES + MESSAGE_PAYLOAD_BYTES;                   // <= MTU

// Encode functions write a complete frame (header + payload) into `out`
// and return the number of bytes written. Returns 0 if `cap` is too small.
std::size_t encode_beacon(const StateVector& sv,
                          std::uint8_t* out, std::size_t cap) noexcept;
std::size_t encode_message(const Message& msg,
                           std::uint8_t* out, std::size_t cap) noexcept;

// Decode functions parse a complete frame (header + payload). Return
// nullopt if the buffer is too short, the tag is wrong for the expected
// type, or the length field disagrees with the schema's fixed size.
std::optional<StateVector> decode_beacon(const std::uint8_t* in,
                                         std::size_t len) noexcept;
std::optional<Message>     decode_message(const std::uint8_t* in,
                                          std::size_t len) noexcept;

// Read the first byte of a frame WITHOUT consuming it. Returns 0 (invalid)
// if `len` is zero. Use to route an incoming datagram into beacon/message
// inboxes before full decode.
std::uint8_t peek_tag(const std::uint8_t* in, std::size_t len) noexcept;

// Serialise the leading 87 bytes of the beacon payload — every field
// except the trailing 64-byte signature. This is the byte string Ed25519
// signs (signed mode) and verifies (signed mode). Returns
// BEACON_SIGNED_PREFIX_BYTES on success, 0 on insufficient capacity.
std::size_t serialise_beacon_signed_payload(const StateVector& sv,
                                            std::uint8_t* out,
                                            std::size_t cap) noexcept;

} // namespace mith::udp_wire
