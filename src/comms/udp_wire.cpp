#include "mith/comms/udp_wire.h"

#include <cstring>

namespace mith::udp_wire {

namespace {

// Little-endian byte packing helpers. All offsets bounds-checked by the
// caller.
inline void put_u16(std::uint8_t* p, std::uint16_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v        & 0xFFu);
    p[1] = static_cast<std::uint8_t>((v >> 8) & 0xFFu);
}
inline void put_u32(std::uint8_t* p, std::uint32_t v) noexcept {
    p[0] = static_cast<std::uint8_t>( v        & 0xFFu);
    p[1] = static_cast<std::uint8_t>((v >>  8) & 0xFFu);
    p[2] = static_cast<std::uint8_t>((v >> 16) & 0xFFu);
    p[3] = static_cast<std::uint8_t>((v >> 24) & 0xFFu);
}
inline void put_f32(std::uint8_t* p, float v) noexcept {
    std::uint32_t bits;
    std::memcpy(&bits, &v, sizeof bits);
    put_u32(p, bits);
}
inline std::uint16_t get_u16(const std::uint8_t* p) noexcept {
    return static_cast<std::uint16_t>(p[0])
         | (static_cast<std::uint16_t>(p[1]) << 8);
}
inline std::uint32_t get_u32(const std::uint8_t* p) noexcept {
    return  static_cast<std::uint32_t>(p[0])
         | (static_cast<std::uint32_t>(p[1]) << 8)
         | (static_cast<std::uint32_t>(p[2]) << 16)
         | (static_cast<std::uint32_t>(p[3]) << 24);
}
inline float get_f32(const std::uint8_t* p) noexcept {
    const std::uint32_t bits = get_u32(p);
    float v;
    std::memcpy(&v, &bits, sizeof v);
    return v;
}

void put_hid(std::uint8_t* p, const HierarchicalID& hid) noexcept {
    put_u16(p, hid.swarm_id);
    std::memcpy(p + 2, hid.unit_id.bytes().data(), 16);
}

HierarchicalID get_hid(const std::uint8_t* p) noexcept {
    HierarchicalID hid;
    hid.swarm_id = get_u16(p);
    std::array<std::uint8_t, 16> bytes;
    std::memcpy(bytes.data(), p + 2, 16);
    hid.unit_id = UUID(bytes);
    return hid;
}

} // namespace

std::size_t serialise_beacon_signed_payload(const StateVector& sv,
                                             std::uint8_t* out,
                                             std::size_t cap) noexcept {
    if (cap < BEACON_SIGNED_PREFIX_BYTES) return 0;

    std::uint8_t* p = out;
    put_hid(p, sv.id);                      p += 18;
    put_f32(p, sv.position.x);              p += 4;
    put_f32(p, sv.position.y);              p += 4;
    put_f32(p, sv.position.z);              p += 4;
    put_f32(p, sv.velocity.vx);             p += 4;
    put_f32(p, sv.velocity.vy);             p += 4;
    put_f32(p, sv.velocity.vz);             p += 4;
    *p++ = sv.health.value;
    put_u32(p, sv.role.role);               p += 4;
    put_u32(p, sv.state.state);             p += 4;
    put_u32(p, sv.tick);                    p += 4;
    std::memcpy(p, sv.sender_pubkey.public_key.data(), 32);
    return BEACON_SIGNED_PREFIX_BYTES;
}

std::size_t encode_beacon(const StateVector& sv,
                          std::uint8_t* out, std::size_t cap) noexcept {
    const std::size_t frame_len = FRAME_HEADER_BYTES + BEACON_PAYLOAD_BYTES;
    if (cap < frame_len) return 0;

    out[0] = TAG_BEACON;
    put_u32(out + 1, static_cast<std::uint32_t>(BEACON_PAYLOAD_BYTES));

    // First 87 bytes of payload: serialise_beacon_signed_payload covers
    // everything except the trailing 64-byte signature.
    serialise_beacon_signed_payload(sv, out + FRAME_HEADER_BYTES,
                                     BEACON_SIGNED_PREFIX_BYTES);
    // Trailing 64 bytes: signature (zero-filled in unsigned mode).
    std::memcpy(out + FRAME_HEADER_BYTES + BEACON_SIGNED_PREFIX_BYTES,
                sv.signature.data(), IdentityKey::SIGNATURE_LEN);
    return frame_len;
}

std::size_t encode_message(const Message& msg,
                           std::uint8_t* out, std::size_t cap) noexcept {
    const std::size_t frame_len = FRAME_HEADER_BYTES + MESSAGE_PAYLOAD_BYTES;
    if (cap < frame_len) return 0;

    out[0] = TAG_MESSAGE;
    put_u32(out + 1, static_cast<std::uint32_t>(MESSAGE_PAYLOAD_BYTES));

    std::uint8_t* p = out + FRAME_HEADER_BYTES;
    put_hid(p, msg.sender);                 p += 18;
    put_hid(p, msg.recipient);              p += 18;
    put_u32(p, msg.type);                   p += 4;
    put_u32(p, msg.seq);                    p += 4;
    put_f32(p, msg.timestamp_s);            p += 4;
    std::memcpy(p, msg.payload.data(), Message::PAYLOAD_SIZE);
    return frame_len;
}

std::optional<StateVector> decode_beacon(const std::uint8_t* in,
                                         std::size_t len) noexcept {
    const std::size_t expect = FRAME_HEADER_BYTES + BEACON_PAYLOAD_BYTES;
    if (len < expect)         return std::nullopt;
    if (in[0] != TAG_BEACON)  return std::nullopt;
    if (get_u32(in + 1) != BEACON_PAYLOAD_BYTES) return std::nullopt;

    StateVector sv;
    const std::uint8_t* p = in + FRAME_HEADER_BYTES;
    sv.id            = get_hid(p);          p += 18;
    sv.position.x    = get_f32(p);          p += 4;
    sv.position.y    = get_f32(p);          p += 4;
    sv.position.z    = get_f32(p);          p += 4;
    sv.velocity.vx   = get_f32(p);          p += 4;
    sv.velocity.vy   = get_f32(p);          p += 4;
    sv.velocity.vz   = get_f32(p);          p += 4;
    sv.health.value  = *p++;
    sv.role.role     = get_u32(p);          p += 4;
    sv.state.state   = get_u32(p);          p += 4;
    sv.tick          = get_u32(p);          p += 4;
    std::memcpy(sv.sender_pubkey.public_key.data(), p, 32);     p += 32;
    std::memcpy(sv.signature.data(), p, IdentityKey::SIGNATURE_LEN);
    return sv;
}

std::optional<Message> decode_message(const std::uint8_t* in,
                                      std::size_t len) noexcept {
    const std::size_t expect = FRAME_HEADER_BYTES + MESSAGE_PAYLOAD_BYTES;
    if (len < expect)         return std::nullopt;
    if (in[0] != TAG_MESSAGE) return std::nullopt;
    if (get_u32(in + 1) != MESSAGE_PAYLOAD_BYTES) return std::nullopt;

    Message msg;
    const std::uint8_t* p = in + FRAME_HEADER_BYTES;
    msg.sender       = get_hid(p);          p += 18;
    msg.recipient    = get_hid(p);          p += 18;
    msg.type         = get_u32(p);          p += 4;
    msg.seq          = get_u32(p);          p += 4;
    msg.timestamp_s  = get_f32(p);          p += 4;
    std::memcpy(msg.payload.data(), p, Message::PAYLOAD_SIZE);
    return msg;
}

std::uint8_t peek_tag(const std::uint8_t* in, std::size_t len) noexcept {
    return (len > 0) ? in[0] : std::uint8_t{0};
}

} // namespace mith::udp_wire
