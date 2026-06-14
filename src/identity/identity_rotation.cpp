#include "mith/identity/identity_rotation.h"

#include <cstring>

namespace mith {

void serialise_cert_payload(const HierarchicalID& prev_id,
                            const HierarchicalID& new_id,
                            const IdentityKey&    new_key,
                            std::uint8_t          out[IDENTITY_CERT_PAYLOAD_LEN]) noexcept {
    std::size_t off = 0;

    // prev_id.swarm_id (LE)
    out[off++] = static_cast<std::uint8_t>(prev_id.swarm_id & 0xFFu);
    out[off++] = static_cast<std::uint8_t>((prev_id.swarm_id >> 8) & 0xFFu);
    // prev_id.unit_id (16 raw bytes)
    std::memcpy(out + off, prev_id.unit_id.bytes().data(), 16);
    off += 16;

    // new_id.swarm_id (LE)
    out[off++] = static_cast<std::uint8_t>(new_id.swarm_id & 0xFFu);
    out[off++] = static_cast<std::uint8_t>((new_id.swarm_id >> 8) & 0xFFu);
    // new_id.unit_id (16 raw bytes)
    std::memcpy(out + off, new_id.unit_id.bytes().data(), 16);
    off += 16;

    // new_key.public_key (32 bytes)
    std::memcpy(out + off, new_key.public_key.data(), 32);
}

} // namespace mith
