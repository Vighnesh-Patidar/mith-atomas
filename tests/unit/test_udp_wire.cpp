#include "doctest.h"

#include "mith/comms/message.h"
#include "mith/comms/state_vector.h"
#include "mith/comms/udp_wire.h"
#include "mith/identity/hierarchical_id.h"

#include <array>
#include <cstring>
#include <vector>

using mith::HierarchicalID;
using mith::Message;
using mith::StateVector;
using mith::SwarmID;
namespace uw = mith::udp_wire;

namespace {

StateVector make_sv() {
    StateVector sv;
    sv.id            = HierarchicalID::generate(SwarmID{7});
    sv.position.x    = 1.5f;
    sv.position.y    = -2.25f;
    sv.position.z    = 100.125f;
    sv.velocity.vx   = 0.5f;
    sv.velocity.vy   = -0.5f;
    sv.velocity.vz   = 12.0f;
    sv.health.value  = 87;
    sv.role.role     = 0x1234'5678u;
    sv.state.state   = 0xABCD'EF01u;
    sv.tick          = 42'000u;
    return sv;
}

Message make_msg() {
    Message m;
    m.sender    = HierarchicalID::generate(SwarmID{3});
    m.recipient = HierarchicalID::generate(SwarmID{3});
    m.type      = 99u;
    m.seq       = 1234u;
    m.timestamp_s = 7.5f;
    for (std::size_t i = 0; i < m.payload.size(); ++i) {
        m.payload[i] = static_cast<std::uint8_t>(i * 3u + 1u);
    }
    return m;
}

bool sv_equals(const StateVector& a, const StateVector& b) {
    return a.id == b.id
        && a.position.x == b.position.x
        && a.position.y == b.position.y
        && a.position.z == b.position.z
        && a.velocity.vx == b.velocity.vx
        && a.velocity.vy == b.velocity.vy
        && a.velocity.vz == b.velocity.vz
        && a.health.value == b.health.value
        && a.role.role == b.role.role
        && a.state.state == b.state.state
        && a.tick == b.tick;
}

bool msg_equals(const Message& a, const Message& b) {
    return a.sender == b.sender
        && a.recipient == b.recipient
        && a.type == b.type
        && a.seq == b.seq
        && a.timestamp_s == b.timestamp_s
        && std::memcmp(a.payload.data(), b.payload.data(), Message::PAYLOAD_SIZE) == 0;
}

} // namespace

TEST_CASE("udp_wire: frame sizes match the documented schema") {
    CHECK(uw::FRAME_HEADER_BYTES        == 5u);
    CHECK(uw::BEACON_PAYLOAD_BYTES       == 151u);
    CHECK(uw::BEACON_SIGNED_PREFIX_BYTES == 87u);
    CHECK(uw::MESSAGE_PAYLOAD_BYTES      == 176u);
    CHECK(uw::MAX_FRAME_BYTES      == uw::FRAME_HEADER_BYTES + uw::MESSAGE_PAYLOAD_BYTES);
}

TEST_CASE("udp_wire: encode_beacon then decode_beacon round-trips losslessly") {
    const auto sv = make_sv();
    std::uint8_t buf[uw::MAX_FRAME_BYTES];

    const std::size_t n = uw::encode_beacon(sv, buf, sizeof buf);
    CHECK(n == uw::FRAME_HEADER_BYTES + uw::BEACON_PAYLOAD_BYTES);

    const auto decoded = uw::decode_beacon(buf, n);
    REQUIRE(decoded.has_value());
    CHECK(sv_equals(sv, *decoded));
}

TEST_CASE("udp_wire: encode_message then decode_message round-trips losslessly") {
    const auto msg = make_msg();
    std::uint8_t buf[uw::MAX_FRAME_BYTES];

    const std::size_t n = uw::encode_message(msg, buf, sizeof buf);
    CHECK(n == uw::FRAME_HEADER_BYTES + uw::MESSAGE_PAYLOAD_BYTES);

    const auto decoded = uw::decode_message(buf, n);
    REQUIRE(decoded.has_value());
    CHECK(msg_equals(msg, *decoded));
}

TEST_CASE("udp_wire: encode returns 0 on insufficient capacity") {
    StateVector sv;
    std::uint8_t tiny[10];
    CHECK(uw::encode_beacon(sv, tiny, sizeof tiny) == 0);

    Message msg;
    std::uint8_t small[100];
    CHECK(uw::encode_message(msg, small, sizeof small) == 0);
}

TEST_CASE("udp_wire: peek_tag returns 0 on empty input") {
    std::uint8_t b[1] = {0};
    CHECK(uw::peek_tag(b, 0) == 0u);
}

TEST_CASE("udp_wire: peek_tag returns the first byte of a valid frame") {
    const auto sv = make_sv();
    std::uint8_t buf[uw::MAX_FRAME_BYTES];
    const auto n = uw::encode_beacon(sv, buf, sizeof buf);
    REQUIRE(n > 0);
    CHECK(uw::peek_tag(buf, n) == uw::TAG_BEACON);

    const auto msg = make_msg();
    const auto m = uw::encode_message(msg, buf, sizeof buf);
    REQUIRE(m > 0);
    CHECK(uw::peek_tag(buf, m) == uw::TAG_MESSAGE);
}

TEST_CASE("udp_wire: decode_beacon rejects a message-tagged frame and vice versa") {
    const auto msg = make_msg();
    std::uint8_t buf[uw::MAX_FRAME_BYTES];
    const auto n = uw::encode_message(msg, buf, sizeof buf);
    REQUIRE(n > 0);

    CHECK_FALSE(uw::decode_beacon(buf, n).has_value());

    const auto sv = make_sv();
    const auto b = uw::encode_beacon(sv, buf, sizeof buf);
    REQUIRE(b > 0);
    CHECK_FALSE(uw::decode_message(buf, b).has_value());
}

TEST_CASE("udp_wire: decode rejects short buffers") {
    CHECK_FALSE(uw::decode_beacon(nullptr, 0).has_value());

    std::uint8_t buf[uw::MAX_FRAME_BYTES];
    const auto n = uw::encode_beacon(make_sv(), buf, sizeof buf);
    CHECK_FALSE(uw::decode_beacon(buf, n - 1).has_value());
    CHECK_FALSE(uw::decode_message(buf, n - 1).has_value());
}

TEST_CASE("udp_wire: payload-length tampering invalidates the frame") {
    std::uint8_t buf[uw::MAX_FRAME_BYTES];
    const auto n = uw::encode_beacon(make_sv(), buf, sizeof buf);
    REQUIRE(n > 0);

    // Corrupt the payload-length field (offset 1..4).
    buf[1] ^= 0xFFu;
    CHECK_FALSE(uw::decode_beacon(buf, n).has_value());
}

TEST_CASE("udp_wire: serialise_beacon_signed_payload covers everything except the signature") {
    StateVector sv = make_sv();
    // Populate signature with garbage — signed-payload must be byte-
    // identical regardless of the signature field's value.
    for (std::size_t i = 0; i < sv.signature.size(); ++i) {
        sv.signature[i] = static_cast<std::uint8_t>(i + 99u);
    }

    std::uint8_t prefix_a[uw::BEACON_SIGNED_PREFIX_BYTES];
    std::uint8_t prefix_b[uw::BEACON_SIGNED_PREFIX_BYTES];
    const auto na = uw::serialise_beacon_signed_payload(sv, prefix_a, sizeof prefix_a);
    CHECK(na == uw::BEACON_SIGNED_PREFIX_BYTES);

    // Change the signature; signed prefix should not change.
    for (std::size_t i = 0; i < sv.signature.size(); ++i) {
        sv.signature[i] = 0xFFu - sv.signature[i];
    }
    const auto nb = uw::serialise_beacon_signed_payload(sv, prefix_b, sizeof prefix_b);
    CHECK(nb == na);
    CHECK(std::memcmp(prefix_a, prefix_b, na) == 0);

    // Sanity: trailing 32 bytes of the prefix are the pubkey bytes.
    CHECK(std::memcmp(prefix_a + (uw::BEACON_SIGNED_PREFIX_BYTES - 32),
                       sv.sender_pubkey.public_key.data(), 32) == 0);
}

TEST_CASE("udp_wire: encode/decode round-trips the pubkey and signature fields") {
    StateVector sv = make_sv();
    for (std::size_t i = 0; i < sv.sender_pubkey.public_key.size(); ++i) {
        sv.sender_pubkey.public_key[i] = static_cast<std::uint8_t>(i * 5u + 11u);
    }
    for (std::size_t i = 0; i < sv.signature.size(); ++i) {
        sv.signature[i] = static_cast<std::uint8_t>(i * 7u + 41u);
    }

    std::uint8_t buf[uw::MAX_FRAME_BYTES];
    const auto n = uw::encode_beacon(sv, buf, sizeof buf);
    REQUIRE(n > 0u);

    const auto decoded = uw::decode_beacon(buf, n);
    REQUIRE(decoded.has_value());
    CHECK(std::memcmp(decoded->sender_pubkey.public_key.data(),
                       sv.sender_pubkey.public_key.data(),
                       sv.sender_pubkey.public_key.size()) == 0);
    CHECK(std::memcmp(decoded->signature.data(),
                       sv.signature.data(),
                       sv.signature.size()) == 0);
}

TEST_CASE("udp_wire: encoding is byte-stable across runs") {
    // Determinism check — same StateVector → same bytes.
    const auto sv = make_sv();
    std::uint8_t a[uw::MAX_FRAME_BYTES] = {};
    std::uint8_t b[uw::MAX_FRAME_BYTES] = {};
    const auto na = uw::encode_beacon(sv, a, sizeof a);
    const auto nb = uw::encode_beacon(sv, b, sizeof b);
    CHECK(na == nb);
    CHECK(std::memcmp(a, b, na) == 0);
}
