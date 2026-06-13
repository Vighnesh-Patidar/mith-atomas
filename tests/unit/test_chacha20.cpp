#include "doctest.h"

#ifdef MITH_AUTH_ENABLED

#include "mith/identity/chacha20.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <unordered_set>

using mith::detail::chacha20_block;
using mith::detail::ChaCha20Csprng;

// ------------------------------------------------------------------------
// RFC 8439 §2.3.2 test vector for chacha20_block.
// ------------------------------------------------------------------------

TEST_CASE("chacha20_block matches RFC 8439 §2.3.2 keystream vector") {
    const std::uint8_t key[32] = {
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
        0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
        0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    };
    const std::uint32_t counter = 1;
    const std::uint8_t nonce[12] = {
        0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x4a,
        0x00, 0x00, 0x00, 0x00,
    };
    const std::uint8_t expected[64] = {
        0x10, 0xf1, 0xe7, 0xe4, 0xd1, 0x3b, 0x59, 0x15,
        0x50, 0x0f, 0xdd, 0x1f, 0xa3, 0x20, 0x71, 0xc4,
        0xc7, 0xd1, 0xf4, 0xc7, 0x33, 0xc0, 0x68, 0x03,
        0x04, 0x22, 0xaa, 0x9a, 0xc3, 0xd4, 0x6c, 0x4e,
        0xd2, 0x82, 0x64, 0x46, 0x07, 0x9f, 0xaa, 0x09,
        0x14, 0xc2, 0xd7, 0x05, 0xd9, 0x8b, 0x02, 0xa2,
        0xb5, 0x12, 0x9c, 0xd1, 0xde, 0x16, 0x4e, 0xb9,
        0xcb, 0xd0, 0x83, 0xe8, 0xa2, 0x50, 0x3c, 0x4e,
    };

    std::uint8_t out[64] = {};
    chacha20_block(key, counter, nonce, out);

    CHECK(std::memcmp(out, expected, 64) == 0);
}

TEST_CASE("chacha20_block: different counters produce different output") {
    const std::uint8_t key[32]   = {};   // all zero
    const std::uint8_t nonce[12] = {};   // all zero

    std::uint8_t a[64] = {}, b[64] = {};
    chacha20_block(key, 0, nonce, a);
    chacha20_block(key, 1, nonce, b);
    CHECK(std::memcmp(a, b, 64) != 0);
}

TEST_CASE("chacha20_block: different keys produce different output") {
    std::uint8_t k0[32] = {};
    std::uint8_t k1[32] = {};
    k1[0] = 1;
    const std::uint8_t nonce[12] = {};

    std::uint8_t a[64] = {}, b[64] = {};
    chacha20_block(k0, 0, nonce, a);
    chacha20_block(k1, 0, nonce, b);
    CHECK(std::memcmp(a, b, 64) != 0);
}

TEST_CASE("chacha20_block is deterministic for the same inputs") {
    const std::uint8_t key[32]   = {1, 2, 3, 4, 5, 6, 7, 8};
    const std::uint8_t nonce[12] = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0xff, 0x00};

    std::uint8_t a[64] = {}, b[64] = {};
    chacha20_block(key, 42, nonce, a);
    chacha20_block(key, 42, nonce, b);
    CHECK(std::memcmp(a, b, 64) == 0);
}

// ------------------------------------------------------------------------
// ChaCha20Csprng
// ------------------------------------------------------------------------

TEST_CASE("ChaCha20Csprng: seeded instance produces deterministic output") {
    const std::uint8_t key[32]   = {0xaa};   // first byte set, rest zero
    const std::uint8_t nonce[12] = {0xbb};

    ChaCha20Csprng a(key, nonce);
    ChaCha20Csprng b(key, nonce);

    std::uint8_t out_a[200] = {}, out_b[200] = {};
    a.fill(out_a, sizeof(out_a));
    b.fill(out_b, sizeof(out_b));

    CHECK(std::memcmp(out_a, out_b, sizeof(out_a)) == 0);
}

TEST_CASE("ChaCha20Csprng: successive fill() calls do not repeat") {
    const std::uint8_t key[32]   = {};
    const std::uint8_t nonce[12] = {};
    ChaCha20Csprng c(key, nonce);

    std::uint8_t a[64] = {}, b[64] = {};
    c.fill(a, sizeof(a));
    c.fill(b, sizeof(b));
    CHECK(std::memcmp(a, b, sizeof(a)) != 0);
}

TEST_CASE("ChaCha20Csprng: produces non-uniform 32-bit values across a large draw") {
    // Statistical sanity: 10 000 32-bit draws from the CSPRNG should
    // produce essentially no collisions among themselves (~birthday
    // probability 0.0000116). If we see lots of repeats the impl is
    // catastrophically broken.
    const std::uint8_t key[32]   = {1, 2, 3, 4};
    const std::uint8_t nonce[12] = {5, 6, 7, 8};
    ChaCha20Csprng c(key, nonce);

    constexpr std::size_t N = 10000;
    std::unordered_set<std::uint32_t> seen;
    seen.reserve(N);

    for (std::size_t i = 0; i < N; ++i) {
        std::uint8_t buf[4];
        c.fill(buf, 4);
        const std::uint32_t v =
            std::uint32_t(buf[0]) | (std::uint32_t(buf[1]) <<  8)
          | (std::uint32_t(buf[2]) << 16) | (std::uint32_t(buf[3]) << 24);
        seen.insert(v);
    }
    CHECK(seen.size() > N - 10);   // very loose; catches catastrophic-only failures
}

TEST_CASE("ChaCha20Csprng: cross-block boundary fills are contiguous keystream") {
    // Filling 100 bytes once should equal two filling 50+50 from a sibling
    // CSPRNG with the same seed.
    const std::uint8_t key[32]   = {7};
    const std::uint8_t nonce[12] = {11};

    ChaCha20Csprng a(key, nonce);
    ChaCha20Csprng b(key, nonce);

    std::uint8_t one_shot[100] = {};
    a.fill(one_shot, sizeof(one_shot));

    std::uint8_t two_shot[100] = {};
    b.fill(two_shot, 50);
    b.fill(two_shot + 50, 50);

    CHECK(std::memcmp(one_shot, two_shot, sizeof(one_shot)) == 0);
}

TEST_CASE("ChaCha20Csprng: default constructor seeds from std::random_device") {
    // No deterministic check possible — just verify two default-constructed
    // instances produce different keystreams (vanishingly unlikely to
    // collide if random_device is functional).
    ChaCha20Csprng a;
    ChaCha20Csprng b;
    std::uint8_t out_a[64] = {}, out_b[64] = {};
    a.fill(out_a, 64);
    b.fill(out_b, 64);
    CHECK(std::memcmp(out_a, out_b, 64) != 0);
}

#endif // MITH_AUTH_ENABLED
