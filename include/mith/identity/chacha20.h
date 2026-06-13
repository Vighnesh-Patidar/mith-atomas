#pragma once

// ChaCha20 CSPRNG — see ARCHITECTURE.md §3.3
//
// Implements RFC 8439 §2.3 (ChaCha20 block function) and a stream CSPRNG
// keyed from std::random_device on construction. Replaces the
// std::random_device-per-call shim that the first crypto slice used,
// removing the platform-quality dependency from every randombytes()
// call into a one-time seeding event.
//
// Threading: each thread that needs randomness gets its own
// thread_local CSPRNG instance (see src/identity/chacha20.cpp's
// randombytes() implementation). Per-thread state means no shared
// mutex on the hot path; per-thread seed means workers don't share
// counter state.
//
// What this is and isn't:
//   - Cryptographically strong keystream (ChaCha20 is the standard
//     stream cipher used in TLS 1.3 / WireGuard / OpenSSH).
//   - NOT an authenticated cipher — we use it only for randomness.
//     For signed-mode message authentication see Ed25519 (§3.3).

#include <cstddef>
#include <cstdint>

namespace mith::detail {

// RFC 8439 §2.3 ChaCha20 block function. Produces 64 bytes of keystream
// from a 256-bit key, 32-bit block counter, and 96-bit nonce.
//
// Pure (no static state). Exposed for the RFC-vector test in
// test_chacha20.cpp; production code goes through ChaCha20Csprng.
void chacha20_block(const std::uint8_t key[32],
                    std::uint32_t       counter,
                    const std::uint8_t  nonce[12],
                    std::uint8_t        out[64]) noexcept;

// Stream CSPRNG keyed at construction. fill() draws n bytes of keystream
// and advances the internal counter; the counter+key are never reset
// short of destruction, so the same instance never produces repeating
// output until the 2^32-block (256 GB) keystream period exhausts.
class ChaCha20Csprng {
public:
    // Seed from std::random_device. If random_device fails the
    // constructor terminates — without entropy we can't safely
    // produce identity material (§3.3 fail-closed rationale).
    ChaCha20Csprng() noexcept;

    // Explicit seed — for reproducible tests and the future seeded-sim
    // determinism story.
    ChaCha20Csprng(const std::uint8_t key[32], const std::uint8_t nonce[12]) noexcept;

    // Fill `out[0..n)` with fresh keystream. Cheap; produces 64 bytes
    // per ChaCha20 block, caches the tail of the current block for the
    // next call.
    void fill(std::uint8_t* out, std::size_t n) noexcept;

private:
    void init_(const std::uint8_t key[32], const std::uint8_t nonce[12]) noexcept;

    std::uint8_t  key_[32];
    std::uint8_t  nonce_[12];
    std::uint32_t counter_      = 0;
    std::uint8_t  block_[64]    = {};
    std::size_t   block_offset_ = 64;   // forces refill on first fill()
};

} // namespace mith::detail
