#include "mith/identity/chacha20.h"

#include <algorithm>
#include <cstring>
#include <random>

namespace mith::detail {

namespace {

// "expand 32-byte k" — RFC 8439 §2.3 ChaCha20 constants.
constexpr std::uint32_t CHACHA_CONST[4] = {
    0x61707865u, 0x3320646eu, 0x79622d32u, 0x6b206574u,
};

inline std::uint32_t rotl32(std::uint32_t x, int n) noexcept {
    return (x << n) | (x >> (32 - n));
}

inline std::uint32_t load_le32(const std::uint8_t* p) noexcept {
    return  static_cast<std::uint32_t>(p[0])
         | (static_cast<std::uint32_t>(p[1]) <<  8)
         | (static_cast<std::uint32_t>(p[2]) << 16)
         | (static_cast<std::uint32_t>(p[3]) << 24);
}

inline void store_le32(std::uint8_t* p, std::uint32_t v) noexcept {
    p[0] = static_cast<std::uint8_t>(v);
    p[1] = static_cast<std::uint8_t>(v >>  8);
    p[2] = static_cast<std::uint8_t>(v >> 16);
    p[3] = static_cast<std::uint8_t>(v >> 24);
}

// RFC 8439 §2.1 quarter-round.
inline void quarter_round(std::uint32_t& a, std::uint32_t& b,
                          std::uint32_t& c, std::uint32_t& d) noexcept {
    a += b; d ^= a; d = rotl32(d, 16);
    c += d; b ^= c; b = rotl32(b, 12);
    a += b; d ^= a; d = rotl32(d,  8);
    c += d; b ^= c; b = rotl32(b,  7);
}

} // namespace

void chacha20_block(const std::uint8_t key[32],
                    std::uint32_t       counter,
                    const std::uint8_t  nonce[12],
                    std::uint8_t        out[64]) noexcept {
    std::uint32_t s[16];
    s[0] = CHACHA_CONST[0]; s[1] = CHACHA_CONST[1];
    s[2] = CHACHA_CONST[2]; s[3] = CHACHA_CONST[3];
    for (int i = 0; i < 8; ++i) s[4 + i] = load_le32(key + i * 4);
    s[12] = counter;
    s[13] = load_le32(nonce + 0);
    s[14] = load_le32(nonce + 4);
    s[15] = load_le32(nonce + 8);

    std::uint32_t w[16];
    std::memcpy(w, s, sizeof(w));

    // 20 rounds = 10 column/diagonal pairs.
    for (int i = 0; i < 10; ++i) {
        // Columns
        quarter_round(w[0], w[4], w[ 8], w[12]);
        quarter_round(w[1], w[5], w[ 9], w[13]);
        quarter_round(w[2], w[6], w[10], w[14]);
        quarter_round(w[3], w[7], w[11], w[15]);
        // Diagonals
        quarter_round(w[0], w[5], w[10], w[15]);
        quarter_round(w[1], w[6], w[11], w[12]);
        quarter_round(w[2], w[7], w[ 8], w[13]);
        quarter_round(w[3], w[4], w[ 9], w[14]);
    }

    for (int i = 0; i < 16; ++i) {
        w[i] += s[i];
        store_le32(out + i * 4, w[i]);
    }
}

ChaCha20Csprng::ChaCha20Csprng() noexcept {
    std::random_device rd;
    std::uint8_t key[32];
    std::uint8_t nonce[12];
    for (auto& b : key)   b = static_cast<std::uint8_t>(rd() & 0xFFu);
    for (auto& b : nonce) b = static_cast<std::uint8_t>(rd() & 0xFFu);
    init_(key, nonce);
}

ChaCha20Csprng::ChaCha20Csprng(const std::uint8_t key[32],
                                const std::uint8_t nonce[12]) noexcept {
    init_(key, nonce);
}

void ChaCha20Csprng::init_(const std::uint8_t key[32],
                            const std::uint8_t nonce[12]) noexcept {
    std::memcpy(key_,   key,   32);
    std::memcpy(nonce_, nonce, 12);
    counter_      = 0;
    block_offset_ = 64;   // force refill on first fill()
}

void ChaCha20Csprng::fill(std::uint8_t* out, std::size_t n) noexcept {
    while (n > 0) {
        if (block_offset_ >= 64) {
            chacha20_block(key_, counter_, nonce_, block_);
            ++counter_;
            block_offset_ = 0;
        }
        const std::size_t take = std::min(n, std::size_t{64} - block_offset_);
        std::memcpy(out, block_ + block_offset_, take);
        out          += take;
        n            -= take;
        block_offset_ += take;
    }
}

} // namespace mith::detail

// ------------------------------------------------------------------------
// TweetNaCl-compatible randombytes(). Per-thread ChaCha20 CSPRNG —
// seeded once from std::random_device at first use on the thread.
// Replaces the per-call std::random_device shim from the first crypto
// slice.
// ------------------------------------------------------------------------

extern "C" {

void randombytes(unsigned char* x, unsigned long long xlen) {
    thread_local mith::detail::ChaCha20Csprng csprng;
    csprng.fill(static_cast<std::uint8_t*>(x), static_cast<std::size_t>(xlen));
}

} // extern "C"
