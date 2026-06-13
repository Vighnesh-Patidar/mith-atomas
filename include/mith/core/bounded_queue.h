#pragma once

// BoundedQueue<T, N, Policy> — see ARCHITECTURE.md §4.5
//
// Compile-time-policy ring buffer for bounded resources (action queues,
// comm buffers, transport TX). Header-only template.
//
// Three overflow policies, selected at compile time — no branch on the
// hot push path beyond the if-constexpr below.
//
//   DropOldest    On overflow, evict oldest, accept new. push() returns true.
//                 Use for observational streams (inbound beacons, outbound
//                 state vectors) where staleness > loss.
//
//   DropNewest    On overflow, refuse new. push() returns false. Use when
//                 in-flight items must complete (queued actions whose intent
//                 was already validated).
//
//   FaultTrigger  At the queue level: identical to DropNewest. The policy
//                 tag is what FaultMonitorSystem (§13.1, v0.2) reads when
//                 deciding which queues to convert overflow_events into
//                 health decrements.
//
// Counters never reset — dropped_count() and overflow_events() are monotonic
// from construction, exposed via dump_state() (§14.1) and assert-able in
// tests. They are equal under the simple per-push semantics in v0.1; both
// remain in the API for forward-compat with future batch semantics.
//
// Not thread-safe. Concurrent access between systems is serialised by the
// scheduler hazard graph (§5.1) — the owning system has exclusive write
// access within a tick.
//
// noexcept throughout. No allocation (uses std::array storage). T must be
// default-constructible and nothrow-movable.

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>
#include <utility>

namespace mith {

enum class OverflowPolicy : std::uint8_t {
    DropOldest    = 0,
    DropNewest    = 1,
    FaultTrigger  = 2,
};

template<typename T, std::size_t N, OverflowPolicy Policy>
class BoundedQueue {
public:
    static_assert(N > 0,
                  "BoundedQueue capacity must be > 0");
    static_assert(std::is_default_constructible_v<T>,
                  "BoundedQueue requires T to be default-constructible");
    static_assert(std::is_nothrow_move_assignable_v<T>,
                  "BoundedQueue requires T to be nothrow-move-assignable");
    static_assert(std::is_nothrow_move_constructible_v<T>,
                  "BoundedQueue requires T to be nothrow-move-constructible");

    using value_type = T;

    static constexpr std::size_t    CAPACITY = N;
    static constexpr OverflowPolicy POLICY   = Policy;

    constexpr BoundedQueue() noexcept = default;

    // Push an item onto the tail.
    //   DropOldest:               always returns true; evicts head on overflow.
    //   DropNewest / FaultTrigger: returns false on overflow; new item discarded.
    bool push(T&& item) noexcept {
        if (size_ == N) {
            ++overflow_events_;
            ++dropped_count_;
            if constexpr (Policy == OverflowPolicy::DropOldest) {
                // Overwrite head's slot (== tail_ when full), advance both.
                buffer_[tail_] = std::move(item);
                tail_ = next_index(tail_);
                head_ = next_index(head_);
                return true;
            } else {
                return false;
            }
        }
        buffer_[tail_] = std::move(item);
        tail_ = next_index(tail_);
        ++size_;
        return true;
    }

    // Pop the oldest item. Returns nullopt on empty.
    std::optional<T> pop() noexcept {
        if (size_ == 0) return std::nullopt;
        T item = std::move(buffer_[head_]);
        head_ = next_index(head_);
        --size_;
        return std::optional<T>{std::move(item)};
    }

    constexpr std::size_t size()     const noexcept { return size_; }
    constexpr std::size_t capacity() const noexcept { return N; }
    constexpr bool        empty()    const noexcept { return size_ == 0; }
    constexpr bool        full()     const noexcept { return size_ == N; }

    // §14.3 observability counters. Monotonic from construction.
    constexpr std::uint32_t dropped_count()    const noexcept { return dropped_count_; }
    constexpr std::uint32_t overflow_events()  const noexcept { return overflow_events_; }

private:
    static constexpr std::size_t next_index(std::size_t i) noexcept {
        return (i + 1) % N;
    }

    std::array<T, N>  buffer_{};
    std::size_t       head_ = 0;
    std::size_t       tail_ = 0;
    std::size_t       size_ = 0;
    std::uint32_t     dropped_count_   = 0;
    std::uint32_t     overflow_events_ = 0;
};

} // namespace mith
