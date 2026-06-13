#include "doctest.h"

#include "mith/core/bounded_queue.h"

#include <cstdint>
#include <memory>
#include <type_traits>

using mith::BoundedQueue;
using mith::OverflowPolicy;

TEST_CASE("default-constructed queue is empty and zeroed") {
    BoundedQueue<int, 4, OverflowPolicy::DropOldest> q;

    CHECK(q.size() == 0u);
    CHECK(q.empty());
    CHECK_FALSE(q.full());
    CHECK(q.capacity() == 4u);
    CHECK(q.dropped_count() == 0u);
    CHECK(q.overflow_events() == 0u);
}

TEST_CASE("push / pop preserves FIFO order") {
    BoundedQueue<int, 4, OverflowPolicy::DropOldest> q;

    REQUIRE(q.push(1));
    REQUIRE(q.push(2));
    REQUIRE(q.push(3));
    CHECK(q.size() == 3u);
    CHECK_FALSE(q.full());

    auto a = q.pop();
    REQUIRE(a.has_value());
    CHECK(*a == 1);

    auto b = q.pop();
    REQUIRE(b.has_value());
    CHECK(*b == 2);

    auto c = q.pop();
    REQUIRE(c.has_value());
    CHECK(*c == 3);

    CHECK(q.empty());
}

TEST_CASE("pop on empty queue returns nullopt") {
    BoundedQueue<int, 4, OverflowPolicy::DropOldest> q;
    CHECK_FALSE(q.pop().has_value());

    // Still nullopt after a push/pop cycle leaves it empty.
    REQUIRE(q.push(7));
    REQUIRE(q.pop().has_value());
    CHECK_FALSE(q.pop().has_value());
}

TEST_CASE("DropOldest accepts on overflow and evicts the oldest") {
    BoundedQueue<int, 3, OverflowPolicy::DropOldest> q;

    REQUIRE(q.push(1));
    REQUIRE(q.push(2));
    REQUIRE(q.push(3));
    REQUIRE(q.full());

    // Overflow push — must return true and evict the oldest (1).
    REQUIRE(q.push(4));
    CHECK(q.full());
    CHECK(q.dropped_count() == 1u);
    CHECK(q.overflow_events() == 1u);

    // FIFO order is now 2, 3, 4.
    CHECK(*q.pop() == 2);
    CHECK(*q.pop() == 3);
    CHECK(*q.pop() == 4);
    CHECK(q.empty());
}

TEST_CASE("DropOldest under sustained overflow retains the newest N") {
    BoundedQueue<int, 3, OverflowPolicy::DropOldest> q;

    for (int i = 1; i <= 10; ++i) {
        REQUIRE(q.push(int{i}));
    }

    CHECK(q.size() == 3u);
    CHECK(q.dropped_count() == 7u);    // 10 pushes, 3 kept, 7 evicted
    CHECK(q.overflow_events() == 7u);

    CHECK(*q.pop() == 8);
    CHECK(*q.pop() == 9);
    CHECK(*q.pop() == 10);
}

TEST_CASE("DropNewest refuses on overflow and retains originals") {
    BoundedQueue<int, 3, OverflowPolicy::DropNewest> q;

    REQUIRE(q.push(1));
    REQUIRE(q.push(2));
    REQUIRE(q.push(3));
    REQUIRE(q.full());

    // Overflow push — must return false and leave originals intact.
    CHECK_FALSE(q.push(4));
    CHECK(q.dropped_count() == 1u);
    CHECK(q.overflow_events() == 1u);
    CHECK(q.full());

    // FIFO order unchanged: 1, 2, 3.
    CHECK(*q.pop() == 1);
    CHECK(*q.pop() == 2);
    CHECK(*q.pop() == 3);
}

TEST_CASE("DropNewest under sustained overflow keeps the oldest N") {
    BoundedQueue<int, 3, OverflowPolicy::DropNewest> q;

    REQUIRE(q.push(1));
    REQUIRE(q.push(2));
    REQUIRE(q.push(3));

    for (int i = 4; i <= 10; ++i) {
        CHECK_FALSE(q.push(int{i}));
    }

    CHECK(q.dropped_count() == 7u);
    CHECK(q.overflow_events() == 7u);

    CHECK(*q.pop() == 1);
    CHECK(*q.pop() == 2);
    CHECK(*q.pop() == 3);
}

TEST_CASE("FaultTrigger behaves like DropNewest at the queue level") {
    BoundedQueue<int, 2, OverflowPolicy::FaultTrigger> q;

    REQUIRE(q.push(10));
    REQUIRE(q.push(20));
    CHECK_FALSE(q.push(30));
    CHECK_FALSE(q.push(40));

    CHECK(q.dropped_count() == 2u);
    CHECK(q.overflow_events() == 2u);

    CHECK(*q.pop() == 10);
    CHECK(*q.pop() == 20);
}

TEST_CASE("ring buffer wraparound across many cycles") {
    BoundedQueue<int, 3, OverflowPolicy::DropNewest> q;

    for (int round = 0; round < 5; ++round) {
        for (int i = 1; i <= 3; ++i) {
            REQUIRE(q.push(int{round * 10 + i}));
        }
        for (int i = 1; i <= 3; ++i) {
            auto v = q.pop();
            REQUIRE(v.has_value());
            CHECK(*v == round * 10 + i);
        }
    }

    CHECK(q.dropped_count() == 0u);
    CHECK(q.overflow_events() == 0u);
}

TEST_CASE("counters never decrement after pops or successful pushes") {
    BoundedQueue<int, 2, OverflowPolicy::DropNewest> q;

    REQUIRE(q.push(1));
    REQUIRE(q.push(2));
    REQUIRE_FALSE(q.push(3));     // triggers one drop
    CHECK(q.dropped_count() == 1u);

    REQUIRE(q.pop().has_value()); // drain one slot
    CHECK(q.dropped_count() == 1u);   // counter holds across pop

    REQUIRE(q.push(4));           // successful push to the now-free slot
    CHECK(q.dropped_count() == 1u);   // counter holds across successful push

    REQUIRE_FALSE(q.push(5));     // another drop
    CHECK(q.dropped_count() == 2u);
    CHECK(q.overflow_events() == 2u);
}

TEST_CASE("static introspection — CAPACITY and POLICY") {
    using Q = BoundedQueue<int, 16, OverflowPolicy::FaultTrigger>;
    static_assert(Q::CAPACITY == 16u);
    static_assert(Q::POLICY == OverflowPolicy::FaultTrigger);
    static_assert(std::is_same_v<Q::value_type, int>);
}

namespace {
struct MoveOnly {
    int v = 0;
    MoveOnly() noexcept = default;
    explicit MoveOnly(int x) noexcept : v(x) {}
    MoveOnly(MoveOnly&&) noexcept = default;
    MoveOnly& operator=(MoveOnly&&) noexcept = default;
    MoveOnly(const MoveOnly&) = delete;
    MoveOnly& operator=(const MoveOnly&) = delete;
};
} // namespace

TEST_CASE("works with move-only element types") {
    BoundedQueue<MoveOnly, 4, OverflowPolicy::DropOldest> q;

    REQUIRE(q.push(MoveOnly{42}));
    REQUIRE(q.push(MoveOnly{43}));
    REQUIRE(q.push(MoveOnly{44}));

    auto a = q.pop();
    REQUIRE(a.has_value());
    CHECK(a->v == 42);

    auto b = q.pop();
    REQUIRE(b.has_value());
    CHECK(b->v == 43);
}

TEST_CASE("DropOldest with move-only types evicts correctly") {
    BoundedQueue<MoveOnly, 2, OverflowPolicy::DropOldest> q;

    REQUIRE(q.push(MoveOnly{1}));
    REQUIRE(q.push(MoveOnly{2}));
    REQUIRE(q.full());

    REQUIRE(q.push(MoveOnly{3}));   // evicts MoveOnly{1}
    CHECK(q.dropped_count() == 1u);

    auto first = q.pop();
    REQUIRE(first.has_value());
    CHECK(first->v == 2);

    auto second = q.pop();
    REQUIRE(second.has_value());
    CHECK(second->v == 3);
}

TEST_CASE("capacity = 1 — degenerate single-slot queue") {
    BoundedQueue<int, 1, OverflowPolicy::DropOldest> q;

    REQUIRE(q.push(1));
    CHECK(q.full());
    REQUIRE(q.push(2));            // evicts 1
    CHECK(q.dropped_count() == 1u);

    auto v = q.pop();
    REQUIRE(v.has_value());
    CHECK(*v == 2);
    CHECK(q.empty());
}
