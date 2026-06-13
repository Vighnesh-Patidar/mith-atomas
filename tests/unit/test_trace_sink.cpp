#include "doctest.h"

#include "mith/core/trace_sink.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <string>

using mith::JsonTraceSink;
using mith::NullTraceSink;
using mith::TraceField;
using mith::TraceLevel;
using mith::TraceSink;
using mith::trace_level_str;

TEST_CASE("trace_level_str maps each level to its lowercase token") {
    static_assert(trace_level_str(TraceLevel::Off)   == std::string_view{"off"});
    static_assert(trace_level_str(TraceLevel::Warn)  == std::string_view{"warn"});
    static_assert(trace_level_str(TraceLevel::Info)  == std::string_view{"info"});
    static_assert(trace_level_str(TraceLevel::Debug) == std::string_view{"debug"});
}

TEST_CASE("TraceField static constructors set kind + value correctly") {
    auto s = TraceField::str("k", "v");
    CHECK(s.key == "k");
    CHECK(s.kind == TraceField::Kind::Str);
    CHECK(s.str_val == "v");

    auto i = TraceField::i64("n", -7);
    CHECK(i.kind == TraceField::Kind::I64);
    CHECK(i.i64_val == -7);

    auto u = TraceField::u64("u", 1234567890ull);
    CHECK(u.kind == TraceField::Kind::U64);
    CHECK(u.u64_val == 1234567890ull);

    auto f = TraceField::f64("x", 3.14);
    CHECK(f.kind == TraceField::Kind::F64);
    CHECK(f.f64_val == doctest::Approx(3.14));

    auto b = TraceField::boolean("flag", true);
    CHECK(b.kind == TraceField::Kind::Bool);
    CHECK(b.bool_val == true);
}

TEST_CASE("JsonTraceSink::format — empty fields list") {
    const std::string line =
        JsonTraceSink::format(TraceLevel::Info, "ping", nullptr, 0);
    CHECK(line == "{\"level\":\"info\",\"event\":\"ping\"}\n");
}

TEST_CASE("JsonTraceSink::format — single str field") {
    const TraceField fields[] = {
        TraceField::str("type_name", "MyKalmanState"),
    };
    const std::string line =
        JsonTraceSink::format(TraceLevel::Info, "component_registered",
                              fields, 1);
    CHECK(line ==
        "{\"level\":\"info\",\"event\":\"component_registered\","
        "\"type_name\":\"MyKalmanState\"}\n");
}

TEST_CASE("JsonTraceSink::format — mixed field types") {
    const TraceField fields[] = {
        TraceField::str("origin",    "user"),
        TraceField::str("type_name", "MyKalmanState"),
        TraceField::u64("tick",      0u),
        TraceField::f64("wall_time_s", 1.25),
        TraceField::boolean("dropped", false),
    };
    const std::string line =
        JsonTraceSink::format(TraceLevel::Info, "component_registered",
                              fields, sizeof(fields) / sizeof(fields[0]));

    // Don't bind to the exact %.17g representation of 1.25, but assert the
    // surrounding shape.
    CHECK(line.find("\"level\":\"info\"")               != std::string::npos);
    CHECK(line.find("\"event\":\"component_registered\"") != std::string::npos);
    CHECK(line.find("\"origin\":\"user\"")              != std::string::npos);
    CHECK(line.find("\"type_name\":\"MyKalmanState\"")  != std::string::npos);
    CHECK(line.find("\"tick\":0")                       != std::string::npos);
    CHECK(line.find("\"dropped\":false")                != std::string::npos);
    CHECK(line.back() == '\n');
}

TEST_CASE("JsonTraceSink::format — WARN and DEBUG levels emit correct strings") {
    auto warn  = JsonTraceSink::format(TraceLevel::Warn,  "queue_overflow", nullptr, 0);
    auto debug = JsonTraceSink::format(TraceLevel::Debug, "component_read", nullptr, 0);
    CHECK(warn.find("\"level\":\"warn\"")   != std::string::npos);
    CHECK(debug.find("\"level\":\"debug\"") != std::string::npos);
}

TEST_CASE("JsonTraceSink::format — special chars in strings get escaped") {
    const TraceField fields[] = {
        TraceField::str("msg", "got \"quotes\"\nand newlines"),
    };
    const std::string line =
        JsonTraceSink::format(TraceLevel::Warn, "weird", fields, 1);
    CHECK(line.find("\\\"quotes\\\"") != std::string::npos);
    CHECK(line.find("\\n") != std::string::npos);
}

TEST_CASE("NullTraceSink::emit — no crash, no side effects observable") {
    NullTraceSink sink;
    sink.emit(TraceLevel::Info, "anything");
    sink.emit(TraceLevel::Warn, "with_fields", {
        TraceField::i64("n", 42),
    });
    // No assertion possible against a no-op sink — the contract is "doesn't
    // throw, doesn't crash, doesn't write anywhere." Passing here means
    // we did not abort.
    CHECK(true);
}

TEST_CASE("JsonTraceSink::emit — Off level is silently dropped") {
    // Write into a tmpfile and verify nothing was written.
    std::FILE* tmp = std::tmpfile();
    REQUIRE(tmp != nullptr);

    JsonTraceSink sink(tmp);
    sink.emit(TraceLevel::Off, "should_be_dropped", {
        TraceField::str("k", "v"),
    });

    std::fflush(tmp);
    std::rewind(tmp);
    char buf[64] = {};
    const std::size_t n = std::fread(buf, 1, sizeof(buf), tmp);
    CHECK(n == 0u);

    std::fclose(tmp);
}

TEST_CASE("JsonTraceSink::emit — Info level writes a line to the FILE") {
    std::FILE* tmp = std::tmpfile();
    REQUIRE(tmp != nullptr);

    JsonTraceSink sink(tmp);
    sink.emit(TraceLevel::Info, "tick_boundary", {
        TraceField::u64("tick", 7u),
    });

    std::fflush(tmp);
    std::rewind(tmp);
    char buf[256] = {};
    const std::size_t n = std::fread(buf, 1, sizeof(buf) - 1, tmp);
    REQUIRE(n > 0u);
    const std::string line(buf, n);

    CHECK(line.find("\"level\":\"info\"")        != std::string::npos);
    CHECK(line.find("\"event\":\"tick_boundary\"") != std::string::npos);
    CHECK(line.find("\"tick\":7")                != std::string::npos);
    CHECK(line.back() == '\n');

    std::fclose(tmp);
}

TEST_CASE("TraceSink ergonomic overloads forward to the virtual emit") {
    // Test that initializer_list and std::array overloads dispatch correctly
    // to the virtual emit() by counting calls in a probe subclass.
    struct CountingSink : TraceSink {
        std::size_t calls = 0;
        std::size_t last_count = 0;
        using TraceSink::emit;
        void emit(TraceLevel, std::string_view,
                  const TraceField*, std::size_t count) noexcept override {
            ++calls;
            last_count = count;
        }
    };

    CountingSink sink;

    // Empty-fields overload
    sink.emit(TraceLevel::Info, "evt0");
    CHECK(sink.calls == 1u);
    CHECK(sink.last_count == 0u);

    // initializer_list overload
    sink.emit(TraceLevel::Info, "evt1", {
        TraceField::str("a", "x"),
        TraceField::i64("b", 1),
    });
    CHECK(sink.calls == 2u);
    CHECK(sink.last_count == 2u);

    // std::array overload
    std::array<TraceField, 3> arr{
        TraceField::str("k1", "v1"),
        TraceField::u64("k2", 2u),
        TraceField::boolean("k3", true),
    };
    sink.emit(TraceLevel::Debug, "evt2", arr);
    CHECK(sink.calls == 3u);
    CHECK(sink.last_count == 3u);
}
