#pragma once

// TraceSink — see ARCHITECTURE.md §14.4
//
// Structured trace events flow through a pluggable sink. The default sink
// (JsonTraceSink) writes one JSON object per line to a FILE* (stderr by
// default). Embedded users plug in their own — binary frame format,
// logfmt, MAVLink-style telemetry, syslog — without touching runtime code.
//
// The runtime's emit() call site is allocation-free: the caller passes a
// pointer + count (typically pointing at a local std::array or initializer-
// list backing). The sink itself may allocate — JsonTraceSink does
// (std::string per line); binary sinks typically don't.
//
// All TraceField string_views must outlive the emit() call. Sinks read them
// synchronously and do not retain pointers.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <string>
#include <string_view>

namespace mith {

enum class TraceLevel : std::uint8_t {
    Off   = 0,
    Warn  = 1,
    Info  = 2,
    Debug = 3,
};

// Lowercase string representation of a TraceLevel, matching the value
// JsonTraceSink emits in the "level" field.
constexpr std::string_view trace_level_str(TraceLevel level) noexcept {
    switch (level) {
        case TraceLevel::Off:   return "off";
        case TraceLevel::Warn:  return "warn";
        case TraceLevel::Info:  return "info";
        case TraceLevel::Debug: return "debug";
    }
    return "unknown";
}

// Tagged-union field. Use the static constructors — they zero-init the
// union and set the kind correctly.
struct TraceField {
    enum class Kind : std::uint8_t { Str, I64, U64, F64, Bool };

    std::string_view key;
    Kind             kind = Kind::Bool;

    union {
        std::string_view str_val;
        std::int64_t     i64_val;
        std::uint64_t    u64_val;
        double           f64_val;
        bool             bool_val;
    };

    constexpr TraceField() noexcept : str_val{} {}

    static constexpr TraceField str(std::string_view k, std::string_view v) noexcept {
        TraceField f; f.key = k; f.kind = Kind::Str;  f.str_val  = v; return f;
    }
    static constexpr TraceField i64(std::string_view k, std::int64_t v) noexcept {
        TraceField f; f.key = k; f.kind = Kind::I64;  f.i64_val  = v; return f;
    }
    static constexpr TraceField u64(std::string_view k, std::uint64_t v) noexcept {
        TraceField f; f.key = k; f.kind = Kind::U64;  f.u64_val  = v; return f;
    }
    static constexpr TraceField f64(std::string_view k, double v) noexcept {
        TraceField f; f.key = k; f.kind = Kind::F64;  f.f64_val  = v; return f;
    }
    static constexpr TraceField boolean(std::string_view k, bool v) noexcept {
        TraceField f; f.key = k; f.kind = Kind::Bool; f.bool_val = v; return f;
    }
};

class TraceSink {
public:
    virtual ~TraceSink() = default;

    // Primary virtual entry point — pointer + count. Sinks override this.
    // Events emitted with TraceLevel::Off must be silently dropped.
    virtual void emit(TraceLevel level,
                      std::string_view event,
                      const TraceField* fields,
                      std::size_t       field_count) noexcept = 0;

    // Ergonomic overload: empty fields list.
    void emit(TraceLevel level, std::string_view event) noexcept {
        emit(level, event, nullptr, 0);
    }

    // Ergonomic overload: initializer_list. Backing storage lives until the
    // end of the call — no allocation in the runtime emit path.
    void emit(TraceLevel level,
              std::string_view event,
              std::initializer_list<TraceField> fields) noexcept {
        emit(level, event, fields.begin(), fields.size());
    }

    // Ergonomic overload: std::array of any size.
    template<std::size_t N>
    void emit(TraceLevel level,
              std::string_view event,
              const std::array<TraceField, N>& fields) noexcept {
        emit(level, event, fields.data(), N);
    }
};

// Default sink: one JSON object per line to a FILE* (stderr by default).
// Caller owns the FILE* lifetime — sink does not close it.
class JsonTraceSink : public TraceSink {
public:
    JsonTraceSink() noexcept;
    explicit JsonTraceSink(std::FILE* dest) noexcept;

    using TraceSink::emit;   // bring inherited overloads into scope
    void emit(TraceLevel level,
              std::string_view event,
              const TraceField* fields,
              std::size_t       field_count) noexcept override;

    // Pure formatting helper. Used internally by emit(); exposed so tests
    // can verify the output shape without FILE I/O. Always emits a trailing
    // newline. Does not filter by level — caller decides.
    static std::string format(TraceLevel level,
                              std::string_view event,
                              const TraceField* fields,
                              std::size_t       field_count);

private:
    std::FILE* dest_;
};

// No-op sink. Use when trace_level is Off or in tests that don't care
// about output.
class NullTraceSink : public TraceSink {
public:
    using TraceSink::emit;
    void emit(TraceLevel,
              std::string_view,
              const TraceField*,
              std::size_t) noexcept override {}
};

} // namespace mith
