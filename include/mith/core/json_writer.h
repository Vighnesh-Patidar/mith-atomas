#pragma once

// JsonWriter — see ARCHITECTURE.md §14.1
//
// Hand-rolled, write-only JSON emitter. Used by World::dump_state() (§14.1)
// and JsonTraceSink (§14.4). The core runtime has no external JSON
// dependency — see §11.
//
// Supported: objects, arrays, nesting, strings (with escape), bool, null,
// signed/unsigned 64-bit integers, doubles (NaN/Inf emit as null per JSON
// spec). NOT supported: arbitrary-precision numbers, JSON parsing.
//
// State machine: a stack of scopes (Object/Array) with a first-item flag
// per scope. Commas land automatically — callers just call begin_object /
// key / value / end_object in the natural order.
//
// Not thread-safe. Allocation is allowed (this lives in the observability
// path, not the hot tick path — see §15).

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace mith {

class JsonWriter {
public:
    JsonWriter() = default;

    void begin_object();
    void end_object();
    void begin_array();
    void end_array();

    // Object key. Must be called inside an object scope; the next value-
    // emitting call provides the value.
    void key(std::string_view k);

    void write_null();
    void write_bool(bool v);
    void write_i64(std::int64_t v);
    void write_u64(std::uint64_t v);
    void write_f64(double v);
    void write_string(std::string_view s);

    // Append a literal '\n' to the buffer (for JSON-lines streams).
    void newline();

    // Output access. take() moves the buffer out and resets state; str()
    // returns a const reference without copying or resetting.
    const std::string& str()  const noexcept;
    std::string        take() noexcept;
    void               clear() noexcept;

    // True iff all scopes have been closed (no unmatched begin_object /
    // begin_array). Useful for asserting in tests.
    bool well_formed() const noexcept;

private:
    enum class ScopeKind : std::uint8_t { Object, Array };
    struct Scope { ScopeKind kind; bool first; };

    void handle_array_separator_();
    void write_quoted_string_(std::string_view s);

    std::string        buf_;
    std::vector<Scope> stack_;
};

} // namespace mith
