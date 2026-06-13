#include "mith/core/json_writer.h"

#include <cmath>
#include <cstdio>
#include <utility>

namespace mith {

void JsonWriter::begin_object() {
    handle_array_separator_();
    buf_.push_back('{');
    stack_.push_back({ScopeKind::Object, true});
}

void JsonWriter::end_object() {
    buf_.push_back('}');
    if (!stack_.empty()) stack_.pop_back();
}

void JsonWriter::begin_array() {
    handle_array_separator_();
    buf_.push_back('[');
    stack_.push_back({ScopeKind::Array, true});
}

void JsonWriter::end_array() {
    buf_.push_back(']');
    if (!stack_.empty()) stack_.pop_back();
}

void JsonWriter::key(std::string_view k) {
    if (!stack_.empty() && !stack_.back().first) {
        buf_.push_back(',');
    }
    write_quoted_string_(k);
    buf_.push_back(':');
    if (!stack_.empty()) {
        stack_.back().first = false;
    }
}

void JsonWriter::write_null() {
    handle_array_separator_();
    buf_.append("null", 4);
}

void JsonWriter::write_bool(bool v) {
    handle_array_separator_();
    if (v) buf_.append("true", 4);
    else   buf_.append("false", 5);
}

void JsonWriter::write_i64(std::int64_t v) {
    handle_array_separator_();
    char tmp[32];
    const int len = std::snprintf(tmp, sizeof(tmp), "%lld",
                                  static_cast<long long>(v));
    if (len > 0) buf_.append(tmp, static_cast<std::size_t>(len));
}

void JsonWriter::write_u64(std::uint64_t v) {
    handle_array_separator_();
    char tmp[32];
    const int len = std::snprintf(tmp, sizeof(tmp), "%llu",
                                  static_cast<unsigned long long>(v));
    if (len > 0) buf_.append(tmp, static_cast<std::size_t>(len));
}

void JsonWriter::write_f64(double v) {
    handle_array_separator_();
    if (!std::isfinite(v)) {
        // Strict JSON doesn't support NaN / +/-Infinity — emit null instead.
        buf_.append("null", 4);
        return;
    }
    char tmp[32];
    const int len = std::snprintf(tmp, sizeof(tmp), "%.17g", v);
    if (len > 0) buf_.append(tmp, static_cast<std::size_t>(len));
}

void JsonWriter::write_string(std::string_view s) {
    handle_array_separator_();
    write_quoted_string_(s);
}

void JsonWriter::newline() {
    buf_.push_back('\n');
}

const std::string& JsonWriter::str() const noexcept { return buf_; }

std::string JsonWriter::take() noexcept {
    std::string out = std::move(buf_);
    buf_.clear();
    stack_.clear();
    return out;
}

void JsonWriter::clear() noexcept {
    buf_.clear();
    stack_.clear();
}

bool JsonWriter::well_formed() const noexcept {
    return stack_.empty();
}

void JsonWriter::handle_array_separator_() {
    if (!stack_.empty() && stack_.back().kind == ScopeKind::Array) {
        if (!stack_.back().first) buf_.push_back(',');
        stack_.back().first = false;
    }
}

void JsonWriter::write_quoted_string_(std::string_view s) {
    buf_.push_back('"');
    for (char c : s) {
        switch (c) {
            case '"':  buf_.append("\\\"", 2); break;
            case '\\': buf_.append("\\\\", 2); break;
            case '\b': buf_.append("\\b", 2);  break;
            case '\f': buf_.append("\\f", 2);  break;
            case '\n': buf_.append("\\n", 2);  break;
            case '\r': buf_.append("\\r", 2);  break;
            case '\t': buf_.append("\\t", 2);  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    // Control character — emit \u00XX escape.
                    char tmp[8];
                    const int n = std::snprintf(
                        tmp, sizeof(tmp), "\\u%04x",
                        static_cast<unsigned>(static_cast<unsigned char>(c)));
                    if (n > 0) buf_.append(tmp, static_cast<std::size_t>(n));
                } else {
                    buf_.push_back(c);
                }
        }
    }
    buf_.push_back('"');
}

} // namespace mith
