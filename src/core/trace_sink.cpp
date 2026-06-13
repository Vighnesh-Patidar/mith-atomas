#include "mith/core/trace_sink.h"

#include "mith/core/json_writer.h"

#include <cstdio>

namespace mith {

JsonTraceSink::JsonTraceSink() noexcept : dest_(stderr) {}
JsonTraceSink::JsonTraceSink(std::FILE* dest) noexcept : dest_(dest) {}

std::string JsonTraceSink::format(TraceLevel level,
                                  std::string_view event,
                                  const TraceField* fields,
                                  std::size_t       field_count) {
    JsonWriter w;
    w.begin_object();

    w.key("level");
    w.write_string(trace_level_str(level));

    w.key("event");
    w.write_string(event);

    for (std::size_t i = 0; i < field_count; ++i) {
        const TraceField& f = fields[i];
        w.key(f.key);
        switch (f.kind) {
            case TraceField::Kind::Str:  w.write_string(f.str_val); break;
            case TraceField::Kind::I64:  w.write_i64(f.i64_val);    break;
            case TraceField::Kind::U64:  w.write_u64(f.u64_val);    break;
            case TraceField::Kind::F64:  w.write_f64(f.f64_val);    break;
            case TraceField::Kind::Bool: w.write_bool(f.bool_val);  break;
        }
    }

    w.end_object();
    w.newline();
    return w.take();
}

void JsonTraceSink::emit(TraceLevel level,
                         std::string_view event,
                         const TraceField* fields,
                         std::size_t       field_count) noexcept {
    if (level == TraceLevel::Off) return;
    if (!dest_) return;

    // format() allocates a std::string; we accept that — observability is
    // not in the hot tick path (§15). Binary sinks for tight platforms
    // avoid this allocation by writing into a pre-allocated ring buffer.
    const std::string line = format(level, event, fields, field_count);
    std::fwrite(line.data(), 1, line.size(), dest_);
}

} // namespace mith
