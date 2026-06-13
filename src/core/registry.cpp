#include "mith/core/registry.h"

#include "mith/core/trace_sink.h"

#include <cstdio>
#include <cstdlib>

namespace mith {

namespace detail {

[[noreturn]] void registry_assert_fail(const char* msg) noexcept {
    // §15 prohibits exceptions; we terminate with a structured message on
    // the stderr stream that EW deployments (§13.5) and CI tests can capture.
    std::fprintf(stderr, "mith::EntityRegistry assertion failed: %s\n", msg);
    std::abort();
}

} // namespace detail

EntityRegistry::EntityRegistry(ComponentRegistrationPolicy policy) noexcept
    : policy_(policy) {}

void EntityRegistry::lock() noexcept {
    locked_ = true;
}

bool EntityRegistry::is_locked() const noexcept {
    return locked_;
}

std::size_t EntityRegistry::registered_count() const noexcept {
    return stores_.size();
}

ComponentRegistrationPolicy EntityRegistry::policy() const noexcept {
    return policy_;
}

void EntityRegistry::set_trace_sink(TraceSink* sink) noexcept {
    sink_ = sink;
}

TraceSink* EntityRegistry::trace_sink() const noexcept {
    return sink_;
}

void EntityRegistry::emit_registered_event_(ComponentOrigin   origin,
                                             std::string_view  type_name_,
                                             ComponentTypeID   type_id) noexcept {
    if (!sink_) return;

    // 0x + 16 hex digits + null = 19 bytes. Lives on the stack only for
    // the duration of the emit() call; the sink reads the string_view
    // synchronously.
    char type_id_buf[19];
    std::snprintf(type_id_buf, sizeof(type_id_buf),
                  "0x%016llx",
                  static_cast<unsigned long long>(type_id));

    const std::string_view origin_str =
        (origin == ComponentOrigin::Built_In) ? "built_in" : "user";

    const TraceField fields[] = {
        TraceField::str("origin",    origin_str),
        TraceField::str("type_name", type_name_),
        TraceField::str("type_id",   type_id_buf),
        TraceField::u64("tick",      0u),
    };
    sink_->emit(TraceLevel::Info, "component_registered",
                fields, sizeof(fields) / sizeof(fields[0]));
}

} // namespace mith
