#include "mith/core/scheduler.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <numeric>

namespace mith {

namespace detail {

[[noreturn]] void scheduler_assert_fail(const char* msg) noexcept {
    // §15 prohibits exceptions; terminate with a structured stderr line that
    // EW deployments (§13.5) and CI can capture.
    std::fprintf(stderr, "mith::SystemScheduler assertion failed: %s\n", msg);
    std::abort();
}

} // namespace detail

SystemScheduler::SystemScheduler(SchedulerMode mode) noexcept
    : mode_(mode) {}

SchedulerStatus SystemScheduler::register_system(std::unique_ptr<System> system) {
    if (!system) {
        detail::scheduler_assert_fail("register_system: null system pointer");
    }

    const auto& name = system->describe().name;
    for (const auto& existing : systems_) {
        if (existing->describe().name == name) {
            return SchedulerStatus::DuplicateName;
        }
    }

    // Registering after build_graph() invalidates the resolved order.
    // The caller must rebuild before tick() — tick() aborts on !built_.
    built_ = false;

    systems_.push_back(std::move(system));
    return SchedulerStatus::Ok;
}

SchedulerStatus SystemScheduler::build_graph() {
    if (built_) return SchedulerStatus::AlreadyBuilt;

    // Sequential mode: lexicographic sort on name (§5.2 stable tie-breaking).
    // Parallel mode would additionally build a §5.1 hazard DAG here.
    order_.resize(systems_.size());
    std::iota(order_.begin(), order_.end(), std::size_t{0});

    std::sort(order_.begin(), order_.end(),
              [this](std::size_t a, std::size_t b) {
                  return systems_[a]->describe().name < systems_[b]->describe().name;
              });

    built_ = true;
    return SchedulerStatus::Ok;
}

void SystemScheduler::tick(EntityRegistry& registry,
                           const SwarmContext& ctx,
                           float delta_time) {
    if (!built_) {
        detail::scheduler_assert_fail(
            "tick(): build_graph() not called since last register_system()");
    }

    if (mode_ == SchedulerMode::Sequential) {
        for (const std::size_t idx : order_) {
            systems_[idx]->tick(registry, ctx, delta_time);
        }
        return;
    }

    // Parallel mode — slated for the next scheduler slice.
    detail::scheduler_assert_fail(
        "tick(): SchedulerMode::Parallel not yet implemented in v0.1 first slice");
}

SchedulerMode SystemScheduler::mode() const noexcept       { return mode_; }
std::size_t   SystemScheduler::system_count() const noexcept { return systems_.size(); }
bool          SystemScheduler::is_built() const noexcept    { return built_; }

} // namespace mith
