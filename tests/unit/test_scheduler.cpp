#include "doctest.h"

#include "mith/core/registry.h"
#include "mith/core/scheduler.h"
#include "mith/core/system.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

using mith::EntityRegistry;
using mith::ResourceID;
using mith::SchedulerMode;
using mith::SchedulerStatus;
using mith::SwarmContext;
using mith::System;
using mith::SystemDescriptor;
using mith::SystemScheduler;

namespace {

// Probe system: appends its name to a shared log when ticked. Supports a
// user-provided SystemDescriptor for hazard-shape tests.
class TaggingSystem : public System {
public:
    TaggingSystem(std::string name, std::vector<std::string>& log)
        : name_(std::move(name)), log_(log) {
        desc_.name = name_;
    }

    TaggingSystem(std::string name, std::vector<std::string>& log,
                  SystemDescriptor desc)
        : name_(std::move(name)), log_(log), desc_(std::move(desc)) {
        // Constructor's `name` always wins — overrides whatever was in desc.
        desc_.name = name_;
    }

    SystemDescriptor describe() const override { return desc_; }

    void tick(EntityRegistry&, const SwarmContext&, float) override {
        log_.push_back(name_);
    }

private:
    std::string               name_;
    std::vector<std::string>& log_;
    SystemDescriptor          desc_;
};

} // namespace

TEST_CASE("default-constructed scheduler: empty, unbuilt, Sequential") {
    SystemScheduler sched;
    CHECK(sched.system_count() == 0u);
    CHECK_FALSE(sched.is_built());
    CHECK(sched.mode() == SchedulerMode::Sequential);
}

TEST_CASE("explicit Parallel mode is preserved on the scheduler") {
    SystemScheduler sched(SchedulerMode::Parallel);
    CHECK(sched.mode() == SchedulerMode::Parallel);
}

TEST_CASE("register_system: success then DuplicateName on same name") {
    std::vector<std::string> log;
    SystemScheduler sched;

    CHECK(sched.register_system(std::make_unique<TaggingSystem>("A", log))
          == SchedulerStatus::Ok);
    CHECK(sched.system_count() == 1u);

    CHECK(sched.register_system(std::make_unique<TaggingSystem>("A", log))
          == SchedulerStatus::DuplicateName);
    CHECK(sched.system_count() == 1u);
}

TEST_CASE("Sequential build + tick: systems run in lexicographic order") {
    std::vector<std::string> log;
    SystemScheduler sched;
    EntityRegistry  reg;
    SwarmContext    ctx;

    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("Charlie", log))
            == SchedulerStatus::Ok);
    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("Alpha", log))
            == SchedulerStatus::Ok);
    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("Bravo", log))
            == SchedulerStatus::Ok);

    REQUIRE(sched.build_graph() == SchedulerStatus::Ok);
    CHECK(sched.is_built());

    sched.tick(reg, ctx, 0.05f);

    REQUIRE(log.size() == 3u);
    CHECK(log[0] == "Alpha");
    CHECK(log[1] == "Bravo");
    CHECK(log[2] == "Charlie");
}

TEST_CASE("Sequential order is stable across multiple ticks") {
    std::vector<std::string> log;
    SystemScheduler sched;
    EntityRegistry  reg;
    SwarmContext    ctx;

    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("Z", log)) == SchedulerStatus::Ok);
    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("M", log)) == SchedulerStatus::Ok);
    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("A", log)) == SchedulerStatus::Ok);
    REQUIRE(sched.build_graph() == SchedulerStatus::Ok);

    for (int i = 0; i < 5; ++i) {
        sched.tick(reg, ctx, 0.05f);
    }

    REQUIRE(log.size() == 15u);
    for (int i = 0; i < 5; ++i) {
        CHECK(log[i * 3 + 0] == "A");
        CHECK(log[i * 3 + 1] == "M");
        CHECK(log[i * 3 + 2] == "Z");
    }
}

TEST_CASE("build_graph called twice without changes returns AlreadyBuilt") {
    std::vector<std::string> log;
    SystemScheduler sched;
    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("X", log))
            == SchedulerStatus::Ok);

    REQUIRE(sched.build_graph() == SchedulerStatus::Ok);
    CHECK(sched.build_graph() == SchedulerStatus::AlreadyBuilt);
}

TEST_CASE("register_system after build invalidates the built state") {
    std::vector<std::string> log;
    SystemScheduler sched;
    EntityRegistry  reg;
    SwarmContext    ctx;

    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("B", log))
            == SchedulerStatus::Ok);
    REQUIRE(sched.build_graph() == SchedulerStatus::Ok);
    REQUIRE(sched.is_built());

    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("A", log))
            == SchedulerStatus::Ok);
    CHECK_FALSE(sched.is_built());   // invalidated by registration

    REQUIRE(sched.build_graph() == SchedulerStatus::Ok);
    sched.tick(reg, ctx, 0.05f);

    REQUIRE(log.size() == 2u);
    CHECK(log[0] == "A");
    CHECK(log[1] == "B");
}

TEST_CASE("empty scheduler: build + tick is a no-op") {
    SystemScheduler sched;
    EntityRegistry  reg;
    SwarmContext    ctx;

    REQUIRE(sched.build_graph() == SchedulerStatus::Ok);
    CHECK(sched.is_built());

    // No systems → tick does nothing.
    sched.tick(reg, ctx, 0.05f);
    CHECK(sched.system_count() == 0u);
}

TEST_CASE("SwarmContext is forwarded unchanged to each system tick") {
    struct ContextProbe : System {
        SwarmContext& captured;
        float&        captured_dt;
        explicit ContextProbe(SwarmContext& c, float& dt) : captured(c), captured_dt(dt) {}
        SystemDescriptor describe() const override { return {"Probe", {}, {}, {}, {}}; }
        void tick(EntityRegistry&, const SwarmContext& ctx, float dt) override {
            captured = ctx;
            captured_dt = dt;
        }
    };

    SwarmContext captured;
    float        captured_dt = 0.0f;

    SystemScheduler sched;
    REQUIRE(sched.register_system(std::make_unique<ContextProbe>(captured, captured_dt))
            == SchedulerStatus::Ok);
    REQUIRE(sched.build_graph() == SchedulerStatus::Ok);

    EntityRegistry reg;
    SwarmContext ctx{
        /*swarm_id=*/      mith::SwarmID{0x42},
        /*elapsed_time_s=*/1.25f,
        /*delta_time_s=*/  0.05f,
        /*tick_count=*/    7u,
    };
    sched.tick(reg, ctx, 0.05f);

    CHECK(captured.swarm_id == 0x42u);
    CHECK(captured.elapsed_time_s == doctest::Approx(1.25f));
    CHECK(captured.delta_time_s == doctest::Approx(0.05f));
    CHECK(captured.tick_count == 7u);
    CHECK(captured_dt == doctest::Approx(0.05f));
}

TEST_CASE("SystemDescriptor preserves declared hazards across both axes") {
    std::vector<std::string> log;
    SystemDescriptor desc{
        /*name=*/             "",   // overwritten by ctor
        /*reads_components=*/ {mith::component_id<int>(), mith::component_id<float>()},
        /*writes_components=*/{mith::component_id<double>()},
        /*reads_resources=*/  {ResourceID::NeighbourTable, ResourceID::TransportRx},
        /*writes_resources=*/ {ResourceID::TransportTx},
    };

    TaggingSystem sys("HazardProbe", log, desc);
    const auto d = sys.describe();

    CHECK(d.name == "HazardProbe");
    REQUIRE(d.reads_components.size() == 2u);
    CHECK(d.reads_components[0] == mith::component_id<int>());
    CHECK(d.reads_components[1] == mith::component_id<float>());
    REQUIRE(d.writes_components.size() == 1u);
    CHECK(d.writes_components[0] == mith::component_id<double>());
    REQUIRE(d.reads_resources.size() == 2u);
    CHECK(d.reads_resources[0] == ResourceID::NeighbourTable);
    CHECK(d.reads_resources[1] == ResourceID::TransportRx);
    REQUIRE(d.writes_resources.size() == 1u);
    CHECK(d.writes_resources[0] == ResourceID::TransportTx);
}

TEST_CASE("ResourceID built-in values are stable and distinct") {
    static_assert(static_cast<std::uint16_t>(ResourceID::NeighbourTable) == 0u);
    static_assert(static_cast<std::uint16_t>(ResourceID::TransportTx)    == 1u);
    static_assert(static_cast<std::uint16_t>(ResourceID::TransportRx)    == 2u);
    static_assert(static_cast<std::uint16_t>(ResourceID::First_User)     == 0x1000u);
}

TEST_CASE("Sequential scheduler runs systems with declared hazards in name order") {
    // In Sequential mode hazards are documentation only — they do not affect
    // ordering. Lexicographic name order is the rule regardless of declared
    // R/W. This test confirms two systems with overlapping hazards run in
    // name order, not registration order.
    std::vector<std::string> log;
    SystemScheduler sched;

    SystemDescriptor writer_desc;
    writer_desc.writes_components = {mith::component_id<int>()};

    SystemDescriptor reader_desc;
    reader_desc.reads_components = {mith::component_id<int>()};

    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("Writer", log, writer_desc))
            == SchedulerStatus::Ok);
    REQUIRE(sched.register_system(std::make_unique<TaggingSystem>("Reader", log, reader_desc))
            == SchedulerStatus::Ok);
    REQUIRE(sched.build_graph() == SchedulerStatus::Ok);

    EntityRegistry reg;
    SwarmContext   ctx;
    sched.tick(reg, ctx, 0.05f);

    REQUIRE(log.size() == 2u);
    CHECK(log[0] == "Reader");    // lexicographic
    CHECK(log[1] == "Writer");
}
