// flocking_demo — 10 robots in a SimBus running BeaconSystem +
// FlockingSystem + KinematicsSystem.
//
// Robots start in a loose 5x2 grid with small random initial velocities,
// run for 10 seconds of sim time at 20 Hz, and emit a JSON-line per tick
// to stdout describing every robot's current position. Pipe into
// tools/visualiser/visualise.py to animate:
//
//   ./build/examples/flocking_demo/flocking_demo | python3 tools/visualiser/visualise.py

#include "mith/comms/beacon_system.h"
#include "mith/core/builtin_components.h"
#include "mith/core/json_writer.h"
#include "mith/core/registry.h"
#include "mith/core/world.h"
#include "mith/sim/sim_bus.h"
#include "mith/systems/flocking_system.h"
#include "mith/systems/kinematics_system.h"

#include <cstdio>
#include <memory>
#include <random>
#include <vector>

int main() {
    using namespace mith;

    sim::SimBusConfig bus_cfg;
    bus_cfg.tick_rate_hz = 20.0f;
    bus_cfg.comm_range_m = 30.0f;
    sim::SimBus bus(bus_cfg);

    constexpr int N_ROBOTS = 10;
    constexpr int N_TICKS  = 200;   // 10 s at 20 Hz

    std::vector<std::unique_ptr<World>> worlds;
    worlds.reserve(N_ROBOTS);

    // Deterministic seed for reproducibility.
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> jitter(-0.5f, 0.5f);
    std::uniform_real_distribution<float> vinit(-1.0f, 1.0f);

    FlockingSystem::Params flock_p;
    flock_p.separation_radius_m = 2.0f;
    flock_p.separation_weight   = 2.0f;
    flock_p.alignment_weight    = 0.8f;
    flock_p.cohesion_weight     = 0.6f;
    flock_p.max_speed_mps       = 5.0f;

    for (int i = 0; i < N_ROBOTS; ++i) {
        auto w = bus.create_world(SwarmID{1});

        w->register_system(std::make_unique<BeaconSystem>(*w));
        w->register_system(std::make_unique<FlockingSystem>(*w, flock_p));
        w->register_system(std::make_unique<KinematicsSystem>());

        w->init();

        // 5x2 grid with jitter; random initial velocity.
        auto& pos = w->registry().get<PositionComponent>(w->self_id());
        pos.x = (i % 5) * 5.0f + jitter(rng);
        pos.y = (i / 5) * 5.0f + jitter(rng);

        auto& vel = w->registry().get<VelocityComponent>(w->self_id());
        vel.vx = vinit(rng);
        vel.vy = vinit(rng);

        worlds.push_back(std::move(w));
    }

    // Run + emit JSON lines.
    for (int tick = 0; tick <= N_TICKS; ++tick) {
        JsonWriter w;
        w.begin_object();
        w.key("tick");
        w.write_u64(static_cast<std::uint64_t>(tick));
        w.key("robots");
        w.begin_array();
        for (const auto& world : worlds) {
            const auto& pos =
                world->registry().get<PositionComponent>(world->self_id());
            w.begin_object();
            w.key("id"); w.write_string(world->identity().to_string());
            w.key("x");  w.write_f64(static_cast<double>(pos.x));
            w.key("y");  w.write_f64(static_cast<double>(pos.y));
            w.end_object();
        }
        w.end_array();
        w.end_object();
        w.newline();

        const auto& line = w.str();
        std::fwrite(line.data(), 1, line.size(), stdout);

        if (tick < N_TICKS) bus.advance(1);
    }

    return 0;
}
