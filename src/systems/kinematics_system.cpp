#include "mith/systems/kinematics_system.h"

#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"

namespace mith {

SystemDescriptor KinematicsSystem::describe() const {
    return SystemDescriptor{
        /*name=*/             "KinematicsSystem",
        /*reads_components=*/ {component_id<VelocityComponent>()},
        /*writes_components=*/{component_id<PositionComponent>()},
        /*reads_resources=*/  {},
        /*writes_resources=*/ {},
    };
}

void KinematicsSystem::tick(EntityRegistry& registry,
                            const SwarmContext& /*ctx*/,
                            float delta_time) {
    const EntityID self = registry.self_id();
    const auto& vel = registry.get<VelocityComponent>(self);
    auto&       pos = registry.get<PositionComponent>(self);
    pos.x += vel.vx * delta_time;
    pos.y += vel.vy * delta_time;
    pos.z += vel.vz * delta_time;
}

} // namespace mith
