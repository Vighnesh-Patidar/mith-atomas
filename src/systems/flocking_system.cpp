#include "mith/systems/flocking_system.h"

#include "mith/comms/neighbour_table.h"
#include "mith/core/builtin_components.h"
#include "mith/core/registry.h"
#include "mith/core/world.h"

#include <cmath>
#include <cstddef>

namespace mith {

FlockingSystem::FlockingSystem(World& world) noexcept
    : FlockingSystem(world, Params{}) {}

FlockingSystem::FlockingSystem(World& world, Params params) noexcept
    : nt_(&world.neighbour_table())
    , params_(params) {}

SystemDescriptor FlockingSystem::describe() const {
    return SystemDescriptor{
        /*name=*/             "FlockingSystem",
        /*reads_components=*/ {component_id<PositionComponent>(),
                                component_id<VelocityComponent>()},
        /*writes_components=*/{component_id<VelocityComponent>()},
        /*reads_resources=*/  {ResourceID::NeighbourTable},
        /*writes_resources=*/ {},
    };
}

void FlockingSystem::tick(EntityRegistry& registry,
                          const SwarmContext& /*ctx*/,
                          float delta_time) {
    if (!nt_ || nt_->empty()) return;

    const EntityID self = registry.self_id();
    const auto&    self_pos = registry.get<PositionComponent>(self);
    auto&          self_vel = registry.get<VelocityComponent>(self);

    // Accumulators.
    float sep_x = 0.0f, sep_y = 0.0f, sep_z = 0.0f;     // separation
    float align_x = 0.0f, align_y = 0.0f, align_z = 0.0f; // sum of velocities
    float coh_x = 0.0f, coh_y = 0.0f, coh_z = 0.0f;     // sum of positions
    std::size_t n = 0;

    const float sep_r_sq = params_.separation_radius_m * params_.separation_radius_m;

    for (const auto& nb : *nt_) {
        const float dx = self_pos.x - nb.position.x;
        const float dy = self_pos.y - nb.position.y;
        const float dz = self_pos.z - nb.position.z;
        const float dist_sq = dx*dx + dy*dy + dz*dz;

        // Separation: only within separation_radius. Weight by 1/dist so
        // close neighbours push harder than far ones.
        if (dist_sq > 0.0f && dist_sq < sep_r_sq) {
            const float dist = std::sqrt(dist_sq);
            sep_x += dx / dist;
            sep_y += dy / dist;
            sep_z += dz / dist;
        }

        // Alignment + cohesion: averaged over all neighbours.
        align_x += nb.velocity.vx;
        align_y += nb.velocity.vy;
        align_z += nb.velocity.vz;
        coh_x   += nb.position.x;
        coh_y   += nb.position.y;
        coh_z   += nb.position.z;
        ++n;
    }

    // n is guaranteed > 0 because we returned early on empty().
    const float inv_n = 1.0f / static_cast<float>(n);
    align_x = align_x * inv_n - self_vel.vx;
    align_y = align_y * inv_n - self_vel.vy;
    align_z = align_z * inv_n - self_vel.vz;
    coh_x   = coh_x   * inv_n - self_pos.x;
    coh_y   = coh_y   * inv_n - self_pos.y;
    coh_z   = coh_z   * inv_n - self_pos.z;

    // v += dt * (w_sep * sep + w_align * align + w_coh * coh)
    self_vel.vx += delta_time * (params_.separation_weight * sep_x
                               + params_.alignment_weight  * align_x
                               + params_.cohesion_weight   * coh_x);
    self_vel.vy += delta_time * (params_.separation_weight * sep_y
                               + params_.alignment_weight  * align_y
                               + params_.cohesion_weight   * coh_y);
    self_vel.vz += delta_time * (params_.separation_weight * sep_z
                               + params_.alignment_weight  * align_z
                               + params_.cohesion_weight   * coh_z);

    // Clamp speed.
    const float speed_sq = self_vel.vx * self_vel.vx
                         + self_vel.vy * self_vel.vy
                         + self_vel.vz * self_vel.vz;
    const float max_sq = params_.max_speed_mps * params_.max_speed_mps;
    if (speed_sq > max_sq && speed_sq > 0.0f) {
        const float scale = params_.max_speed_mps / std::sqrt(speed_sq);
        self_vel.vx *= scale;
        self_vel.vy *= scale;
        self_vel.vz *= scale;
    }
}

} // namespace mith
