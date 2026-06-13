#pragma once

// EntityRegistry — see ARCHITECTURE.md §4.3 and §4.1
//
// At v0.1, the registry holds exactly one entity (§3.2 N=1, SELF_ENTITY).
// Every component flows through the same unified registration path; the
// CRTP base (HotComponent / ColdComponent) is a storage hint that v0.1
// largely ignores (single slot per component either way) and v0.5 N>1
// will specialise on.
//
// Registration is policy-gated by ComponentRegistrationPolicy (§4.1, §13.5):
//
//   Open           — registration allowed any time. Test / sim only.
//   LockAfterInit  — default. Registration after lock() is rejected.
//   BuiltInOnly    — only register_builtin_component<T>() is accepted;
//                    public register_component<T>() is hard-rejected.
//                    The EW lockdown posture from §13.5.
//
// Public register_component<T>() always tags the registration as
// ComponentOrigin::User. The Built_In origin is reachable only through
// register_builtin_component<T>(), which the World::init() flow uses for
// the §4.4 built-in component list. Both APIs are exposed for now
// because World does not yet exist; once it does, builtin registration
// becomes friend-restricted.
//
// Type IDs are FNV-1a 64 over the type name (§4.1, §15 — no RTTI). The
// registry detects collisions at registration time via type_tag<T> address
// equality (a per-template inline-constexpr variable whose address is
// unique per T), returning TypeIdCollision rather than silently aliasing.
//
// Errors are not exceptions (§15). Misuse on the access path (get / emplace
// without registration, or on a non-self entity at N=1) is a programming
// bug — the registry aborts with a structured message rather than ignoring
// or returning a sentinel.

#include "mith/core/component.h"
#include "mith/core/entity.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace mith {

class TraceSink;   // fwd — defined in trace_sink.h. Registry only forward-
                   // declares it; only registry.cpp pulls the full header.

enum class ComponentRegistrationPolicy : std::uint8_t {
    Open           = 0,
    LockAfterInit  = 1,
    BuiltInOnly    = 2,
};

enum class RegistrationStatus : std::uint8_t {
    Ok                = 0,
    AlreadyRegistered = 1,   // same T re-registered; treated as idempotent
    TypeIdCollision   = 2,   // different T hashes to a registered ID
    PolicyForbidsUser = 3,   // BuiltInOnly rejected a User registration
    PolicyLocked      = 4,   // LockAfterInit rejected post-lock registration
};

namespace detail {

// Per-template-instantiation unique address. Two T instantiations have
// distinct &type_tag_value<T> addresses; this gives us collision detection
// without RTTI.
template<typename T>
inline constexpr char type_tag_value = 0;

template<typename T>
constexpr const void* type_tag() noexcept {
    return &type_tag_value<T>;
}

class ComponentStoreBase {
public:
    virtual ~ComponentStoreBase() noexcept = default;

    virtual ComponentTypeID type_id()  const noexcept = 0;
    virtual ComponentOrigin origin()   const noexcept = 0;
    virtual const void*     type_tag() const noexcept = 0;
    virtual bool            has()      const noexcept = 0;
    virtual void            remove()   noexcept       = 0;
};

template<typename T>
class ComponentStore final : public ComponentStoreBase {
public:
    static_assert(is_well_formed_component_v<T>,
                  "ComponentStore<T>: T must inherit exactly one of "
                  "HotComponent<T> or ColdComponent<T>");

    explicit ComponentStore(ComponentOrigin origin) noexcept
        : origin_(origin) {}

    ComponentTypeID type_id()  const noexcept override { return component_id<T>(); }
    ComponentOrigin origin()   const noexcept override { return origin_; }
    const void*     type_tag() const noexcept override { return ::mith::detail::type_tag<T>(); }
    bool            has()      const noexcept override { return value_.has_value(); }
    void            remove()   noexcept       override { value_.reset(); }

    T&       get()       noexcept { return *value_; }
    const T& get() const noexcept { return *value_; }

    void emplace(T&& v) noexcept(std::is_nothrow_move_constructible_v<T>) {
        value_.emplace(std::move(v));
    }

private:
    std::optional<T> value_;
    ComponentOrigin  origin_;
};

[[noreturn]] void registry_assert_fail(const char* msg) noexcept;

} // namespace detail

class EntityRegistry {
public:
    explicit EntityRegistry(ComponentRegistrationPolicy policy
                              = ComponentRegistrationPolicy::LockAfterInit) noexcept;

    // Public path. Always ComponentOrigin::User. Rejected under BuiltInOnly.
    template<typename T>
    RegistrationStatus register_component() {
        return register_impl<T>(ComponentOrigin::User);
    }

    // Privileged path. Used by the runtime to register §4.4 built-ins in
    // World::init(). Not friend-restricted yet because World does not exist;
    // future refactor will hide it from user code.
    template<typename T>
    RegistrationStatus register_builtin_component() {
        return register_impl<T>(ComponentOrigin::Built_In);
    }

    void lock() noexcept;
    bool is_locked() const noexcept;

    constexpr EntityID self_id() const noexcept { return SELF_ENTITY; }

    template<typename T>
    bool is_registered() const noexcept {
        return stores_.find(component_id<T>()) != stores_.end();
    }

    // Requires T to be registered. Aborts otherwise.
    template<typename T>
    ComponentOrigin origin_of() const noexcept {
        const auto it = stores_.find(component_id<T>());
        if (it == stores_.end()) {
            detail::registry_assert_fail("origin_of<T>: component not registered");
        }
        return it->second->origin();
    }

    template<typename T>
    bool has(EntityID id) const noexcept {
        if (id != SELF_ENTITY) return false;
        const auto it = stores_.find(component_id<T>());
        if (it == stores_.end()) return false;
        return it->second->has();
    }

    // Requires T registered AND emplaced AND id == SELF_ENTITY. Aborts otherwise.
    template<typename T>
    T& get(EntityID id) {
        if (id != SELF_ENTITY) {
            detail::registry_assert_fail("get<T>: id != SELF_ENTITY (v0.1 N=1)");
        }
        auto it = stores_.find(component_id<T>());
        if (it == stores_.end()) {
            detail::registry_assert_fail("get<T>: component not registered");
        }
        auto* typed = static_cast<detail::ComponentStore<T>*>(it->second.get());
        if (!typed->has()) {
            detail::registry_assert_fail("get<T>: component not emplaced");
        }
        return typed->get();
    }

    // Requires T registered AND id == SELF_ENTITY. Aborts otherwise.
    template<typename T>
    void emplace(EntityID id, T&& component) {
        if (id != SELF_ENTITY) {
            detail::registry_assert_fail("emplace<T>: id != SELF_ENTITY (v0.1 N=1)");
        }
        auto it = stores_.find(component_id<T>());
        if (it == stores_.end()) {
            detail::registry_assert_fail("emplace<T>: component not registered");
        }
        auto* typed = static_cast<detail::ComponentStore<T>*>(it->second.get());
        typed->emplace(std::move(component));
    }

    template<typename T>
    void remove(EntityID id) noexcept {
        if (id != SELF_ENTITY) return;
        auto it = stores_.find(component_id<T>());
        if (it == stores_.end()) return;
        it->second->remove();
    }

    std::size_t registered_count() const noexcept;

    ComponentRegistrationPolicy policy() const noexcept;

    // Audit / observability sink (§4.1, §14.4). Nullable; default is unset
    // (no emission). When set, every successful registration — both User
    // and Built_In — emits a `component_registered` event at TraceLevel::Info
    // with fields: origin, type_name, type_id (hex), tick (0 — registration
    // is pre-init).
    //
    // Failed registrations (AlreadyRegistered, TypeIdCollision, policy
    // rejections) do NOT emit. Lifecycle: caller owns the sink and must
    // ensure it outlives this registry.
    void       set_trace_sink(TraceSink* sink) noexcept;
    TraceSink* trace_sink() const noexcept;

private:
    template<typename T>
    RegistrationStatus register_impl(ComponentOrigin origin) {
        static_assert(is_well_formed_component_v<T>,
                      "register_*_component<T>: T must inherit exactly one of "
                      "HotComponent<T> or ColdComponent<T>");

        // Policy gating: lock state checked first (it's the policy-independent
        // part), then per-policy rules.
        if (locked_ && policy_ == ComponentRegistrationPolicy::LockAfterInit) {
            return RegistrationStatus::PolicyLocked;
        }
        if (origin == ComponentOrigin::User
            && policy_ == ComponentRegistrationPolicy::BuiltInOnly) {
            return RegistrationStatus::PolicyForbidsUser;
        }

        const ComponentTypeID id = component_id<T>();
        const auto existing = stores_.find(id);
        if (existing != stores_.end()) {
            // Same ID — distinguish "same T re-registered" from a hash collision
            // by comparing type_tag addresses (RTTI-free, per §15).
            if (existing->second->type_tag() == detail::type_tag<T>()) {
                return RegistrationStatus::AlreadyRegistered;
            }
            return RegistrationStatus::TypeIdCollision;
        }

        stores_.emplace(id, std::make_unique<detail::ComponentStore<T>>(origin));
        emit_registered_event_(origin, type_name<T>(), id);
        return RegistrationStatus::Ok;
    }

    // Implemented in registry.cpp so the registry header doesn't have to
    // pull mith/core/trace_sink.h. Safe to call with sink_ == nullptr (no-op).
    void emit_registered_event_(ComponentOrigin   origin,
                                std::string_view  type_name_,
                                ComponentTypeID   type_id) noexcept;

    std::unordered_map<ComponentTypeID, std::unique_ptr<detail::ComponentStoreBase>> stores_;
    ComponentRegistrationPolicy policy_;
    bool                        locked_ = false;
    TraceSink*                  sink_   = nullptr;
};

} // namespace mith
