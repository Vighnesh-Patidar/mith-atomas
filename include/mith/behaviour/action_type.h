#pragma once

// Action type identifiers — see ARCHITECTURE.md §6.1
//
// ActionTypeID is a flat uint32 namespace. Well-known built-in IDs are
// reserved in the low range (0..6 below); bits 7..31 are reserved for
// future built-ins that PermissionMaskComponent's 32-bit bitmask (§4.4)
// can still gate. User-defined action types start at actions::CUSTOM
// (= 0x1000); per-user-action gating is the application's responsibility.
//
// The full Action struct + handler dispatch (§6.4) lands alongside the
// §6 implementation work — this header carries only the IDs because
// PermissionMaskComponent (§4.4) needs them now.

#include <cstdint>

namespace mith {

using ActionTypeID = std::uint32_t;

namespace actions {

inline constexpr ActionTypeID IDLE     = 0;
inline constexpr ActionTypeID MOVE     = 1;
inline constexpr ActionTypeID HOVER    = 2;
inline constexpr ActionTypeID TRANSMIT = 3;
inline constexpr ActionTypeID SCAN     = 4;
inline constexpr ActionTypeID REGROUP  = 5;
inline constexpr ActionTypeID FOLLOW   = 6;

// Bits 7..31 reserved for future built-ins that PermissionMaskComponent's
// 32-bit bitmask still covers. ActionTypeID values in [32, CUSTOM) are
// reserved-but-unallocated — currently rejected by the permission mask.
inline constexpr ActionTypeID BUILTIN_MAX = 31;

// User-defined action types start here.
inline constexpr ActionTypeID CUSTOM = 0x1000;

} // namespace actions

} // namespace mith
