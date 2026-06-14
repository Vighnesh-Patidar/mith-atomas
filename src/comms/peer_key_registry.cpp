#include "mith/comms/peer_key_registry.h"

#include <cstring>

namespace mith {

bool PeerKeyRegistry::try_pin(const HierarchicalID& hid, const IdentityKey& key) {
    auto it = keys_.find(hid);
    if (it == keys_.end()) {
        keys_.emplace(hid, key);
        return true;
    }
    // hid is already pinned — accept only if the new key matches.
    return std::memcmp(it->second.public_key.data(),
                       key.public_key.data(),
                       IdentityKey::PUBLIC_KEY_LEN) == 0;
}

const IdentityKey* PeerKeyRegistry::find(const HierarchicalID& hid) const noexcept {
    auto it = keys_.find(hid);
    return it == keys_.end() ? nullptr : &it->second;
}

std::size_t PeerKeyRegistry::size() const noexcept {
    return keys_.size();
}

void PeerKeyRegistry::clear() noexcept {
    keys_.clear();
}

} // namespace mith
