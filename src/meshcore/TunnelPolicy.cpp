#include "TunnelPolicy.h"

#include <string.h>

#include "RnsPacket.h"

namespace handheld::meshcore::tunnel {

namespace {
// Throttle key: the first 10 bytes of the destination the packet is about.
bool announceKey(const uint8_t* packet, size_t length, uint8_t (&key)[DiscoveryThrottle::kKeyBytes]) {
    uint8_t destination[rns::kAddressSize];
    if (!rns::destination(packet, length, destination)) return false;
    memcpy(key, destination, sizeof(key));
    return true;
}
bool pathRequestKey(const uint8_t* packet, size_t length, uint8_t (&key)[DiscoveryThrottle::kKeyBytes]) {
    uint8_t target[rns::kAddressSize];
    if (!rns::pathRequestTarget(packet, length, target)) return false;
    memcpy(key, target, sizeof(key));
    return true;
}
} // namespace

DiscoveryThrottle::Entry* DiscoveryThrottle::find(Entry* table, size_t count, const uint8_t* key) {
    for (size_t i = 0; i < count; ++i)
        if (table[i].used && memcmp(table[i].key, key, kKeyBytes) == 0) return &table[i];
    return nullptr;
}

// Existing entry, a free one, or the least recently touched (ages are wrap-safe).
DiscoveryThrottle::Entry& DiscoveryThrottle::claim(Entry* table, size_t count, const uint8_t* key, uint32_t nowMs) {
    if (Entry* found = find(table, count, key)) return *found;
    Entry* slot = &table[0];
    for (size_t i = 0; i < count; ++i) {
        if (!table[i].used) { slot = &table[i]; break; }
        if (nowMs - table[i].lastMs > nowMs - slot->lastMs) slot = &table[i];
    }
    *slot = Entry{};
    slot->used = true;
    memcpy(slot->key, key, kKeyBytes);
    return *slot;
}

bool DiscoveryThrottle::allowOutbound(const uint8_t* packet, size_t length, uint32_t nowMs) {
    uint8_t key[kKeyBytes];
    if (rns::isAnnounce(packet, length) && announceKey(packet, length, key)) return allowAnnounce(key, nowMs);
    if (rns::isPathRequest(packet, length) && pathRequestKey(packet, length, key)) return allowPathRequest(key, nowMs);
    return true;
}

bool DiscoveryThrottle::allowAnnounce(const uint8_t* key, uint32_t nowMs) {
    if (_limits.announceIntervalMs == 0) return true;
    Entry* last = find(_announces, kEntries, key);
    if (!consumeBypass(key, nowMs) && last && nowMs - last->lastMs < _limits.announceIntervalMs) {
        ++_announcesSuppressed;
        return false;
    }
    claim(_announces, kEntries, key, nowMs).lastMs = nowMs;
    return true;
}

bool DiscoveryThrottle::allowPathRequest(const uint8_t* key, uint32_t nowMs) {
    if (_limits.pathRequestIntervalMs == 0) return true;
    Entry* entry = find(_pathRequests, kEntries, key);
    if (entry && nowMs - entry->firstMs < _limits.pathRequestBurstMs) {
        entry->lastMs = nowMs;           // inside the burst window
        return true;
    }
    if (entry && nowMs - entry->lastMs < _limits.pathRequestIntervalMs) {
        ++_pathRequestsSuppressed;
        return false;
    }
    Entry& fresh = claim(_pathRequests, kEntries, key, nowMs);
    fresh.firstMs = fresh.lastMs = nowMs;
    return true;
}

bool DiscoveryThrottle::consumeBypass(const uint8_t* key, uint32_t nowMs) {
    Entry* entry = find(_bypass, kBypassEntries, key);
    if (!entry) return false;
    entry->used = false;
    return int32_t(nowMs - entry->lastMs) < 0;
}

void DiscoveryThrottle::noteInbound(const uint8_t* packet, size_t length, uint32_t nowMs) {
    uint8_t key[kKeyBytes];
    if (rns::isAnnounce(packet, length) && announceKey(packet, length, key)) {
        if (_limits.announceIntervalMs) claim(_announces, kEntries, key, nowMs).lastMs = nowMs;
        return;
    }
    if (!rns::isPathRequest(packet, length) || !pathRequestKey(packet, length, key)) return;
    if (_limits.pathRequestIntervalMs) {
        // Start past the burst allowance, so our own request for the same
        // target waits a full interval instead of riding the burst.
        Entry& entry = claim(_pathRequests, kEntries, key, nowMs);
        entry.firstMs = nowMs - _limits.pathRequestBurstMs;
        entry.lastMs = nowMs;
    }
    // Our transport may answer this request with an announce: let it through.
    claim(_bypass, kBypassEntries, key, nowMs).lastMs = nowMs + _limits.pathResponseBypassMs;
}

void AirtimeBudget::roll(uint32_t nowMs) {
    if (!_started || nowMs - _windowStartMs >= kWindowMs) {
        _started = true;
        _windowStartMs = nowMs;
        _used = 0;
    }
}

bool AirtimeBudget::admit(bool discovery, uint32_t estimatedBytes, uint32_t nowMs) {
    if (_bytesPerHour == 0) return true;
    roll(nowMs);
    const uint32_t limit = discovery ? _bytesPerHour - _bytesPerHour / 5 : _bytesPerHour;
    if (_used + estimatedBytes > limit) {
        ++_shed;
        return false;
    }
    return true;
}

void AirtimeBudget::note(uint32_t bytes, uint32_t nowMs) {
    roll(nowMs);
    _used += bytes;
}

} // namespace handheld::meshcore::tunnel
