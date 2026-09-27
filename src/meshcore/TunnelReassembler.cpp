#include "TunnelReassembler.h"

#include <string.h>

namespace handheld::meshcore::tunnel {

bool Reassembler::sameSender(const uint8_t* a, const uint8_t* b) {
    return memcmp(a, b, kSenderPrefix) == 0;
}

size_t Reassembler::fragmentLength(const Assembly& a, uint8_t index) {
    return index + 1 == a.total ? a.lastLength : kMaxFragmentPayload;
}

// Spec 2.1: all fragments but the last are full, and the packet fits the MTU.
bool Reassembler::valid(const Fragment& f) {
    if (f.total == 0 || f.total > kMaxFragments || f.index >= f.total) return false;
    if (f.payloadLength == 0 || f.payloadLength > kMaxFragmentPayload) return false;
    if (f.index + 1 < f.total) return f.payloadLength == kMaxFragmentPayload;
    return size_t(f.total - 1) * kMaxFragmentPayload + f.payloadLength <= kMaxPacket;
}

Reassembler::Result Reassembler::accept(const Fragment& f, uint32_t nowMs, uint8_t (&out)[kMaxPacket],
                                        size_t& outLength, uint8_t meshHops, uint8_t* outMeshHops) {
    outLength = 0;
    if (!valid(f)) return Result::Rejected;
    if (recentlyCompleted(f, nowMs)) return Result::Duplicate;

    Assembly& a = slotFor(f, nowMs);
    if (a.received & (1u << f.index)) return Result::Duplicate;
    store(a, f, nowMs);
    if (meshHops > a.meshHops) a.meshHops = meshHops;
    if (a.received != fullMask(a.total)) mergeLossyCopies(a);
    if (a.received != fullMask(a.total)) return Result::Pending;
    if (outMeshHops) *outMeshHops = a.meshHops;
    outLength = finish(a, out, nowMs);
    return Result::Complete;
}

bool Reassembler::recentlyCompleted(const Fragment& f, uint32_t nowMs) {
    for (const auto& s : _seen)
        if (s.used && s.packetId == f.packetId && sameSender(s.sender, f.sender) &&
            int32_t(nowMs - s.untilMs) < 0) return true;
    return false;
}

Reassembler::Assembly& Reassembler::slotFor(const Fragment& f, uint32_t nowMs) {
    Assembly* free = nullptr;
    Assembly* oldest = &_slots[0];
    for (auto& a : _slots) {
        if (a.used && a.packetId == f.packetId && sameSender(a.sender, f.sender) && a.total == f.total) return a;
        if (!a.used && !free) free = &a;
        if (a.used && nowMs - a.updatedMs > nowMs - oldest->updatedMs) oldest = &a;
    }
    Assembly& slot = free ? *free : *oldest;
    if (!free) ++_evictions;   // a stale partial is less likely to finish than a new one
    slot = Assembly{};
    slot.used = true;
    memcpy(slot.sender, f.sender, kSenderPrefix);
    slot.packetId = f.packetId;
    slot.total = f.total;
    return slot;
}

void Reassembler::store(Assembly& a, const Fragment& f, uint32_t nowMs) {
    memcpy(a.data + size_t(f.index) * kMaxFragmentPayload, f.payload, f.payloadLength);
    if (f.index + 1 == f.total) a.lastLength = static_cast<uint8_t>(f.payloadLength);
    a.received |= static_cast<uint8_t>(1u << f.index);
    a.updatedMs = nowMs;
}

// Two copies prove they carry the same packet when they share the sender and
// fragment count and every fragment both hold is byte-identical. Encrypted
// payloads never match by accident. (rns-gateway, observed 2026-09-06.)
void Reassembler::mergeLossyCopies(Assembly& a) {
    for (auto& other : _slots) {
        if (&other == &a || !other.used || other.total != a.total || !sameSender(other.sender, a.sender)) continue;
        const uint8_t shared = a.received & other.received;
        if (!shared || (other.received & ~a.received) == 0) continue;
        bool same = true;
        for (uint8_t i = 0; i < a.total && same; ++i) {
            if (!(shared & (1u << i))) continue;
            const size_t length = fragmentLength(a, i);
            same = length == fragmentLength(other, i) &&
                   memcmp(a.data + size_t(i) * kMaxFragmentPayload,
                          other.data + size_t(i) * kMaxFragmentPayload, length) == 0;
        }
        if (!same) continue;
        for (uint8_t i = 0; i < a.total; ++i) {
            if (!(other.received & (1u << i)) || (a.received & (1u << i))) continue;
            const size_t length = fragmentLength(other, i);
            memcpy(a.data + size_t(i) * kMaxFragmentPayload, other.data + size_t(i) * kMaxFragmentPayload, length);
            if (i + 1 == a.total) a.lastLength = other.lastLength;
            a.received |= static_cast<uint8_t>(1u << i);
        }
        // `a` now holds everything `other` had, so the older copy is spent.
        if (other.meshHops > a.meshHops) a.meshHops = other.meshHops;
        ++_merged;
        other.used = false;
        if (a.received == fullMask(a.total)) return;
    }
}

void Reassembler::rememberCompleted(const Assembly& a, uint32_t nowMs) {
    Seen& s = _seen[_seenNext];
    _seenNext = (_seenNext + 1) % kSeenEntries;
    s.used = true;
    memcpy(s.sender, a.sender, kSenderPrefix);
    s.packetId = a.packetId;
    s.untilMs = nowMs + _limits.dedupTtlMs;
}

size_t Reassembler::finish(Assembly& a, uint8_t (&out)[kMaxPacket], uint32_t nowMs) {
    const size_t length = size_t(a.total - 1) * kMaxFragmentPayload + a.lastLength;
    memcpy(out, a.data, length);
    rememberCompleted(a, nowMs);
    a.used = false;
    return length;
}

void Reassembler::expire(uint32_t nowMs) {
    for (auto& a : _slots)
        if (a.used && nowMs - a.updatedMs > _limits.fragmentTimeoutMs) { a.used = false; ++_timeouts; }
    for (auto& s : _seen)
        if (s.used && int32_t(nowMs - s.untilMs) >= 0) s.used = false;
}

size_t Reassembler::pending() const {
    size_t count = 0;
    for (const auto& a : _slots) count += a.used ? 1 : 0;
    return count;
}

} // namespace handheld::meshcore::tunnel
