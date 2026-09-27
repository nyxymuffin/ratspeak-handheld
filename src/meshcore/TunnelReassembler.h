#pragma once

// Reassembles GRP_DATA tunnel fragments (rns-gateway docs/GRP_DATA_TUNNEL.md
// section 2.1) into RNS packets. Pure, fixed-size, host-tested in
// test/host/meshcore/test_tunnel_reassembler.cpp.
//
// Behaviour follows rns-gateway's MeshCoreInterface: key (sender, pkt_id),
// duplicate suppression for a short window after completion, 5-minute
// timeout, and completion of a packet from two lossy copies of the same bytes
// (a re-sent announce or path response arrives under a new pkt_id).

#include <stddef.h>
#include <stdint.h>

#include "TunnelCodec.h"

namespace handheld::meshcore::tunnel {

class Reassembler {
public:
    static constexpr size_t kMaxPacket = 500;   // Reticulum MTU
    static constexpr uint8_t kMaxFragments = (kMaxPacket + kMaxFragmentPayload - 1) / kMaxFragmentPayload;
    static constexpr size_t kSlots = 8;
    static constexpr size_t kSeenEntries = 32;

    enum class Result : uint8_t {
        Complete,    // `out` holds the packet
        Pending,     // stored, more fragments needed
        Duplicate,   // already held, or the packet completed recently
        Rejected,    // violates the spec (size, fragment count, short middle fragment)
    };

    struct Limits {
        uint32_t fragmentTimeoutMs = 300000;
        uint32_t dedupTtlMs = 30000;
    };

    void setLimits(const Limits& limits) { _limits = limits; }
    // `meshHops` is how many MeshCore repeaters this fragment crossed; on
    // Complete, `outMeshHops` (if given) is the most any fragment crossed.
    Result accept(const Fragment& fragment, uint32_t nowMs, uint8_t (&out)[kMaxPacket], size_t& outLength,
                  uint8_t meshHops = 0, uint8_t* outMeshHops = nullptr);
    // Drops timed-out assemblies and expired duplicate records.
    void expire(uint32_t nowMs);

    size_t pending() const;
    uint32_t timeouts() const { return _timeouts; }
    uint32_t evictions() const { return _evictions; }
    uint32_t mergedCopies() const { return _merged; }

private:
    struct Assembly {
        bool used = false;
        uint8_t sender[kSenderPrefix] = {};
        uint32_t packetId = 0;
        uint8_t total = 0;
        uint8_t received = 0;          // bit i: fragment i held
        uint8_t lastLength = 0;        // payload length of fragment total-1, once held
        uint8_t meshHops = 0;          // most MeshCore hops any held fragment crossed
        uint32_t updatedMs = 0;
        uint8_t data[kMaxPacket] = {};
    };
    struct Seen {
        bool used = false;
        uint8_t sender[kSenderPrefix] = {};
        uint32_t packetId = 0;
        uint32_t untilMs = 0;
    };

    static bool sameSender(const uint8_t* a, const uint8_t* b);
    static uint8_t fullMask(uint8_t total) { return static_cast<uint8_t>((1u << total) - 1); }
    static size_t fragmentLength(const Assembly& a, uint8_t index);
    static bool valid(const Fragment& fragment);

    bool recentlyCompleted(const Fragment& fragment, uint32_t nowMs);
    Assembly& slotFor(const Fragment& fragment, uint32_t nowMs);
    void store(Assembly& a, const Fragment& fragment, uint32_t nowMs);
    void mergeLossyCopies(Assembly& a);
    void rememberCompleted(const Assembly& a, uint32_t nowMs);
    size_t finish(Assembly& a, uint8_t (&out)[kMaxPacket], uint32_t nowMs);

    Limits _limits;
    Assembly _slots[kSlots];
    Seen _seen[kSeenEntries];
    size_t _seenNext = 0;
    uint32_t _timeouts = 0;
    uint32_t _evictions = 0;
    uint32_t _merged = 0;
};

} // namespace handheld::meshcore::tunnel
