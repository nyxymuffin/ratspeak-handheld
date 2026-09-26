#pragma once

// Airtime policy for RNS traffic entering the MeshCore tunnel: per-destination
// announce and path-request throttles and an hourly airtime budget. Pure and
// fixed-size; host-tested in test/host/meshcore/test_tunnel_policy.cpp.
//
// Semantics follow rns-gateway's MeshCoreInterface (rate_limit_ok,
// note_heard_on_mesh, air_budget_ok) so a handheld and a gateway sharing a
// channel treat discovery traffic the same way. These are local policy, not
// part of the wire format (GRP_DATA_TUNNEL.md section 4).

#include <stddef.h>
#include <stdint.h>

namespace handheld::meshcore::tunnel {

class DiscoveryThrottle {
public:
    struct Limits {
        uint32_t announceIntervalMs = 600000;    // per destination; 0 = off
        uint32_t pathRequestIntervalMs = 60000;  // per queried destination after the burst; 0 = off
        uint32_t pathRequestBurstMs = 60000;     // requests inside this window all pass
        uint32_t pathResponseBypassMs = 15000;   // an answering announce skips the announce throttle
    };
    static constexpr size_t kEntries = 32;
    static constexpr size_t kBypassEntries = 8;
    static constexpr size_t kKeyBytes = 10;      // destination prefix, as in rns-gateway

    void setLimits(const Limits& limits) { _limits = limits; }

    // An outbound RNS packet is about to enter the tunnel. false: hold it back.
    bool allowOutbound(const uint8_t* packet, size_t length, uint32_t nowMs);
    // An RNS packet arrived from the tunnel. Another node already carried this
    // announce or path request, so ours would be redundant; an inbound path
    // request also lets our answering announce through.
    void noteInbound(const uint8_t* packet, size_t length, uint32_t nowMs);

    uint32_t announcesSuppressed() const { return _announcesSuppressed; }
    uint32_t pathRequestsSuppressed() const { return _pathRequestsSuppressed; }

private:
    struct Entry {
        bool used = false;
        uint8_t key[kKeyBytes] = {};
        uint32_t firstMs = 0;   // path requests: start of the burst window
        uint32_t lastMs = 0;    // last sent; for bypass entries, the expiry
    };

    static Entry* find(Entry* table, size_t count, const uint8_t* key);
    static Entry& claim(Entry* table, size_t count, const uint8_t* key, uint32_t nowMs);
    bool allowAnnounce(const uint8_t* key, uint32_t nowMs);
    bool allowPathRequest(const uint8_t* key, uint32_t nowMs);
    bool consumeBypass(const uint8_t* key, uint32_t nowMs);

    Limits _limits;
    Entry _announces[kEntries];
    Entry _pathRequests[kEntries];
    Entry _bypass[kBypassEntries];
    uint32_t _announcesSuppressed = 0;
    uint32_t _pathRequestsSuppressed = 0;
};

// Rolling one-hour cap on bytes the tunnel puts on the air. Announces and path
// requests (recoverable on demand) shed at 80%; everything sheds at 100%.
class AirtimeBudget {
public:
    static constexpr uint32_t kWindowMs = 3600000;

    void setBytesPerHour(uint32_t bytes) { _bytesPerHour = bytes; }   // 0 = unlimited
    bool admit(bool discovery, uint32_t estimatedBytes, uint32_t nowMs);
    void note(uint32_t bytes, uint32_t nowMs);
    uint32_t usedThisWindow() const { return _used; }
    uint32_t shed() const { return _shed; }

private:
    void roll(uint32_t nowMs);

    uint32_t _bytesPerHour = 0;
    uint32_t _windowStartMs = 0;
    uint32_t _used = 0;
    uint32_t _shed = 0;
    bool _started = false;
};

// Estimated on-air size of one GRP_DATA tunnel packet carrying `bodyLength`
// bytes: MeshCore header and path length, channel hash and 2-byte MAC, and the
// AES ciphertext of data_type + len + body padded to 16 bytes (MeshCore spec
// section 8). Zero-hop path; flood paths add a few bytes per hop.
inline constexpr uint32_t grpDataOnAirBytes(size_t bodyLength) {
    return uint32_t(2 + 1 + 2 + ((3 + bodyLength + 15) / 16) * 16);
}

} // namespace handheld::meshcore::tunnel
