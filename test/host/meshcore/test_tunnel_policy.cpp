// Host unit tests for src/meshcore/RnsPacket.h and TunnelPolicy.{h,cpp}.
// Packets are built per the Reticulum Manual, section 6.7.3 "Wire Format".
// Build and run: make -C test/host/meshcore

#include <cstdio>
#include <cstring>
#include <vector>

#include "RnsPacket.h"
#include "TunnelPolicy.h"

using namespace handheld::meshcore;
using namespace handheld::meshcore::tunnel;

static int failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            ++failures;                                                          \
        }                                                                        \
    } while (0)

using Bytes = std::vector<uint8_t>;

static Bytes address(uint8_t fill) { return Bytes(16, fill); }

// flags: packet type bits 0-1, destination type bits 2-3, header type bit 6.
static Bytes packet(uint8_t packetType, uint8_t destinationType, const Bytes& destination,
                    const Bytes* transport = nullptr, const Bytes& data = Bytes(20, 0x55)) {
    Bytes p;
    p.push_back(uint8_t((transport ? 0x40 : 0) | (destinationType << 2) | packetType));
    p.push_back(0);                                             // hops
    if (transport) p.insert(p.end(), transport->begin(), transport->end());
    p.insert(p.end(), destination.begin(), destination.end());
    p.push_back(0);                                             // context
    p.insert(p.end(), data.begin(), data.end());
    return p;
}

static Bytes announce(uint8_t dest, const Bytes* transport = nullptr) {
    return packet(1, 0, address(dest), transport);              // ANNOUNCE, SINGLE
}

static Bytes pathRequest(uint8_t target) {
    Bytes data = address(target);                               // queried destination
    Bytes tag(16, 0x77);
    data.insert(data.end(), tag.begin(), tag.end());
    return packet(0, 2, address(0x6b), nullptr, data);          // DATA, PLAIN, well-known hash
}

static bool allow(DiscoveryThrottle& t, const Bytes& p, uint32_t now) {
    return t.allowOutbound(p.data(), p.size(), now);
}

static void headerParsing() {
    const Bytes a = announce(0x11);
    uint8_t out[16];
    CHECK(rns::isAnnounce(a.data(), a.size()) && !rns::isPathRequest(a.data(), a.size()));
    CHECK(rns::destination(a.data(), a.size(), out) && out[0] == 0x11);
    const Bytes relay = address(0xee);
    const Bytes relayed = announce(0x22, &relay);
    CHECK(rns::destination(relayed.data(), relayed.size(), out) && out[0] == 0x22);   // second address
    const Bytes pr = pathRequest(0x33);
    CHECK(pr.size() == 51);                                    // manual: path request is 51 bytes
    CHECK(rns::pathRequestTarget(pr.data(), pr.size(), out) && out[0] == 0x33);
    Bytes ifac = a;
    ifac[0] |= 0x80;
    CHECK(!rns::destination(ifac.data(), ifac.size(), out));   // IFAC never crosses the tunnel
    CHECK(!rns::destination(a.data(), 18, out));               // truncated
}

static void announceThrottle() {
    DiscoveryThrottle t;
    CHECK(allow(t, announce(1), 0));
    CHECK(!allow(t, announce(1), 599999));                     // within 10 minutes
    CHECK(allow(t, announce(2), 1000));                        // other destination
    CHECK(allow(t, announce(1), 600000));
    CHECK(t.announcesSuppressed() == 1);
}

static void relayedAnnouncesAreKeyedByDestination() {
    DiscoveryThrottle t;
    const Bytes relay = address(0xee);
    CHECK(allow(t, announce(1, &relay), 0));
    CHECK(allow(t, announce(2, &relay), 1));                   // same transport, different destination
}

static void pathRequestBurstThenRate() {
    DiscoveryThrottle t;
    CHECK(allow(t, pathRequest(5), 0));
    CHECK(allow(t, pathRequest(5), 30000));                    // inside the 60 s burst
    CHECK(allow(t, pathRequest(5), 59999));
    CHECK(!allow(t, pathRequest(5), 60000));                   // burst over, last was 1 ms ago
    CHECK(allow(t, pathRequest(6), 60000));                    // keyed by target, not the shared hash
    CHECK(allow(t, pathRequest(5), 119999));                   // a full interval after the last
    CHECK(t.pathRequestsSuppressed() == 1);
}

static void heardTrafficPreMarks() {
    DiscoveryThrottle t;
    const Bytes a = announce(7), pr = pathRequest(8);
    t.noteInbound(a.data(), a.size(), 0);
    CHECK(!allow(t, announce(7), 1000));                       // another node already announced it
    t.noteInbound(pr.data(), pr.size(), 0);
    CHECK(!allow(t, pathRequest(8), 1000));                    // no burst allowance for ours
    CHECK(allow(t, pathRequest(8), 60000));
}

static void pathResponseBypass() {
    DiscoveryThrottle t;
    CHECK(allow(t, announce(9), 0));
    const Bytes pr = pathRequest(9);
    t.noteInbound(pr.data(), pr.size(), 1000);
    CHECK(allow(t, announce(9), 2000));                        // answering the request
    CHECK(!allow(t, announce(9), 3000));                       // bypass is single-use
    t.noteInbound(pr.data(), pr.size(), 100000);
    CHECK(!allow(t, announce(9), 115001));                     // bypass expired (15 s)
}

static void zeroIntervalsDisable() {
    DiscoveryThrottle t;
    DiscoveryThrottle::Limits off;
    off.announceIntervalMs = 0;
    off.pathRequestIntervalMs = 0;
    t.setLimits(off);
    for (uint32_t i = 0; i < 5; ++i) {
        CHECK(allow(t, announce(1), i));
        CHECK(allow(t, pathRequest(1), 100000 + i));
    }
}

static void otherTrafficAlwaysPasses() {
    DiscoveryThrottle t;
    const Bytes data = packet(0, 0, address(3));               // DATA to SINGLE
    for (int i = 0; i < 3; ++i) CHECK(allow(t, data, 0));
}

static void tablesEvictLeastRecent() {
    DiscoveryThrottle t;
    for (uint32_t i = 0; i < DiscoveryThrottle::kEntries + 1; ++i) CHECK(allow(t, announce(uint8_t(i + 1)), i));
    CHECK(allow(t, announce(1), 100));                         // destination 1 was evicted
    CHECK(!allow(t, announce(3), 101));                        // destination 3 still tracked
}

static void airtimeBudget() {
    AirtimeBudget b;
    CHECK(b.admit(true, 1000000, 0));                          // 0 = unlimited
    b.setBytesPerHour(1000);
    CHECK(b.admit(true, 800, 0));                              // discovery limit is 80% = 800
    b.note(700, 0);
    CHECK(!b.admit(true, 101, 10));                            // 801 > 800
    CHECK(b.admit(false, 300, 10));                            // data may use all 1000
    b.note(300, 10);
    CHECK(!b.admit(false, 1, 20));
    CHECK(b.shed() == 2);
    CHECK(b.admit(true, 800, AirtimeBudget::kWindowMs));       // a new hour
    CHECK(b.usedThisWindow() == 0);
}

static void meshHopsAreAdded() {
    Bytes a = announce(0x61);
    a[1] = 2;                                                  // two Reticulum hops so far
    rns::addMeshHops(a.data(), a.size(), 7);
    CHECK(a[1] == 9);
    rns::addMeshHops(a.data(), a.size(), 0);
    CHECK(a[1] == 9);
    a[1] = 100;
    rns::addMeshHops(a.data(), a.size(), 64);                  // clamps, never wraps
    CHECK(a[1] == rns::kMaxHopsBeforeIngest);
    uint8_t runt[1] = {0};
    rns::addMeshHops(runt, 1, 5);                              // too short: untouched
    CHECK(runt[0] == 0);
}

static void onAirEstimate() {
    CHECK(grpDataOnAirBytes(165) == 5 + 176);                  // 3 + 165 = 168 -> 11 blocks
    CHECK(grpDataOnAirBytes(46) == 5 + 64);                    // Bind request from the spec example
}

int main() {
    headerParsing();
    announceThrottle();
    relayedAnnouncesAreKeyedByDestination();
    pathRequestBurstThenRate();
    heardTrafficPreMarks();
    pathResponseBypass();
    zeroIntervalsDisable();
    otherTrafficAlwaysPasses();
    tablesEvictLeastRecent();
    airtimeBudget();
    meshHopsAreAdded();
    onAirEstimate();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("meshcore tunnel policy: PASS\n");
    return 0;
}
