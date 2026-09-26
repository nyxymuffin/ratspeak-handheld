// Host tests for src/meshcore/TunnelInterface.{h,cpp}: the slot-0 driver
// contract (leases, receipts, queueing, maintenance) over a fake MeshCore link.
// Build and run: make -C test/host/meshcore

#include <cstdio>
#include <cstring>
#include <vector>

#include "TunnelInterface.h"

using namespace handheld::meshcore::tunnel;
using handheld::TxLease;
using handheld::TxOffer;
using handheld::TxReceipt;
using handheld::TxReceiptEvent;

static int failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            ++failures;                                                          \
        }                                                                        \
    } while (0)

using Bytes = std::vector<uint8_t>;

class FakeLink : public TunnelLink {
public:
    bool linkOnline() const override { return online; }
    bool sendBody(const uint8_t* body, size_t length) override {
        if (refuse) { --refuse; return false; }
        sent.emplace_back(body, body + length);
        return true;
    }
    uint32_t bodyAirtimeMs(size_t) const override { return airtime; }
    bool floodReach() const override { return flood; }
    bool senderPrefix(uint8_t out[kSenderPrefix]) const override {
        const uint8_t own[kSenderPrefix] = {0x3a, 0x7c, 0x01, 0xe9};
        memcpy(out, own, kSenderPrefix);
        return true;
    }
    uint32_t nowMs() const override { return now; }
    uint32_t random32() override { return ++ids; }
    int lastRssi() const override { return -90; }
    float lastSnr() const override { return 5.5f; }

    bool online = true;
    bool flood = false;
    int refuse = 0;
    uint32_t airtime = 400;
    uint32_t now = 1000;
    uint32_t ids = 0x100;
    std::vector<Bytes> sent;
};

// Stands in for RustInterfacePump: validates leases and records receipts.
struct FakePump {
    static bool validate(void* context, const TxLease& lease) {
        auto* self = static_cast<FakePump*>(context);
        const bool expired = lease.receiptSlot < 16 && self->expired[lease.receiptSlot];
        return lease.generation && !expired;
    }
    static bool receipt(void* context, TxReceipt r, TxReceiptEvent event) {
        static_cast<FakePump*>(context)->events.push_back({r.slot, event});
        return true;
    }
    TxLease lease(uint8_t slot) const {
        TxLease l;
        l.generation = 1;
        l.receiptGeneration = 1;
        l.receiptSlot = slot;
        return l;
    }
    struct Event { uint8_t slot; TxReceiptEvent event; };
    std::vector<Event> events;
    bool expired[16] = {};
};

struct Fixture {
    FakeLink link;
    FakePump pump;
    TunnelInterface tunnel{link};
    std::vector<Bytes> received;
    Fixture() {
        tunnel.setTxValidator(&pump, &FakePump::validate);
        tunnel.setReceiptHook(&pump, &FakePump::receipt);
        tunnel.setRawSink([this](const uint8_t* d, size_t n) { received.emplace_back(d, d + n); });
        tunnel.start();
    }
    TxOffer offer(const Bytes& p, uint8_t slot) { return tunnel.offerLeased(p.data(), p.size(), pump.lease(slot)); }
    void advance(uint32_t ms) { link.now += ms; tunnel.loop(); }
    bool last(uint8_t slot, TxReceiptEvent event) const {
        return !pump.events.empty() && pump.events.back().slot == slot && pump.events.back().event == event;
    }
};

// A DATA packet to a SINGLE destination (Reticulum Manual 6.7.3), `length` bytes.
static Bytes dataPacket(size_t length, uint8_t seed) {
    Bytes p(length);
    for (size_t i = 0; i < length; ++i) p[i] = uint8_t(seed + i);
    p[0] = 0x00;   // DATA, SINGLE, one address
    return p;
}

static Bytes announcePacket(uint8_t dest) {
    Bytes p(120, 0x44);
    p[0] = 0x01;   // ANNOUNCE, SINGLE
    p[1] = 0;
    memset(p.data() + 2, dest, 16);
    return p;
}

static void singleFragmentStartsImmediately() {
    Fixture f;
    const Bytes p = dataPacket(100, 1);
    CHECK(f.offer(p, 1) == TxOffer::Started);
    CHECK(f.last(1, TxReceiptEvent::Started));
    CHECK(f.link.sent.size() == 1);
    Fragment frag;
    CHECK(decodeFragment(f.link.sent[0].data(), f.link.sent[0].size(), frag));
    CHECK(frag.total == 1 && frag.payloadLength == 100 && memcmp(frag.payload, p.data(), 100) == 0);
    CHECK(f.tunnel.counters().packetsSent == 1);
}

static void fullPacketIsPaced() {
    Fixture f;
    const Bytes p = dataPacket(500, 2);
    CHECK(f.offer(p, 1) == TxOffer::Started);
    CHECK(f.link.sent.size() == 1);
    f.advance(2499);
    CHECK(f.link.sent.size() == 1);                              // zero-hop gap is 2.5 s
    f.advance(1);
    CHECK(f.link.sent.size() == 2);
    f.advance(2500); f.advance(2500);
    CHECK(f.link.sent.size() == 4);
    f.advance(10000);
    CHECK(f.link.sent.size() == 4 && f.tunnel.counters().packetsSent == 1);
    // Reassembling what was sent gives the packet back.
    Reassembler r;
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    for (const auto& body : f.link.sent) {
        Fragment frag;
        decodeFragment(body.data(), body.size(), frag);
        r.accept(frag, 0, out, length);
    }
    CHECK(length == 500 && memcmp(out, p.data(), 500) == 0);
}

static void busyTunnelQueuesThenStarts() {
    Fixture f;
    CHECK(f.offer(dataPacket(300, 3), 1) == TxOffer::Started);  // 2 fragments
    CHECK(f.offer(dataPacket(50, 4), 2) == TxOffer::Queued);
    CHECK(f.pump.events.size() == 1);                           // no receipt yet for slot 2
    f.advance(2500);                                            // 2nd fragment of packet 1
    CHECK(f.pump.events.size() == 1);
    f.advance(2500);                                            // packet 2 starts
    CHECK(f.last(2, TxReceiptEvent::Started));
    CHECK(f.link.sent.size() == 3);
}

static void fullQueueBlocksWithoutThrottling() {
    Fixture f;
    CHECK(f.offer(dataPacket(500, 5), 1) == TxOffer::Started);
    for (uint8_t s = 2; s < 2 + TunnelInterface::kQueueDepth; ++s)
        CHECK(f.offer(dataPacket(60, s), s) == TxOffer::Queued);
    const Bytes a = announcePacket(0x21);
    CHECK(f.offer(a, 9) == TxOffer::Blocked);                   // caller keeps it
    CHECK(f.tunnel.throttle().announcesSuppressed() == 0);
    // Drain everything, then the retried announce must be accepted.
    for (int i = 0; i < 20; ++i) f.advance(2500);
    const TxOffer retry = f.offer(a, 9);
    CHECK(retry == TxOffer::Started || retry == TxOffer::Queued);
}

static void throttledAnnounceIsRejected() {
    Fixture f;
    CHECK(f.offer(announcePacket(0x31), 1) == TxOffer::Started);
    f.advance(5000);
    CHECK(f.offer(announcePacket(0x31), 2) == TxOffer::Rejected);  // 10-minute announce throttle
    CHECK(f.last(2, TxReceiptEvent::Dropped));
    CHECK(f.tunnel.counters().refusedByPolicy == 1);
}

// Frames from the pump's drain carry no receipt. A false sendLeased with a
// live lease makes the pump retry and block slot 0, so a policy refusal must
// report the packet as consumed.
static void receiptlessPolicyRefusalIsConsumed() {
    Fixture f;
    TxLease plain = f.pump.lease(1);
    plain.receiptSlot = UINT8_MAX;                              // no receipt (drainOutbound frame)
    const Bytes a = announcePacket(0x41);
    CHECK(f.tunnel.sendLeased(a.data(), a.size(), plain));      // sent
    f.advance(5000);
    CHECK(f.tunnel.sendLeased(a.data(), a.size(), plain));      // throttled, but consumed
    CHECK(f.tunnel.counters().refusedByPolicy == 1);
    CHECK(f.pump.events.empty());
    plain.generation = 0;                                       // retired by Rust
    CHECK(!f.tunnel.sendLeased(a.data(), a.size(), plain));     // a dead lease is still a plain failure
}

static void budgetRefusalDoesNotThrottle() {
    Fixture f;
    TunnelInterface::Settings tight;
    tight.airtimeBytesPerHour = 100;                            // discovery limit 80 bytes: nothing fits
    f.tunnel.configure(tight);
    const Bytes a = announcePacket(0x51);
    CHECK(f.offer(a, 1) == TxOffer::Rejected);
    CHECK(f.tunnel.budget().shed() == 1);
    f.tunnel.configure(TunnelInterface::Settings{});            // budget restored
    CHECK(f.offer(a, 2) == TxOffer::Started);                   // not held by the throttle
}

static FakePump* reenterPump = nullptr;
static TunnelInterface* reenterTunnel = nullptr;
static TxOffer reenterResult = TxOffer::Started;

static void offersDuringCallbacksAreBlocked() {
    Fixture f;
    reenterPump = &f.pump;
    reenterTunnel = &f.tunnel;
    f.tunnel.setReceiptHook(&f.pump, [](void* context, TxReceipt r, TxReceiptEvent event) {
        FakePump::receipt(context, r, event);
        const Bytes p = dataPacket(20, 1);
        reenterResult = reenterTunnel->offerLeased(p.data(), p.size(), reenterPump->lease(7));
        return true;
    });
    CHECK(f.offer(dataPacket(20, 2), 1) == TxOffer::Started);
    CHECK(reenterResult == TxOffer::Blocked);                   // the pump retries it later
}

static void stopInsideDropCallback() {
    Fixture f;
    reenterTunnel = &f.tunnel;
    f.tunnel.setReceiptHook(&f.pump, [](void* context, TxReceipt r, TxReceiptEvent event) {
        FakePump::receipt(context, r, event);
        if (event == TxReceiptEvent::Dropped) reenterTunnel->stop();
        return true;
    });
    CHECK(f.offer(dataPacket(500, 3), 1) == TxOffer::Started);
    CHECK(f.offer(dataPacket(40, 4), 2) == TxOffer::Queued);
    CHECK(f.offer(dataPacket(40, 5), 3) == TxOffer::Queued);
    f.pump.expired[2] = true;
    f.advance(1);                                               // drop of slot 2 stops the tunnel
    size_t dropped = 0;
    for (const auto& e : f.pump.events) dropped += e.event == TxReceiptEvent::Dropped;
    CHECK(dropped == 2);                                        // each queued packet exactly once
    CHECK(!f.tunnel.isOnline());
}

static void leasesAreChecked() {
    Fixture f;
    f.pump.expired[3] = true;
    CHECK(f.offer(dataPacket(40, 6), 3) == TxOffer::Rejected);  // dead on arrival
    CHECK(f.last(3, TxReceiptEvent::Dropped));
    CHECK(f.offer(dataPacket(300, 7), 4) == TxOffer::Started);
    CHECK(f.offer(dataPacket(40, 8), 5) == TxOffer::Queued);
    f.pump.expired[5] = true;                                   // Rust retired it while queued
    f.advance(1);
    CHECK(f.last(5, TxReceiptEvent::Dropped));
    CHECK(f.tunnel.counters().queuedDropped == 1);
}

static void refusedFragmentIsRetried() {
    Fixture f;
    f.link.refuse = 1;                                          // MeshCore packet pool full
    CHECK(f.offer(dataPacket(80, 9), 1) == TxOffer::Queued);
    CHECK(f.pump.events.empty());
    f.advance(1);                                               // pacing allows, link accepts now
    CHECK(f.last(1, TxReceiptEvent::Started));
    CHECK(f.link.sent.size() == 1);
}

static void maintenanceDropsQueue() {
    Fixture f;
    CHECK(f.offer(dataPacket(500, 10), 1) == TxOffer::Started);
    CHECK(f.offer(dataPacket(40, 11), 2) == TxOffer::Queued);
    f.tunnel.beginMaintenance();
    CHECK(f.last(2, TxReceiptEvent::Dropped));
    CHECK(f.tunnel.maintenanceDrained());
    const size_t before = f.link.sent.size();
    f.advance(10000);
    CHECK(f.link.sent.size() == before);                        // in-flight packet abandoned
    CHECK(f.offer(dataPacket(40, 12), 3) == TxOffer::Rejected);
}

static void offlineRejects() {
    Fixture f;
    f.link.online = false;
    CHECK(!f.tunnel.isOnline());
    CHECK(f.offer(dataPacket(40, 13), 1) == TxOffer::Rejected);
    CHECK(f.tunnel.bitrate() == 0);
}

static void receivesPackets() {
    Fixture f;
    const Bytes p = dataPacket(320, 14);
    const uint8_t peer[4] = {0xbb, 1, 2, 3};
    uint8_t body[kMaxBody];
    for (uint8_t i = 0; i < 3; ++i) {
        const size_t n = encodeFragment(peer, 77, i, p.data(), p.size(), body, sizeof(body));
        Fragment frag;
        decodeFragment(body, n, frag);
        f.tunnel.onFragment(frag);
    }
    CHECK(f.received.size() == 1 && f.received[0] == p);
    CHECK(f.tunnel.counters().packetsReceived == 1);
}

static void bitrateFollowsPacing() {
    Fixture f;
    CHECK(f.tunnel.bitrate() == 154u * 8 * 1000 / 2500);       // zero-hop: 492 bit/s
    f.link.flood = true;                                        // 400 ms x 12 = 4.8 s gap
    CHECK(f.tunnel.bitrate() == 154u * 8 * 1000 / 4800);
    CHECK(f.tunnel.txWaitBudgetMs(1) == (4 + 1) * 4800u);
}

int main() {
    singleFragmentStartsImmediately();
    fullPacketIsPaced();
    busyTunnelQueuesThenStarts();
    fullQueueBlocksWithoutThrottling();
    throttledAnnounceIsRejected();
    receiptlessPolicyRefusalIsConsumed();
    budgetRefusalDoesNotThrottle();
    offersDuringCallbacksAreBlocked();
    stopInsideDropCallback();
    leasesAreChecked();
    refusedFragmentIsRetried();
    maintenanceDropsQueue();
    offlineRejects();
    receivesPackets();
    bitrateFollowsPacing();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("meshcore tunnel interface: PASS\n");
    return 0;
}
