#pragma once

// The MeshCore tunnel as Reticulum interface 0 (LoRaSlotDriver) in MeshCore
// mode. Whole RNS packets from the pump are admitted through the airtime
// policy, split into GRP_DATA fragments (GRP_DATA_TUNNEL.md 2.1) and sent at
// rns-gateway's pacing; inbound fragments are reassembled and handed to the
// pump. Pure: MeshCore is reached through TunnelLink, so the whole slot-0
// contract is host-tested (test/host/meshcore/test_tunnel_interface.cpp).
//
// Receipts follow LoRaInterface: Started when the first fragment of a packet
// goes to MeshCore (the rest follow regardless, like a split LoRa frame);
// Dropped when a queued packet is refused, expires or maintenance begins.
//
// Differences from LoRaInterface, by design:
// - A packet refused by the airtime policy is consumed (sendLeased returns
//   true), not retried: the pump would otherwise hold slot 0 behind it.
// - Maintenance abandons the in-flight packet and is drained at once; the
//   MeshCore service then aborts any fragment still on the air. A partial
//   packet is useless to the far side, and no receipt is owed for it.
// - The data channel is joined once at start and never dropped, so the
//   interface generation only changes on start().

#include <stddef.h>
#include <stdint.h>

#include "TunnelCodec.h"
#include "TunnelPolicy.h"
#include "TunnelReassembler.h"
#include "transport/LoRaSlotDriver.h"

namespace handheld::meshcore::tunnel {

// What the tunnel needs from the MeshCore side.
class TunnelLink {
public:
    virtual ~TunnelLink() = default;
    virtual bool linkOnline() const = 0;
    // Queue one GRP_DATA tunnel body on the data channel.
    virtual bool sendBody(const uint8_t* body, size_t length) = 0;
    // On-air time of one GRP_DATA packet carrying `bodyLength` bytes.
    virtual uint32_t bodyAirtimeMs(size_t bodyLength) const = 0;
    virtual bool floodReach() const = 0;
    virtual bool senderPrefix(uint8_t out[kSenderPrefix]) const = 0;
    virtual uint32_t nowMs() const = 0;
    virtual uint32_t random32() = 0;
    virtual int lastRssi() const = 0;
    virtual float lastSnr() const = 0;
};

class TunnelInterface final : public LoRaSlotDriver {
public:
    struct Settings {
        DiscoveryThrottle::Limits throttle;
        uint32_t airtimeBytesPerHour = 60 * 1024;    // rns-gateway default
        uint32_t fragmentGapMs = 2500;               // zero-hop spacing
        uint8_t floodGapAirtimes = 12;               // flood: clear ~4 hops of repeater echo
    };
    static constexpr size_t kQueueDepth = 4;

    explicit TunnelInterface(TunnelLink& link) : _link(link) {}

    void configure(const Settings& settings);
    void start();
    void stop();
    // A fragment from the data channel (Personality's GRP_DATA sink).
    void onFragment(const Fragment& fragment);

    // LoRaSlotDriver
    void setRawSink(RawSink sink) override { _rawSink = sink; }
    void setTxValidator(void* context, TxValidator validator) override;
    void setReceiptHook(void* context, handheld::TxReceiptHook hook) override;
    uint32_t generation() const override { return _generation; }
    bool isOnline() const override { return _online && _link.linkOnline(); }
    uint32_t bitrate() const override;
    uint32_t txWaitBudgetMs(uint32_t packets) const override;
    int lastRxRssi() const override { return _link.lastRssi(); }
    float lastRxSnr() const override { return _link.lastSnr(); }
    void loop() override;
    // The tunnel never owns a radio burst (MeshCore does), so blocking work is always safe.
    bool pollBeforeBlockingWork() override { return true; }
    handheld::TxOffer offerLeased(const uint8_t* data, size_t len, const handheld::TxLease& lease) override;
    bool sendLeased(const uint8_t* data, size_t len, const handheld::TxLease& lease) override;
    void beginMaintenance() override;
    void pollMaintenance() override {}
    // The in-flight packet is abandoned at maintenance: its later fragments are
    // useless without the rest, so nothing is left to drain.
    bool maintenanceDrained() const override { return _maintenance; }
    bool maintenanceFailed() const override { return false; }

    struct Counters {
        uint32_t packetsSent = 0;
        uint32_t fragmentsSent = 0;
        uint32_t packetsReceived = 0;
        uint32_t refusedByPolicy = 0;
        uint32_t queuedDropped = 0;
    };
    const Counters& counters() const { return _counters; }
    const DiscoveryThrottle& throttle() const { return _throttle; }
    const AirtimeBudget& budget() const { return _budget; }
    const Reassembler& reassembler() const { return _reassembler; }

private:
    struct Packet {
        uint8_t data[Reassembler::kMaxPacket] = {};
        size_t length = 0;
        handheld::TxLease lease;
    };
    struct Outgoing {
        Packet packet;
        uint32_t packetId = 0;
        uint8_t next = 0;
        uint8_t total = 0;
        bool active = false;
    };

    handheld::TxOffer offer(const uint8_t* data, size_t len, const handheld::TxLease& lease, bool& policyDrop);
    handheld::TxOffer reject(const handheld::TxLease& lease);
    bool admittedByPolicy(const uint8_t* data, size_t len);
    bool leaseLive(const handheld::TxLease& lease) const;
    bool pacingReady() const { return int32_t(_link.nowMs() - _nextTxMs) >= 0; }
    uint32_t gapMs(size_t bodyLength) const;
    bool begin(const uint8_t* data, size_t len, const handheld::TxLease& lease);
    bool sendNextFragment();
    void startQueued();
    void discardExpired();
    void dropQueued(size_t index, const char* cause);
    void notify(const handheld::TxLease& lease, handheld::TxReceiptEvent event);

    TunnelLink& _link;
    Settings _settings;
    DiscoveryThrottle _throttle;
    AirtimeBudget _budget;
    Reassembler _reassembler;
    RawSink _rawSink;
    void* _validatorContext = nullptr;
    TxValidator _validator = nullptr;
    void* _receiptContext = nullptr;
    handheld::TxReceiptHook _receiptHook = nullptr;

    Outgoing _current;
    Packet _queue[kQueueDepth];
    size_t _queued = 0;
    uint8_t _rxPacket[Reassembler::kMaxPacket] = {};
    uint32_t _nextTxMs = 0;
    uint32_t _lastExpireMs = 0;
    uint32_t _generation = 0;
    bool _online = false;
    bool _maintenance = false;
    bool _notifying = false;
    bool _changingOwner = false;
    Counters _counters;
};

} // namespace handheld::meshcore::tunnel
