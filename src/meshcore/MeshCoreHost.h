#pragma once

// The MeshCore node: BaseChatMesh reduced to what the RNS tunnel needs - one
// private data channel, adverts, and GRP_DATA send/receive. No CLI, ACL,
// contacts UI or room-server logic. Include from src/meshcore/*.cpp only.
//
// The shape follows genemichael/rns-gateway's MyMesh (channel join, scoped
// flood policy, send timeouts), written against MeshCore's API. Its echo ring
// is not needed: Mesh::sendFlood/sendZeroHop mark our own packets as seen, and
// every GRP_DATA tunnel fragment is unique.

#include <helpers/BaseChatMesh.h>

#include "MeshCoreRadio.h"
#include "MeshCoreTypes.h"

namespace handheld::meshcore {

class Host final : public BaseChatMesh {
public:
    // `meshHops`: MeshCore repeaters the packet crossed (flood path length; 0 zero-hop).
    using DataSink = void (*)(void* context, DataType type, const uint8_t* data, size_t length, uint8_t meshHops);

    Host(RadioAdapter& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc,
         mesh::PacketManager& packets, mesh::MeshTables& tables)
        : BaseChatMesh(radio, ms, rng, rtc, packets, tables) {}

    void setIdentity(const mesh::LocalIdentity& identity) { self_id = identity; }
    // An out-of-range hash size falls back to one byte: Mesh::sendFlood drops
    // (and leaks from the pool) any packet with a size outside 1..3.
    void setRouting(PathHashSize hashSize, ChannelReach reach);
    void setDataSink(DataSink sink, void* context) { _sink = sink; _sinkContext = context; }

    // Region for flooded packets. Empty or "*" floods unscoped; a public region
    // name ("name" or "#name") floods as that region, keyed as repeaters and the
    // companion firmware do (TransportKeyStore::getAutoKeyFor).
    bool setFloodScope(const char* name);
    bool floodScoped() const { return _scoped; }
    bool joinChannel(const char* name, const char* pskBase64);
    bool channelJoined() const { return _channel != nullptr; }
    bool sendData(DataType type, const uint8_t* data, size_t length);
    bool sendAdvert(const char* nodeName, bool flood);

    uint32_t dataReceived() const { return _dataReceived; }
    uint32_t dataSent() const { return _dataSent; }

protected:
    void onChannelDataRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint16_t type,
                           const uint8_t* data, size_t length) override;
    void sendFloodScoped(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint32_t delayMs) override;
    void sendFloodScoped(const ContactInfo& recipient, mesh::Packet* pkt, uint32_t delayMs) override;
    uint32_t calcFloodTimeoutMillisFor(uint32_t airtimeMs) const override;
    uint32_t calcDirectTimeoutMillisFor(uint32_t airtimeMs, uint8_t pathLength) const override;

    // Never add contacts from adverts: a contact would make BaseChatMesh ACK
    // direct messages and return paths, i.e. transmit on the tunnel's behalf.
    bool isAutoAddEnabled() const override { return false; }
    bool shouldAutoAddContactType(uint8_t) const override { return false; }

    // Not used by the tunnel: direct messages, contacts and requests.
    void onDiscoveredContact(ContactInfo&, bool, uint8_t, const uint8_t*) override {}
    ContactInfo* processAck(const uint8_t*) override { return nullptr; }
    void onContactPathUpdated(const ContactInfo&) override {}
    void onMessageRecv(const ContactInfo&, mesh::Packet*, uint32_t, const char*) override {}
    void onCommandDataRecv(const ContactInfo&, mesh::Packet*, uint32_t, const char*) override {}
    void onSignedMessageRecv(const ContactInfo&, mesh::Packet*, uint32_t, const uint8_t*, const char*) override {}
    void onSendTimeout() override {}
    void onChannelMessageRecv(const mesh::GroupChannel&, mesh::Packet*, uint32_t, const char*) override {}
    uint8_t onContactRequest(const ContactInfo&, uint32_t, const uint8_t*, uint8_t, uint8_t*) override { return 0; }
    void onContactResponse(const ContactInfo&, const uint8_t*, uint8_t) override {}

private:
    bool isOurChannel(const mesh::GroupChannel& channel) const;
    void sendScoped(mesh::Packet* pkt, uint32_t delayMs);
    void flood(mesh::Packet* pkt, uint32_t delayMs);
    uint16_t transportCode(const mesh::Packet* pkt) const;

    static constexpr size_t kScopeKeyBytes = 16;   // TransportKey::key
    uint8_t _scopeKey[kScopeKeyBytes] = {};
    bool _scoped = false;

    ChannelDetails* _channel = nullptr;
    PathHashSize _hashSize = PathHashSize::OneByte;
    ChannelReach _reach = ChannelReach::Flood;
    DataSink _sink = nullptr;
    void* _sinkContext = nullptr;
    uint32_t _dataReceived = 0;
    uint32_t _dataSent = 0;
};

} // namespace handheld::meshcore
