#pragma once

// The MeshCore LoRa personality as the T-Pager board sees it: the MeshCore
// service, tunnel peer discovery (Bind / Bind request, rns-gateway
// docs/GRP_DATA_TUNNEL.md section 2.2) and its serial commands. RNS packet
// transport over the tunnel is added in phase 3.
//
// Same threading rule as Service: all calls on the device-service owner task
// (begin() may also run during setup(), before ownership is handed over).

#include "MeshCoreService.h"
#include "TunnelCodec.h"
#include "config/UserConfig.h"

namespace handheld::meshcore {

class Personality {
public:
    Personality(BoardRadio& radio, FlashStore& flash) : _service(radio, flash) {}

    bool begin(const UserSettings& settings);
    void loop();
    void stop() { _service.stop(); }
    bool online() const { return _service.running(); }

    // Board serial commands; returns false for characters it does not own.
    bool serialCommand(char command);
    static constexpr const char* kSerialHelp = "M meshcore-status  B bind-request  A advert";

private:
    struct Peer {
        uint8_t publicKey[tunnel::kPublicKeySize] = {};
        char name[tunnel::kMaxNameBytes + 1] = {};
        bool router = false;
        bool used = false;
        uint32_t lastSeenMs = 0;
    };
    static constexpr size_t kMaxPeers = 8;

    void onData(DataType type, const uint8_t* body, size_t length);
    void onBind(const tunnel::Bind& bind);
    void onFragment(const tunnel::Fragment& fragment);
    bool sendBind(bool request);
    void rememberPeer(const tunnel::Bind& bind);
    void printStatus() const;

    Service _service;
    Peer _peers[kMaxPeers];
    char _nodeName[kNodeNameMax] = {};
    uint32_t _bindReplyAtMs = 0;     // 0: no reply pending
    uint32_t _lastBindSentMs = 0;
    bool _bindSent = false;
};

// Settings -> service configuration. The channel name is a local label only
// (MeshCore identifies a channel by its key), so an empty one gets a default.
Config configFrom(const UserSettings& settings);

} // namespace handheld::meshcore
