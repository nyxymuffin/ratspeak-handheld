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

// Board serial commands (DeviceDiagnostics falls through to the board for
// letters it does not own; a/m/t/d/r/i/p/q and the line letters are taken).
#define MESHCORE_SERIAL_HELP "N meshcore-status  B bind-request  V advert"

namespace handheld::meshcore {

static_assert(tunnel::kPublicKeySize == kPublicKeySize, "one MeshCore public key size");

class Personality {
public:
    Personality(BoardRadio& radio, FlashStore& flash) : _service(radio, flash) {}

    bool begin(const UserSettings& settings);
    void loop();
    void stop() { _service.stop(); _bindReplyAtMs = 0; }
    bool online() const { return _service.running(); }

    // Board serial commands (MESHCORE_SERIAL_HELP); false for other characters.
    bool serialCommand(char command);

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
    uint32_t _lastBindReplyMs = 0;   // last Bind (not Bind request) we sent
    bool _bindReplied = false;
};

// Settings -> service configuration. The channel name is a local label only
// (MeshCore identifies a channel by its key), so an empty one gets a default.
Config configFrom(const UserSettings& settings);

} // namespace handheld::meshcore
