#include "MeshCorePersonality.h"

#include <esp_heap_caps.h>
#include <esp_random.h>
#include <new>

#include "config/MeshCoreRules.h"
#include "util/DisplayName.h"

namespace handheld::meshcore {

namespace {

constexpr const char* kDefaultChannelName = "Tunnel";
constexpr const char* kDefaultNodeName = "Ratspeak";
// Deferred Bind reply window (spec 2.2): wait 1-5 s so several nodes that hear
// the same request do not all answer at once, and do not repeat within 5 s.
constexpr uint32_t kBindReplyMinMs = 1000;
constexpr uint32_t kBindReplySpreadMs = 4000;
constexpr uint32_t kBindRepeatGuardMs = 5000;

// Copies a name, cut at a whole UTF-8 character within MeshCore's 31 bytes
// (the display name allows 16 characters, up to 64 bytes). Falls back when
// empty or not a valid MeshCore name.
void copyName(char (&dest)[kNodeNameMax], const String& text, const char* fallback) {
    const size_t length = displayNamePrefix(text.c_str(), text.length(), meshcore_rules::kMaxNameBytes);
    if (length == 0 || !meshcore_rules::validName(text.c_str(), length)) {
        strlcpy(dest, fallback, sizeof(dest));
        return;
    }
    memcpy(dest, text.c_str(), length);
    dest[length] = '\0';
}

void printKeyPrefix(const uint8_t* key, size_t bytes) {
    for (size_t i = 0; i < bytes; ++i) Serial.printf("%02x", key[i]);
}

// Names from the air are untrusted: keep control/escape bytes out of the log.
void printSafe(const char* text) {
    for (const char* p = text; *p; ++p) Serial.print(*p >= 0x20 && *p < 0x7F ? *p : '?');
}

} // namespace

Config configFrom(const UserSettings& settings) {
    const MeshCoreSettings& mc = settings.meshcore;
    Config config;
    config.radio = {mc.frequency, mc.bandwidth, mc.spreadingFactor, mc.codingRate, mc.txPower};
    copyName(config.nodeName, mc.nodeName.isEmpty() ? settings.displayName : mc.nodeName, kDefaultNodeName);
    copyName(config.channelName, mc.channelName, kDefaultChannelName);
    strlcpy(config.channelPsk, mc.channelPsk.c_str(), sizeof(config.channelPsk));
    strlcpy(config.floodScope, mc.floodScope.c_str(), sizeof(config.floodScope));
    config.pathHashSize = static_cast<PathHashSize>(mc.pathHashSize);   // sanitized to 1..3
    config.channelReach = mc.floodChannel ? ChannelReach::Flood : ChannelReach::ZeroHop;
    return config;
}

tunnel::TunnelInterface::Settings tunnelSettingsFrom(const MeshCoreSettings& mc) {
    tunnel::TunnelInterface::Settings t;
    t.throttle.announceIntervalMs = mc.announceHoldS * 1000UL;        // <= 86400 s, fits 32 bits
    t.throttle.pathRequestIntervalMs = mc.pathRequestHoldS * 1000UL;
    t.airtimeBytesPerHour = uint32_t(mc.airtimeBudgetKbH) * 1024UL;
    return t;
}

bool Personality::begin(const UserSettings& settings) {
    const Config config = configFrom(settings);
    memcpy(_nodeName, config.nodeName, sizeof(_nodeName));
    _flood = config.channelReach == ChannelReach::Flood;
    _service.setDataSink([this](DataType type, const uint8_t* body, size_t length, uint8_t meshHops) {
        onData(type, body, length, meshHops);
    });
    if (!_service.begin(config)) return false;
    if (!_tunnel) {
        // The reassembler and send queue are ~7 KB of buffers: keep them in PSRAM.
        void* memory = heap_caps_aligned_alloc(alignof(tunnel::TunnelInterface), sizeof(tunnel::TunnelInterface),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!memory) {
            Serial.println("[TUNNEL] No PSRAM for the tunnel; MeshCore runs without Reticulum");
            return true;
        }
        _tunnel = new (memory) tunnel::TunnelInterface(_link);
    }
    _tunnel->configure(tunnelSettingsFrom(settings.meshcore));
    _tunnel->start();
    return true;
}

void Personality::stop() {
    if (_tunnel) _tunnel->stop();
    _service.stop();
    _bindReplyAtMs = 0;
}

// ── TunnelLink ──────────────────────────────────────────────────────────────

bool Personality::Link::linkOnline() const { return _owner._service.channelJoined(); }

bool Personality::Link::sendBody(const uint8_t* body, size_t length) {
    return _owner._service.sendData(DataType::RnsTunnel, body, length);
}

uint32_t Personality::Link::bodyAirtimeMs(size_t bodyLength) const {
    return _owner._service.airtimeMs(tunnel::grpDataOnAirBytes(bodyLength));
}

bool Personality::Link::senderPrefix(uint8_t out[tunnel::kSenderPrefix]) const {
    uint8_t key[kPublicKeySize];
    if (!_owner._service.publicKey(key)) return false;
    memcpy(out, key, tunnel::kSenderPrefix);
    return true;
}

uint32_t Personality::Link::nowMs() const { return millis(); }
uint32_t Personality::Link::random32() { return esp_random(); }
int Personality::Link::lastRssi() const { return static_cast<int>(_owner._service.status().lastRssi); }
float Personality::Link::lastSnr() const { return _owner._service.status().lastSnr; }

void Personality::loop() {
    _service.loop();
    if (_bindReplyAtMs && int32_t(millis() - _bindReplyAtMs) >= 0) {
        _bindReplyAtMs = 0;
        sendBind(false);
    }
}

void Personality::onData(DataType type, const uint8_t* body, size_t length, uint8_t meshHops) {
    if (type != DataType::RnsTunnel) return;
    tunnel::Kind kind;
    if (!tunnel::kindOf(body, length, kind)) {
        Serial.printf("[TUNNEL] rx unknown kind 0x%02x (%u bytes), dropped\n", length ? body[0] : 0, unsigned(length));
        return;
    }
    if (kind == tunnel::Kind::Fragment) {
        tunnel::Fragment fragment;
        if (tunnel::decodeFragment(body, length, fragment)) onFragment(fragment, meshHops);
        else Serial.printf("[TUNNEL] rx malformed fragment (%u bytes)\n", unsigned(length));
        return;
    }
    tunnel::Bind bind;
    if (tunnel::decodeBind(body, length, bind)) onBind(bind);
    else Serial.printf("[TUNNEL] rx malformed bind (%u bytes)\n", unsigned(length));
}

void Personality::onFragment(const tunnel::Fragment& fragment, uint8_t meshHops) {
    if (_tunnel) _tunnel->onFragment(fragment, meshHops);
}

void Personality::onBind(const tunnel::Bind& bind) {
    uint8_t own[kPublicKeySize];
    if (_service.publicKey(own) && memcmp(own, bind.publicKey, kPublicKeySize) == 0) return;   // own echo
    Serial.printf("[TUNNEL] rx %s from '", bind.request ? "bind-request" : "bind");
    printSafe(bind.name);
    Serial.print("' key=");
    printKeyPrefix(bind.publicKey, 8);
    Serial.printf("... %s\n", bind.router ? "router" : "edge");
    rememberPeer(bind);
    if (!bind.request || _bindReplyAtMs) return;
    // Spec 2.2: no reply if we already sent a Bind within the window.
    if (_bindReplied && millis() - _lastBindReplyMs < kBindRepeatGuardMs) return;
    _bindReplyAtMs = millis() + kBindReplyMinMs + esp_random() % kBindReplySpreadMs;
    if (_bindReplyAtMs == 0) _bindReplyAtMs = 1;   // 0 means "none pending"
}

void Personality::rememberPeer(const tunnel::Bind& bind) {
    Peer* slot = nullptr;
    for (auto& peer : _peers) {
        if (peer.used && memcmp(peer.publicKey, bind.publicKey, kPublicKeySize) == 0) { slot = &peer; break; }
        if (!slot && !peer.used) slot = &peer;
    }
    if (!slot) {   // table full: replace the least recently seen (ages are wrap-safe)
        const uint32_t now = millis();
        slot = &_peers[0];
        for (auto& peer : _peers)
            if (now - peer.lastSeenMs > now - slot->lastSeenMs) slot = &peer;
    }
    memcpy(slot->publicKey, bind.publicKey, kPublicKeySize);
    memcpy(slot->name, bind.name, sizeof(slot->name));
    slot->router = bind.router;
    slot->used = true;
    slot->lastSeenMs = millis();
}

bool Personality::sendBind(bool request) {
    tunnel::Bind bind;
    if (!_service.publicKey(bind.publicKey)) return false;
    bind.request = request;
    bind.router = false;   // a handheld is an edge node
    const size_t nameLength = strnlen(_nodeName, tunnel::kMaxNameBytes);
    bind.nameLength = static_cast<uint8_t>(nameLength);
    memcpy(bind.name, _nodeName, nameLength);
    uint8_t body[tunnel::kMaxBody];
    const size_t length = tunnel::encodeBind(bind, body, sizeof(body));
    const bool sent = length && _service.sendData(DataType::RnsTunnel, body, length);
    if (sent && !request) { _bindReplied = true; _lastBindReplyMs = millis(); }
    Serial.printf("[TUNNEL] tx %s %s\n", request ? "bind-request" : "bind", sent ? "queued" : "FAILED");
    return sent;
}

bool Personality::serialCommand(char command) {
    switch (command) {
    case 'N': case 'n': printStatus(); return true;
    case 'B': case 'b': sendBind(true); return true;
    case 'V': case 'v':
        Serial.printf("[MESHCORE] advert %s\n", _service.sendAdvert(true) ? "queued" : "FAILED");
        return true;
    default: return false;
    }
}

void Personality::printStatus() const {
    const Status s = _service.status();
    Serial.printf("[MESHCORE] %s node='%s' key=%s channel=%s rx=%lu tx=%lu txfail=%lu data_rx=%lu data_tx=%lu "
                  "rssi=%.0f snr=%.1f\n",
                  s.online ? "online" : "offline", _nodeName, s.publicKeyPrefix,
                  s.channelJoined ? "joined" : "none", (unsigned long)s.rxPackets, (unsigned long)s.txPackets,
                  (unsigned long)s.txFailures, (unsigned long)s.dataReceived, (unsigned long)s.dataSent,
                  s.lastRssi, s.lastSnr);
    if (_tunnel) {
        const auto& c = _tunnel->counters();
        Serial.printf("[TUNNEL] %s %lu bit/s pkts tx=%lu rx=%lu frags tx=%lu refused=%lu dropped=%lu "
                      "announces-held=%lu path-req-held=%lu air=%luB/h shed=%lu assembling=%u "
                      "mesh-hops last=%u max=%u\n",
                      _tunnel->isOnline() ? "online" : "offline", (unsigned long)_tunnel->bitrate(),
                      (unsigned long)c.packetsSent, (unsigned long)c.packetsReceived,
                      (unsigned long)c.fragmentsSent, (unsigned long)c.refusedByPolicy,
                      (unsigned long)c.queuedDropped,
                      (unsigned long)_tunnel->throttle().announcesSuppressed(),
                      (unsigned long)_tunnel->throttle().pathRequestsSuppressed(),
                      (unsigned long)_tunnel->budget().usedThisWindow(), (unsigned long)_tunnel->budget().shed(),
                      unsigned(_tunnel->reassembler().pending()), unsigned(c.lastMeshHops),
                      unsigned(c.maxMeshHops));
    }
    for (const auto& peer : _peers) {
        if (!peer.used) continue;
        Serial.print("[MESHCORE]   peer '");
        printSafe(peer.name);
        Serial.print("' ");
        printKeyPrefix(peer.publicKey, 8);
        Serial.printf("... %s, seen %lus ago\n", peer.router ? "router" : "edge",
                      (unsigned long)((millis() - peer.lastSeenMs) / 1000));
    }
}

} // namespace handheld::meshcore
