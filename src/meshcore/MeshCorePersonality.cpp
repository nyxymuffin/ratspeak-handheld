#include "MeshCorePersonality.h"

#include <esp_random.h>

namespace handheld::meshcore {

namespace {

constexpr const char* kDefaultChannelName = "Tunnel";
constexpr const char* kDefaultNodeName = "Ratspeak";
// Deferred Bind reply window (spec 2.2): wait 1-5 s so several nodes that hear
// the same request do not all answer at once, and do not repeat within 5 s.
constexpr uint32_t kBindReplyMinMs = 1000;
constexpr uint32_t kBindReplySpreadMs = 4000;
constexpr uint32_t kBindRepeatGuardMs = 5000;

void copyText(char* dest, size_t capacity, const String& text) {
    const size_t length = text.length() < capacity - 1 ? text.length() : capacity - 1;
    memcpy(dest, text.c_str(), length);
    dest[length] = '\0';
}

void printKeyPrefix(const uint8_t* key, size_t bytes) {
    for (size_t i = 0; i < bytes; ++i) Serial.printf("%02x", key[i]);
}

} // namespace

Config configFrom(const UserSettings& settings) {
    const MeshCoreSettings& mc = settings.meshcore;
    Config config;
    config.radio = {mc.frequency, mc.bandwidth, mc.spreadingFactor, mc.codingRate, mc.txPower};
    const String& name = !mc.nodeName.isEmpty() ? mc.nodeName
        : !settings.displayName.isEmpty() ? settings.displayName : String(kDefaultNodeName);
    copyText(config.nodeName, sizeof(config.nodeName), name);
    copyText(config.channelName, sizeof(config.channelName),
             mc.channelName.isEmpty() ? String(kDefaultChannelName) : mc.channelName);
    copyText(config.channelPsk, sizeof(config.channelPsk), mc.channelPsk);
    config.pathHashSize = static_cast<PathHashSize>(mc.pathHashSize);   // sanitized to 1..3
    config.channelReach = mc.floodChannel ? ChannelReach::Flood : ChannelReach::ZeroHop;
    return config;
}

bool Personality::begin(const UserSettings& settings) {
    const Config config = configFrom(settings);
    memcpy(_nodeName, config.nodeName, sizeof(_nodeName));
    _service.setDataSink([this](DataType type, const uint8_t* body, size_t length) {
        onData(type, body, length);
    });
    return _service.begin(config);
}

void Personality::loop() {
    _service.loop();
    if (_bindReplyAtMs && int32_t(millis() - _bindReplyAtMs) >= 0) {
        _bindReplyAtMs = 0;
        sendBind(false);
    }
}

void Personality::onData(DataType type, const uint8_t* body, size_t length) {
    if (type != DataType::RnsTunnel) return;
    tunnel::Kind kind;
    if (!tunnel::kindOf(body, length, kind)) {
        Serial.printf("[TUNNEL] rx unknown kind 0x%02x (%u bytes), dropped\n", length ? body[0] : 0, unsigned(length));
        return;
    }
    if (kind == tunnel::Kind::Fragment) {
        tunnel::Fragment fragment;
        if (tunnel::decodeFragment(body, length, fragment)) onFragment(fragment);
        else Serial.printf("[TUNNEL] rx malformed fragment (%u bytes)\n", unsigned(length));
        return;
    }
    tunnel::Bind bind;
    if (tunnel::decodeBind(body, length, bind)) onBind(bind);
    else Serial.printf("[TUNNEL] rx malformed bind (%u bytes)\n", unsigned(length));
}

void Personality::onFragment(const tunnel::Fragment& fragment) {
    // Phase 3 reassembles these into RNS packets; for now they are only logged.
    Serial.print("[TUNNEL] rx fragment from ");
    printKeyPrefix(fragment.sender, tunnel::kSenderPrefix);
    Serial.printf(" pkt=%08lx %u/%u len=%u\n", (unsigned long)fragment.packetId,
                  unsigned(fragment.index) + 1, unsigned(fragment.total), unsigned(fragment.payloadLength));
}

void Personality::onBind(const tunnel::Bind& bind) {
    uint8_t own[kPublicKeySize];
    if (_service.publicKey(own) && memcmp(own, bind.publicKey, kPublicKeySize) == 0) return;   // own echo
    Serial.printf("[TUNNEL] rx %s from '%s' key=", bind.request ? "bind-request" : "bind", bind.name);
    printKeyPrefix(bind.publicKey, 8);
    Serial.printf("... %s\n", bind.router ? "router" : "edge");
    rememberPeer(bind);
    if (!bind.request || _bindReplyAtMs) return;
    if (_bindSent && millis() - _lastBindSentMs < kBindRepeatGuardMs) return;
    _bindReplyAtMs = millis() + kBindReplyMinMs + esp_random() % kBindReplySpreadMs;
    if (_bindReplyAtMs == 0) _bindReplyAtMs = 1;   // 0 means "none pending"
}

void Personality::rememberPeer(const tunnel::Bind& bind) {
    Peer* slot = nullptr;
    for (auto& peer : _peers) {
        if (peer.used && memcmp(peer.publicKey, bind.publicKey, kPublicKeySize) == 0) { slot = &peer; break; }
        if (!slot && !peer.used) slot = &peer;
    }
    if (!slot) {   // table full: replace the least recently seen
        slot = &_peers[0];
        for (auto& peer : _peers) if (peer.lastSeenMs < slot->lastSeenMs) slot = &peer;
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
    if (sent) { _bindSent = true; _lastBindSentMs = millis(); }
    Serial.printf("[TUNNEL] tx %s %s\n", request ? "bind-request" : "bind", sent ? "queued" : "FAILED");
    return sent;
}

bool Personality::serialCommand(char command) {
    switch (command) {
    case 'M': printStatus(); return true;
    case 'B': sendBind(true); return true;
    case 'A': Serial.printf("[MESHCORE] advert %s\n", _service.sendAdvert(true) ? "queued" : "FAILED"); return true;
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
    for (const auto& peer : _peers) {
        if (!peer.used) continue;
        Serial.printf("[MESHCORE]   peer '%s' ", peer.name);
        printKeyPrefix(peer.publicKey, 8);
        Serial.printf("... %s, seen %lus ago\n", peer.router ? "router" : "edge",
                      (unsigned long)((millis() - peer.lastSeenMs) / 1000));
    }
}

} // namespace handheld::meshcore
