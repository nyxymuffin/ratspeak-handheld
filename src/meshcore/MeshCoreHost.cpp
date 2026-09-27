#include "MeshCoreHost.h"

#include <SHA256.h>

#include "config/MeshCoreRules.h"

namespace handheld::meshcore {

static_assert(kMaxGroupData == MAX_GROUP_DATA_LENGTH, "MeshCoreTypes.h must track MeshCore.h");

// Base64 of a 32-byte key is 44 characters. decode_base64 writes the decoded
// bytes straight into the 32-byte channel secret, so longer input would overrun it.
static constexpr size_t kMaxPskBase64 = 44;

// Send timeouts as in MeshCore's companion firmware (examples/companion_radio).
static constexpr uint32_t kSendTimeoutBaseMs = 500;
static constexpr float kFloodTimeoutFactor = 16.0f;
static constexpr float kDirectPerHopFactor = 6.0f;
static constexpr uint32_t kDirectPerHopExtraMs = 250;

void Host::setRouting(PathHashSize hashSize, ChannelReach reach) {
    const uint8_t size = static_cast<uint8_t>(hashSize);
    _hashSize = size >= 1 && size <= 3 ? hashSize : PathHashSize::OneByte;
    _reach = reach;
}

bool Host::joinChannel(const char* name, const char* pskBase64) {
    if (_channel || !name || !*name || !pskBase64 || !*pskBase64) return false;
    if (strlen(pskBase64) > kMaxPskBase64) return false;
    // addChannel() rejects keys that do not decode to 16 or 32 bytes.
    _channel = addChannel(name, pskBase64);
    return _channel != nullptr;
}

bool Host::sendData(DataType type, const uint8_t* data, size_t length) {
    if (!_channel || !data || length == 0 || length > kMaxGroupData) return false;
    // OUT_PATH_UNKNOWN routes through sendFloodScoped(), i.e. our reach policy.
    const bool sent = sendGroupData(_channel->channel, nullptr, OUT_PATH_UNKNOWN,
                                    static_cast<uint16_t>(type), data, static_cast<int>(length));
    if (sent) ++_dataSent;
    return sent;
}

bool Host::sendAdvert(const char* nodeName, bool flooded) {
    mesh::Packet* pkt = createSelfAdvert(nodeName);
    if (!pkt) return false;
    if (flooded) flood(pkt, 0);
    else sendZeroHop(pkt);
    return true;
}

bool Host::setFloodScope(const char* name) {
    const size_t length = name ? strlen(name) : 0;
    _scoped = false;
    memset(_scopeKey, 0, sizeof(_scopeKey));
    if (meshcore_rules::unscopedFlood(name, length)) return true;
    if (!meshcore_rules::validFloodScope(name, length)) return false;
    // Key = first 16 bytes of SHA-256("#name"); the '#' is implied if absent
    // (RegionMap::getTransportKeysFor, TransportKeyStore::getAutoKeyFor).
    SHA256 sha;
    if (name[0] != '#') sha.update("#", 1);
    sha.update(name, length);
    sha.finalize(_scopeKey, sizeof(_scopeKey));
    _scoped = true;
    return true;
}

// HMAC-SHA256 over payload type + payload, truncated to 2 bytes; 0000 and FFFF
// are reserved (TransportKey::calcTransportCode).
uint16_t Host::transportCode(const mesh::Packet* pkt) const {
    uint16_t code = 0;
    SHA256 sha;
    sha.resetHMAC(_scopeKey, sizeof(_scopeKey));
    const uint8_t type = pkt->getPayloadType();
    sha.update(&type, 1);
    sha.update(pkt->payload, pkt->payload_len);
    sha.finalizeHMAC(_scopeKey, sizeof(_scopeKey), &code, sizeof(code));
    if (code == 0) ++code;
    else if (code == 0xFFFF) --code;
    return code;
}

// A flood, scoped like the companion firmware's sendFloodScoped: transport
// codes {region code, 0} when a region is set, a plain flood otherwise.
void Host::flood(mesh::Packet* pkt, uint32_t delayMs) {
    const uint8_t hashSize = static_cast<uint8_t>(_hashSize);
    if (!_scoped) {
        sendFlood(pkt, delayMs, hashSize);
        return;
    }
    uint16_t codes[2] = {transportCode(pkt), 0};
    sendFlood(pkt, codes, delayMs, hashSize);
}

bool Host::isOurChannel(const mesh::GroupChannel& channel) const {
    return _channel && memcmp(channel.hash, _channel->channel.hash, sizeof(channel.hash)) == 0 &&
           memcmp(channel.secret, _channel->channel.secret, sizeof(channel.secret)) == 0;
}

void Host::onChannelDataRecv(const mesh::GroupChannel& channel, mesh::Packet* pkt, uint16_t type,
                             const uint8_t* data, size_t length) {
    if (!isOurChannel(channel) || type != static_cast<uint16_t>(DataType::RnsTunnel)) return;
    if (length == 0 || length > kMaxGroupData) return;   // the decoder bounds it; do not rely on that
    ++_dataReceived;
    const uint8_t hops = pkt && pkt->isRouteFlood() ? pkt->getPathHashCount() : 0;
    if (_sink) _sink(_sinkContext, DataType::RnsTunnel, data, length, hops);
}

void Host::sendScoped(mesh::Packet* pkt, uint32_t delayMs) {
    if (_reach == ChannelReach::Flood) flood(pkt, delayMs);
    else sendZeroHop(pkt, delayMs);
}

void Host::sendFloodScoped(const mesh::GroupChannel&, mesh::Packet* pkt, uint32_t delayMs) {
    sendScoped(pkt, delayMs);
}

void Host::sendFloodScoped(const ContactInfo&, mesh::Packet* pkt, uint32_t delayMs) {
    sendScoped(pkt, delayMs);
}

uint32_t Host::calcFloodTimeoutMillisFor(uint32_t airtimeMs) const {
    return kSendTimeoutBaseMs + static_cast<uint32_t>(kFloodTimeoutFactor * airtimeMs);
}

uint32_t Host::calcDirectTimeoutMillisFor(uint32_t airtimeMs, uint8_t pathLength) const {
    const uint8_t hops = pathLength & 63;   // low 6 bits: hash count (spec section 3)
    return kSendTimeoutBaseMs +
           static_cast<uint32_t>((airtimeMs * kDirectPerHopFactor + kDirectPerHopExtraMs) * (hops + 1));
}

} // namespace handheld::meshcore
