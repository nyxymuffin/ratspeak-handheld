#include "MeshCoreService.h"

#include <WiFi.h>
#include <esp_heap_caps.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>
#include <new>
#include <time.h>

#include "MeshCoreHost.h"
#include "MeshCoreRadio.h"
#include "protocol/RustEntropy.h"
#include "storage/FlashStore.h"

namespace handheld::meshcore {

namespace {

constexpr const char* kIdentityDir = "/meshcore";
// Private key only (PRV_KEY_SIZE); the public key is derived on load, so a
// stored pair can never disagree. Removed by factory reset with the rest of flash.
constexpr const char* kIdentityPath = "/meshcore/node.key";
constexpr int kPacketPool = 16;
// MeshCore rejects keys whose public key starts 0x00/0xFF; each attempt fails
// with probability 1/128, so this bound is never reached in practice.
constexpr int kIdentityAttempts = 16;

// Hardware RNG, as used for the Reticulum identity (RustEntropy.h).
class HardwareRng final : public mesh::RNG {
public:
    void random(uint8_t* dest, size_t size) override { RustEntropy::fill(dest, size); }
};

// Ratspeak owns the system clock (NTP/GPS/restored epoch); MeshCore only reads it.
class SystemClock final : public mesh::RTCClock {
public:
    uint32_t getCurrentTime() override { return static_cast<uint32_t>(time(nullptr)); }
    void setCurrentTime(uint32_t) override {}
};

// esp_random() is a true RNG only while RF is enabled (ESP-IDF v4.4, "Random
// Number Generation"); otherwise its output is pseudo-random. The ADC noise
// source (bootloader_random_enable) is not used because the T-Pager battery
// monitor reads the same SAR ADC from another task. Instead the WiFi radio is
// started, unconnected, for the moment a key is generated. Callers are on the
// task that owns WiFi, so nothing else changes the mode meanwhile.
class RfEntropyWindow {
public:
    RfEntropyWindow() : _wasOff(WiFi.getMode() == WIFI_OFF) {
        if (_wasOff) _active = WiFi.mode(WIFI_STA);
    }
    ~RfEntropyWindow() { if (_wasOff) WiFi.mode(WIFI_OFF); }
    bool trueRandom() const { return !_wasOff || _active; }

private:
    bool _wasOff;
    bool _active = false;
};

void toHex(char* out, const uint8_t* bytes, size_t count) {
    static constexpr char kDigits[] = "0123456789abcdef";
    for (size_t i = 0; i < count; ++i) {
        out[i * 2] = kDigits[bytes[i] >> 4];
        out[i * 2 + 1] = kDigits[bytes[i] & 0x0F];
    }
    out[count * 2] = '\0';
}

} // namespace

struct Service::Node {
    explicit Node(BoardRadio& radio)
        : adapter(radio), packets(kPacketPool), host(adapter, millisClock, rng, clock, packets, tables) {}

    RadioAdapter adapter;
    ArduinoMillis millisClock;
    HardwareRng rng;
    SystemClock clock;
    StaticPoolPacketManager packets;
    SimpleMeshTables tables;
    Host host;
    Config config;
};

bool Service::begin(const Config& config) {
    if (_running) return true;
    if (!_node && !createNode(config)) return false;
    // Each step is retried by a later begin() if it fails; the node is kept.
    if (!_identityReady && !(_identityReady = loadOrCreateIdentity(*_node))) return false;
    if (!_node->adapter.configure(_node->config.radio)) {
        Serial.println("[MESHCORE] Radio unavailable");
        return false;
    }
    if (!_hostStarted) startHost();
    _running = true;
    return true;
}

bool Service::createNode(const Config& config) {
    void* memory = heap_caps_aligned_alloc(alignof(Node), sizeof(Node), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!memory) {
        Serial.println("[MESHCORE] No PSRAM for the node");
        return false;
    }
    _node = new (memory) Node(_radio);
    _node->config = config;
    return true;
}

void Service::startHost() {
    const Config& config = _node->config;
    Host& host = _node->host;
    host.begin();
    host.setRouting(config.pathHashSize, config.channelReach);
    host.setDataSink(&Service::deliver, this);
    if (config.channelPsk[0] && !host.joinChannel(config.channelName, config.channelPsk))
        Serial.printf("[MESHCORE] Channel '%s' rejected (key must be base64 of 16 or 32 bytes)\n",
                      config.channelName);
    host.sendAdvert(config.nodeName, true);
    _hostStarted = true;
    Serial.printf("[MESHCORE] Up: node=%s key=%s channel=%s\n", config.nodeName,
                  status().publicKeyPrefix, host.channelJoined() ? config.channelName : "(none)");
}

void Service::stop() {
    if (!_running) return;
    _node->adapter.quiesce();
    _running = false;
}

void Service::loop() {
    if (_running) _node->host.loop();
}

bool Service::sendData(DataType type, const uint8_t* data, size_t length) {
    return _running && _node->host.sendData(type, data, length);
}

bool Service::sendAdvert(bool flood) {
    return _running && _node->host.sendAdvert(_node->config.nodeName, flood);
}

Status Service::status() const {
    Status out;
    if (!_node) return out;
    const Host& host = _node->host;
    out.online = _running;
    out.channelJoined = host.channelJoined();
    toHex(out.publicKeyPrefix, host.self_id.pub_key, 4);
    out.rxPackets = host.getNumRecvFlood() + host.getNumRecvDirect();
    out.txPackets = host.getNumSentFlood() + host.getNumSentDirect();
    out.txFailures = _node->adapter.txFailures();
    out.dataReceived = host.dataReceived();
    out.dataSent = host.dataSent();
    out.lastRssi = _node->adapter.getLastRSSI();
    out.lastSnr = _node->adapter.getLastSNR();
    return out;
}

bool Service::publicKey(uint8_t out[kPublicKeySize]) const {
    static_assert(kPublicKeySize == PUB_KEY_SIZE, "MeshCoreTypes.h must track MeshCore.h");
    if (!_node || !_identityReady) return false;
    memcpy(out, _node->host.self_id.pub_key, kPublicKeySize);
    return true;
}

bool Service::loadOrCreateIdentity(Node& node) {
    // A key file that exists but cannot be read is a fault, not a first boot:
    // replacing it would silently give this node a new identity on the mesh.
    return _flash.exists(kIdentityPath) ? loadIdentity(node) : createIdentity(node);
}

bool Service::loadIdentity(Node& node) {
    uint8_t key[PRV_KEY_SIZE] = {};
    size_t length = 0;
    bool ok = _flash.readFileFully(kIdentityPath, key, sizeof(key), length) && length == sizeof(key) &&
              mesh::LocalIdentity::validatePrivateKey(key);
    if (ok) {
        mesh::LocalIdentity identity;
        identity.readFrom(key, sizeof(key));   // derives the public key
        node.host.setIdentity(identity);
    } else {
        Serial.println("[MESHCORE] Identity file unreadable or invalid; not replacing it");
    }
    memset(key, 0, sizeof(key));
    return ok;
}

bool Service::createIdentity(Node& node) {
    RfEntropyWindow entropy;
    if (!entropy.trueRandom()) {
        Serial.println("[MESHCORE] No RF entropy source; identity not created");
        return false;
    }
    uint8_t key[PRV_KEY_SIZE] = {};
    bool ok = false;
    for (int attempt = 0; attempt < kIdentityAttempts && !ok; ++attempt) {
        mesh::LocalIdentity identity(&node.rng);
        ok = identity.writeTo(key, sizeof(key)) == sizeof(key) && mesh::LocalIdentity::validatePrivateKey(key);
        if (ok) node.host.setIdentity(identity);
    }
    ok = ok && _flash.ensureDir(kIdentityDir) && _flash.writeAtomic(kIdentityPath, key, sizeof(key));
    Serial.println(ok ? "[MESHCORE] New node identity created" : "[MESHCORE] Identity creation failed");
    memset(key, 0, sizeof(key));
    return ok;
}

void Service::deliver(void* context, DataType type, const uint8_t* data, size_t length) {
    auto* self = static_cast<Service*>(context);
    if (self->_sink) self->_sink(type, data, length);
}

} // namespace handheld::meshcore
