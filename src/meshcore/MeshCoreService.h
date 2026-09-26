#pragma once

// Facade for the MeshCore LoRa personality: the only MeshCore header the rest
// of Ratspeak includes. No MeshCore headers here (see MeshCoreTypes.h).
//
// Threading: every call, including loop(), must come from the one task that
// owns the radio and WiFi (the device-service owner task). MeshCore is not
// thread-safe, the radio driver is shared with the native Reticulum LoRa
// interface (which must not run at the same time), and first-time identity
// creation briefly switches the WiFi radio on for entropy.
//
// Lifetime: the node is allocated once per boot and never freed (MeshCore's
// packet pool has no destructors). stop() releases the radio; begin() after
// stop() resumes with the configuration given to the first begin(). Changing
// the configuration therefore needs a reboot, as with Ratspeak's LoRa enable.

#include <functional>

#include "MeshCoreTypes.h"
#include "radio/BoardRadio.h"

class FlashStore;

namespace handheld::meshcore {

class Service {
public:
    // Called from inside Service::loop() for each GRP_DATA packet on our
    // channel. `data` is valid only for the duration of the call. The sink may
    // call sendData(); it must not call stop() or setDataSink().
    using DataSink = std::function<void(DataType type, const uint8_t* data, size_t length)>;

    Service(BoardRadio& radio, FlashStore& flash) : _radio(radio), _flash(flash) {}
    Service(const Service&) = delete;
    Service& operator=(const Service&) = delete;

    // First call: loads or creates the MeshCore identity, applies the radio
    // parameters, joins the data channel (if a PSK is set) and sends a flood
    // advert. Later calls (after stop()) only re-apply the radio parameters.
    bool begin(const Config& config);
    void stop();
    void loop();
    bool running() const { return _running; }

    bool sendData(DataType type, const uint8_t* data, size_t length);
    bool sendAdvert(bool flood);
    void setDataSink(DataSink sink) { _sink = std::move(sink); }
    Status status() const;

private:
    struct Node;
    bool createNode(const Config& config);
    void startHost();
    bool loadOrCreateIdentity(Node& node);
    bool loadIdentity(Node& node);
    bool createIdentity(Node& node);
    static void deliver(void* context, DataType type, const uint8_t* data, size_t length);

    BoardRadio& _radio;
    FlashStore& _flash;
    Node* _node = nullptr;   // PSRAM, allocated on first begin(), kept for the boot
    bool _identityReady = false;
    bool _hostStarted = false;
    bool _running = false;
    DataSink _sink;
};

} // namespace handheld::meshcore
