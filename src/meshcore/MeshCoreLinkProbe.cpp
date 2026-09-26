// Phase 2a link probe: proves the vendored MeshCore subset (vendor/meshcore)
// compiles and links into the T-Pager image with no macro or symbol collisions.
// Nothing calls it. `used` stops the compiler dropping it and the
// `-Wl,-u,ratspeak_meshcore_link_probe` flag in platformio.ini stops the
// linker's --gc-sections dropping it, so every BaseChatMesh vtable entry and
// helper it references must resolve. The mesh object is heap-allocated rather
// than static so the ~20 KB contact table does not occupy internal RAM from
// boot. Replaced by the real MeshCore host in phase 2b.

#include <helpers/BaseChatMesh.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SimpleMeshTables.h>
#include <helpers/StaticPoolPacketManager.h>

namespace {

class ProbeRadio final : public mesh::Radio {
public:
    int recvRaw(uint8_t*, int) override { return 0; }
    uint32_t getEstAirtimeFor(int) override { return 0; }
    float packetScore(float, int) override { return 0.0f; }
    bool startSendRaw(const uint8_t*, int) override { return false; }
    bool isSendComplete() override { return true; }
    void onSendFinished() override {}
    bool isInRecvMode() const override { return true; }
};

class ProbeRng final : public mesh::RNG {
public:
    void random(uint8_t* dest, size_t size) override { memset(dest, 0, size); }
};

class ProbeMesh final : public BaseChatMesh {
public:
    ProbeMesh(mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng,
              mesh::RTCClock& rtc, mesh::PacketManager& mgr, mesh::MeshTables& tables)
        : BaseChatMesh(radio, ms, rng, rtc, mgr, tables) {}

protected:
    void onDiscoveredContact(ContactInfo&, bool, uint8_t, const uint8_t*) override {}
    ContactInfo* processAck(const uint8_t*) override { return nullptr; }
    void onContactPathUpdated(const ContactInfo&) override {}
    void onMessageRecv(const ContactInfo&, mesh::Packet*, uint32_t, const char*) override {}
    void onCommandDataRecv(const ContactInfo&, mesh::Packet*, uint32_t, const char*) override {}
    void onSignedMessageRecv(const ContactInfo&, mesh::Packet*, uint32_t, const uint8_t*, const char*) override {}
    uint32_t calcFloodTimeoutMillisFor(uint32_t airtime) const override { return airtime; }
    uint32_t calcDirectTimeoutMillisFor(uint32_t airtime, uint8_t) const override { return airtime; }
    void onSendTimeout() override {}
    void onChannelMessageRecv(const mesh::GroupChannel&, mesh::Packet*, uint32_t, const char*) override {}
    uint8_t onContactRequest(const ContactInfo&, uint32_t, const uint8_t*, uint8_t, uint8_t*) override { return 0; }
    void onContactResponse(const ContactInfo&, const uint8_t*, uint8_t) override {}
};

} // namespace

extern "C" __attribute__((used)) void ratspeak_meshcore_link_probe() {
    static ProbeRadio radio;
    static ArduinoMillis clock;
    static ProbeRng rng;
    static VolatileRTCClock rtc;
    static StaticPoolPacketManager packets(4);
    auto* tables = new SimpleMeshTables();
    auto* mesh = new ProbeMesh(radio, clock, rng, rtc, packets, *tables);
    mesh->begin();
    mesh->addChannel("probe", "");  // empty key is rejected; only the symbol matters
    mesh->loop();
    delete mesh;
    delete tables;
}
