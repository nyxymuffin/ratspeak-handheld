#pragma once

// MeshCore mesh::Radio over Ratspeak's own SX1262 driver (BoardRadio). Used
// only by the MeshCore host; include from src/meshcore/*.cpp only.

#include <Dispatcher.h>

#include "radio/BoardRadio.h"
#include "MeshCoreRadioPolicy.h"
#include "MeshCoreTypes.h"

namespace handheld::meshcore {

class RadioAdapter final : public mesh::Radio {
public:
    explicit RadioAdapter(BoardRadio& radio) : _radio(radio) {}

    // Applies MeshCore modulation to the shared radio and arms RX. The driver
    // already uses MeshCore's private sync word (0x1424 in the register),
    // explicit header and CRC on; this sets the parameters that differ.
    bool configure(const RadioParams& params);

    void begin() override;
    int recvRaw(uint8_t* bytes, int size) override;
    uint32_t getEstAirtimeFor(int length) override;
    float packetScore(float snrDb, int length) override;
    bool startSendRaw(const uint8_t* bytes, int length) override;
    bool isSendComplete() override;
    void onSendFinished() override;
    bool isInRecvMode() const override { return _rxArmed && !_txActive; }
    bool isReceiving() override;
    float getLastRSSI() const override { return _lastRssi; }
    float getLastSNR() const override { return _lastSnr; }

    // Stops using the radio: aborts any transmission and leaves it in standby.
    void quiesce();
    uint32_t txFailures() const { return _txFailures; }

private:
    void armReceive();

    BoardRadio& _radio;
    ChannelActivity _activity;
    uint8_t _sf = 0;
    bool _rxArmed = false;
    bool _txActive = false;
    float _lastRssi = 0;
    float _lastSnr = 0;
    uint32_t _txFailures = 0;
};

} // namespace handheld::meshcore
