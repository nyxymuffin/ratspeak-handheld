#include "MeshCoreRadio.h"

#include <math.h>

namespace handheld::meshcore {

// Largest raw MeshCore frame (MAX_TRANS_UNIT); bounds the header-valid wait.
static constexpr uint16_t kMaxFrame = 255;

bool RadioAdapter::configure(const RadioParams& params) {
    if (!_radio.isRadioOnline()) return false;
    _sf = params.spreadingFactor;
    const uint16_t preamble = preambleSymbols(_sf);
    _radio.setFrequency(params.frequencyHz);
    _radio.setSpreadingFactor(_sf);
    _radio.setSignalBandwidth(params.bandwidthHz);
    _radio.setCodingRate4(params.codingRate);
    _radio.setTxPower(params.txPowerDbm);
    _radio.setPreambleLength(preamble);
    _radio.setInvertIQ(false);
    _radio.enableCrc();

    const uint32_t preambleMs = preambleMillis(_sf, params.bandwidthHz, preamble);
    const uint32_t frameMs = static_cast<uint32_t>(ceilf(_radio.getAirtime(kMaxFrame)));
    _activity.setLimits(preambleMs, frameMs > preambleMs ? frameMs - preambleMs : frameMs);
    armReceive();
    return _radio.isRadioOnline();
}

void RadioAdapter::begin() {
    armReceive();
}

void RadioAdapter::armReceive() {
    // receive() also clears every latched IRQ flag, which resets listen-before-talk.
    _radio.receive();
    _activity.reset();
    _rxArmed = true;
}

int RadioAdapter::recvRaw(uint8_t* bytes, int size) {
    if (_txActive || !_radio.packetAvailable) return 0;
    _radio.packetAvailable = false;
    const int length = _radio.parsePacket();   // 0 on CRC failure; the driver re-arms RX
    if (length <= 0 || length > size) {
        if (length > size) armReceive();
        return 0;
    }
    memcpy(bytes, _radio.packetBuffer(), length);
    _lastRssi = static_cast<float>(_radio.packetRssi());
    _lastSnr = _radio.packetSnr();
    armReceive();
    return length;
}

uint32_t RadioAdapter::getEstAirtimeFor(int length) {
    return static_cast<uint32_t>(ceilf(_radio.getAirtime(static_cast<uint16_t>(length))));
}

float RadioAdapter::packetScore(float snrDb, int length) {
    return handheld::meshcore::packetScore(snrDb, _sf, length);
}

bool RadioAdapter::startSendRaw(const uint8_t* bytes, int length) {
    // beginPacket() puts the radio in standby, so every refusal must re-arm RX.
    const bool started = _radio.beginPacket() &&
        _radio.write(bytes, length) == static_cast<size_t>(length) &&
        _radio.endPacket(true);
    if (!started) {
        ++_txFailures;
        armReceive();
        return false;
    }
    _txActive = true;
    _rxArmed = false;
    return true;
}

bool RadioAdapter::isSendComplete() {
    // isTxBusy() also ends a transmission that overran its airtime budget.
    return !_radio.isTxBusy();
}

void RadioAdapter::onSendFinished() {
    // MeshCore gives up at 1.5x airtime, before the driver's own deadline
    // (1.5x + 2 s). Without the abort the driver would keep refusing TX.
    if (_radio.isTxBusy()) _radio.abortTx();
    if (_radio.txFailed()) ++_txFailures;
    _txActive = false;
    armReceive();
}

void RadioAdapter::quiesce() {
    if (_txActive && _radio.isTxBusy()) _radio.abortTx();
    _txActive = false;
    _rxArmed = false;
    _radio.standby();
}

bool RadioAdapter::isReceiving() {
    if (_txActive) return false;
    switch (_activity.update(_radio.getIrqFlags(), millis())) {
    case ChannelActivity::Verdict::Busy: return true;
    case ChannelActivity::Verdict::Rearm: armReceive(); return false;
    case ChannelActivity::Verdict::Idle: return false;
    }
    return false;
}

} // namespace handheld::meshcore
