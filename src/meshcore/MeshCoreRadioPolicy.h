#pragma once

// Pure radio policy for the MeshCore personality: no Arduino, radio or
// MeshCore dependencies, so it is unit-tested on the host (test/host/meshcore).
//
// MeshCore's protocol specification leaves the radio layer out of scope
// (section 00 Overview: "Radio-layer parameters" are not covered), so these
// values follow the reference
// implementation at the vendored commit (vendor/meshcore/PROVENANCE.txt):
//   - preamble: RadioLibWrappers.h preambleLengthForSF()
//   - packet score: RadioLibWrappers.cpp packetScoreInt()
//   - channel activity: CustomSX1262.h isReceiving()

#include <stdint.h>

namespace handheld::meshcore {

// SX126x GetIrqStatus bits (Semtech SX1261/2 datasheet, table 13-29).
enum class IrqBit : uint16_t {
    PreambleDetected = 1u << 2,
    HeaderValid      = 1u << 4,
    HeaderError      = 1u << 5,
};

inline constexpr bool hasIrq(uint16_t flags, IrqBit bit) {
    return (flags & static_cast<uint16_t>(bit)) != 0;
}

// Longer preamble at low SF, as in MeshCore, so sleeping receivers catch it.
inline constexpr uint16_t preambleSymbols(uint8_t sf) {
    return sf <= 8 ? 32 : 16;
}

// Chance (0..1) that a rebroadcast of this packet would be heard. MeshCore
// uses it to delay flood retransmissions: weaker packets wait longer.
inline float packetScore(float snrDb, uint8_t sf, int packetLength) {
    static constexpr float kSnrFloorDb[] = {-7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f};  // SF7..SF12
    if (sf < 7 || sf > 12) return 0.0f;
    const float floor = kSnrFloorDb[sf - 7];
    if (snrDb < floor) return 0.0f;
    const float snrMargin = (snrDb - floor) / 10.0f;
    const float collisionPenalty = 1.0f - (packetLength / 256.0f);
    const float score = snrMargin * collisionPenalty;
    return score < 0.0f ? 0.0f : (score > 1.0f ? 1.0f : score);
}

// Time (ms) of the preamble, sync word, SFD and header for the given
// modulation: how long a genuine packet may show "preamble detected" before
// "header valid" must follow. Mirrors RadioLibWrapper::calcMaxPacketMillis.
inline uint32_t preambleMillis(uint8_t sf, uint32_t bandwidthHz, uint16_t preamble) {
    if (bandwidthHz == 0 || sf < 5 || sf > 12) return 0;
    const uint32_t symbolUs = static_cast<uint32_t>((1000000ull << sf) / bandwidthHz);
    const uint32_t sfCoeffX4 = (sf == 5 || sf == 6) ? 25 : 17;  // sync word + SFD, x4
    const uint32_t preambleUs = (((preamble + 8u) * 4u + sfCoeffX4) * symbolUs) / 4u;
    return (preambleUs + 999u) / 1000u;
}

// Listen-before-talk state machine over the latched SX126x IRQ flags.
// The radio driver leaves preamble/header flags latched until RX is re-armed,
// so a false preamble detection would otherwise block TX forever.
class ChannelActivity {
public:
    enum class Verdict : uint8_t { Idle, Busy, Rearm };

    void setLimits(uint32_t preambleMs, uint32_t maxPayloadMs) {
        _preambleMs = preambleMs;
        _maxPayloadMs = maxPayloadMs;
    }
    void reset() { _active = false; _headerSeen = false; }

    Verdict update(uint16_t irqFlags, uint32_t nowMs) {
        if (hasIrq(irqFlags, IrqBit::HeaderError)) return rearm();
        const bool header = hasIrq(irqFlags, IrqBit::HeaderValid);
        if (header) return trackHeader(nowMs);
        if (_headerSeen) { reset(); return Verdict::Idle; }  // flags cleared elsewhere
        if (hasIrq(irqFlags, IrqBit::PreambleDetected)) return trackPreamble(nowMs);
        reset();
        return Verdict::Idle;
    }

private:
    Verdict rearm() { reset(); return Verdict::Rearm; }

    Verdict trackHeader(uint32_t nowMs) {
        if (!_headerSeen) { _headerSeen = true; start(nowMs); }
        return nowMs - _activityAt > _maxPayloadMs ? rearm() : Verdict::Busy;
    }

    Verdict trackPreamble(uint32_t nowMs) {
        if (!_active) start(nowMs);
        return nowMs - _activityAt > _preambleMs ? rearm() : Verdict::Busy;
    }

    void start(uint32_t nowMs) { _active = true; _activityAt = nowMs; }

    uint32_t _preambleMs = 0;
    uint32_t _maxPayloadMs = 0;
    uint32_t _activityAt = 0;   // valid only while _active
    bool _active = false;
    bool _headerSeen = false;
};

} // namespace handheld::meshcore
