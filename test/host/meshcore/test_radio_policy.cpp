// Host unit tests for src/meshcore/MeshCoreRadioPolicy.h and MeshCoreTypes.h.
// Build and run: make -C test/host/meshcore
// Expected values come from MeshCore's reference implementation at the
// vendored commit (RadioLibWrappers.{h,cpp}, CustomSX1262.h).

#include <cmath>
#include <cstdio>

#include "MeshCoreRadioPolicy.h"
#include "MeshCoreTypes.h"

using namespace handheld::meshcore;

static int failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            ++failures;                                                          \
        }                                                                        \
    } while (0)

static bool near(float a, float b) { return std::fabs(a - b) < 1e-4f; }

static void preambleFollowsSpreadingFactor() {
    CHECK(preambleSymbols(7) == 32);
    CHECK(preambleSymbols(8) == 32);
    CHECK(preambleSymbols(9) == 16);
    CHECK(preambleSymbols(12) == 16);
}

static void packetScoreMatchesReference() {
    CHECK(packetScore(-8.0f, 7, 10) == 0.0f);            // below SF7 floor (-7.5 dB)
    CHECK(packetScore(10.0f, 6, 10) == 0.0f);            // SF out of range
    CHECK(packetScore(10.0f, 13, 10) == 0.0f);
    CHECK(near(packetScore(2.5f, 7, 0), 1.0f));          // 10 dB margin, no collision penalty
    CHECK(near(packetScore(-2.5f, 7, 128), 0.25f));      // 5 dB margin x half penalty
    CHECK(near(packetScore(30.0f, 7, 0), 1.0f));         // clamped high
    CHECK(near(packetScore(-20.0f, 12, 64), 0.0f));      // exactly at SF12 floor
    CHECK(packetScore(10.0f, 7, 300) == 0.0f);           // penalty below zero clamps low
}

static void preambleMillisMatchesReference() {
    // US preset: SF7, 62.5 kHz, 32 symbols. Tsym = 2048 us;
    // ((32 + 8) * 4 + 17) * 2048 / 4 = 90624 us -> 91 ms (rounded up).
    CHECK(preambleMillis(7, 62500, 32) == 91);
    // SF12, 125 kHz, 16 symbols: Tsym = 32768 us; (24*4+17)*32768/4 = 925696 us.
    CHECK(preambleMillis(12, 125000, 16) == 926);
    CHECK(preambleMillis(7, 0, 32) == 0);
    CHECK(preambleMillis(4, 125000, 32) == 0);
}

static constexpr uint16_t kPreamble = static_cast<uint16_t>(IrqBit::PreambleDetected);
static constexpr uint16_t kHeader = static_cast<uint16_t>(IrqBit::HeaderValid);
static constexpr uint16_t kHeaderError = static_cast<uint16_t>(IrqBit::HeaderError);
using Verdict = ChannelActivity::Verdict;

static void activityIdleWithoutFlags() {
    ChannelActivity activity;
    activity.setLimits(100, 1000);
    CHECK(activity.update(0, 5000) == Verdict::Idle);
}

static void falsePreambleTimesOut() {
    ChannelActivity activity;
    activity.setLimits(100, 1000);
    CHECK(activity.update(kPreamble, 5000) == Verdict::Busy);
    CHECK(activity.update(kPreamble, 5100) == Verdict::Busy);    // at the limit
    CHECK(activity.update(kPreamble, 5101) == Verdict::Rearm);   // past it
    CHECK(activity.update(0, 5102) == Verdict::Idle);            // after re-arm
}

static void headerHoldsUntilMaxPayload() {
    ChannelActivity activity;
    activity.setLimits(100, 1000);
    CHECK(activity.update(kPreamble, 5000) == Verdict::Busy);
    CHECK(activity.update(kPreamble | kHeader, 5050) == Verdict::Busy);
    CHECK(activity.update(kPreamble | kHeader, 6050) == Verdict::Busy);   // header timer starts at 5050
    CHECK(activity.update(kPreamble | kHeader, 6051) == Verdict::Rearm);
}

static void headerErrorRearmsImmediately() {
    ChannelActivity activity;
    activity.setLimits(100, 1000);
    CHECK(activity.update(kPreamble | kHeaderError, 5000) == Verdict::Rearm);
}

static void headerErrorWhileIdleRearms() {
    ChannelActivity activity;
    activity.setLimits(100, 1000);
    CHECK(activity.update(kHeaderError, 5000) == Verdict::Rearm);
    CHECK(activity.update(0, 5001) == Verdict::Idle);
}

static void headerWithoutPreambleUsesPayloadLimit() {
    ChannelActivity activity;
    activity.setLimits(100, 1000);
    CHECK(activity.update(kHeader, 5000) == Verdict::Busy);
    CHECK(activity.update(kHeader, 5500) == Verdict::Busy);    // past the preamble limit, still busy
    CHECK(activity.update(kHeader, 6000) == Verdict::Busy);
    CHECK(activity.update(kHeader, 6001) == Verdict::Rearm);
}

static void clearedHeaderResetsState() {
    ChannelActivity activity;
    activity.setLimits(100, 1000);
    CHECK(activity.update(kHeader, 5000) == Verdict::Busy);
    CHECK(activity.update(0, 5010) == Verdict::Idle);
    // A new preamble after the reset starts a fresh timer.
    CHECK(activity.update(kPreamble, 9000) == Verdict::Busy);
    CHECK(activity.update(kPreamble, 9100) == Verdict::Busy);
}

static void timestampZeroIsStillActivity() {
    ChannelActivity activity;
    activity.setLimits(100, 1000);
    CHECK(activity.update(kPreamble, 0) == Verdict::Busy);     // millis() == 0 at boot
    CHECK(activity.update(kPreamble, 100) == Verdict::Busy);   // at the limit
    CHECK(activity.update(kPreamble, 101) == Verdict::Rearm);
}

static void millisWrapIsHandled() {
    ChannelActivity activity;
    activity.setLimits(100, 1000);
    CHECK(activity.update(kPreamble, 0xFFFFFFF0u) == Verdict::Busy);
    CHECK(activity.update(kPreamble, 0x00000010u) == Verdict::Busy);   // 32 ms later
    CHECK(activity.update(kPreamble, 0x00000060u) == Verdict::Rearm);  // 112 ms later
}

static void typesMatchSpecification() {
    static_assert(static_cast<uint16_t>(DataType::RnsTunnel) == 0xFFFF, "DATA_TYPE_DEV");
    static_assert(static_cast<uint8_t>(PathHashSize::OneByte) == 1, "spec section 3");
    static_assert(static_cast<uint8_t>(PathHashSize::ThreeBytes) == 3, "spec section 3");
    static_assert(kMaxGroupData == 184 - 16 - 3, "MAX_GROUP_DATA_LENGTH");
    const Config config;
    CHECK(config.radio.frequencyHz == 910525000u);
    CHECK(config.radio.bandwidthHz == 62500u);
    CHECK(config.radio.spreadingFactor == 7);
    CHECK(config.radio.codingRate == 5);
    CHECK(config.channelPsk[0] == '\0');   // never a compiled-in key
    CHECK(config.channelReach == ChannelReach::Flood);   // agreed default
}

int main() {
    preambleFollowsSpreadingFactor();
    packetScoreMatchesReference();
    preambleMillisMatchesReference();
    activityIdleWithoutFlags();
    falsePreambleTimesOut();
    headerHoldsUntilMaxPayload();
    headerErrorRearmsImmediately();
    headerErrorWhileIdleRearms();
    headerWithoutPreambleUsesPayloadLimit();
    clearedHeaderResetsState();
    timestampZeroIsStillActivity();
    millisWrapIsHandled();
    typesMatchSpecification();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("meshcore radio policy: PASS\n");
    return 0;
}
