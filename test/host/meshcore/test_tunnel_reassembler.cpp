// Host unit tests for src/meshcore/TunnelReassembler.{h,cpp}.
// Build and run: make -C test/host/meshcore

#include <cstdio>
#include <cstring>

#include "TunnelReassembler.h"

using namespace handheld::meshcore::tunnel;
using Result = Reassembler::Result;

static int failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            ++failures;                                                          \
        }                                                                        \
    } while (0)

static const uint8_t kA[4] = {0xaa, 0x00, 0x00, 0x01};
static const uint8_t kB[4] = {0xbb, 0x00, 0x00, 0x02};

struct Packet {
    uint8_t bytes[500];
    size_t length;
};

static Packet makePacket(size_t length, uint8_t seed) {
    Packet p{};
    p.length = length;
    for (size_t i = 0; i < length; ++i) p.bytes[i] = uint8_t(seed + i * 13);
    return p;
}

// Fragment `index` of `p` as the codec would encode and decode it.
static Fragment fragmentOf(const Packet& p, const uint8_t* sender, uint32_t id, uint8_t index, uint8_t (&body)[kMaxBody]) {
    const size_t n = encodeFragment(sender, id, index, p.bytes, p.length, body, sizeof(body));
    Fragment f;
    decodeFragment(body, n, f);
    return f;
}

static Result feed(Reassembler& r, const Packet& p, const uint8_t* sender, uint32_t id, uint8_t index,
                   uint32_t now, size_t& outLength, uint8_t (&out)[Reassembler::kMaxPacket]) {
    uint8_t body[kMaxBody];
    return r.accept(fragmentOf(p, sender, id, index, body), now, out, outLength);
}

static void completesInAnyOrder() {
    Reassembler r;
    const Packet p = makePacket(500, 1);
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    CHECK(feed(r, p, kA, 7, 2, 100, length, out) == Result::Pending);
    CHECK(feed(r, p, kA, 7, 0, 110, length, out) == Result::Pending);
    CHECK(feed(r, p, kA, 7, 3, 120, length, out) == Result::Pending);
    CHECK(feed(r, p, kA, 7, 1, 130, length, out) == Result::Complete);
    CHECK(length == 500 && memcmp(out, p.bytes, 500) == 0);
    CHECK(r.pending() == 0);
}

static void singleFragmentPacket() {
    Reassembler r;
    const Packet p = makePacket(51, 9);                      // a path request (manual 6.7.3)
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    CHECK(feed(r, p, kA, 1, 0, 0, length, out) == Result::Complete);
    CHECK(length == 51 && memcmp(out, p.bytes, 51) == 0);
}

static void duplicatesAreSuppressed() {
    Reassembler r;
    const Packet p = makePacket(200, 3);
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    CHECK(feed(r, p, kA, 5, 0, 0, length, out) == Result::Pending);
    CHECK(feed(r, p, kA, 5, 0, 1, length, out) == Result::Duplicate);    // same fragment again
    CHECK(feed(r, p, kA, 5, 1, 2, length, out) == Result::Complete);
    CHECK(feed(r, p, kA, 5, 1, 3, length, out) == Result::Duplicate);    // flood echo after completion
    CHECK(feed(r, p, kA, 5, 0, 30002, length, out) == Result::Pending);  // dedup window (30 s) over
}

static void sendersAreSeparate() {
    Reassembler r;
    const Packet p = makePacket(300, 4), q = makePacket(300, 5);
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    CHECK(feed(r, p, kA, 9, 0, 0, length, out) == Result::Pending);
    CHECK(feed(r, q, kB, 9, 1, 0, length, out) == Result::Pending);      // same pkt_id, other sender
    CHECK(feed(r, q, kB, 9, 0, 0, length, out) == Result::Complete);
    CHECK(memcmp(out, q.bytes, 300) == 0);
    CHECK(r.pending() == 1);
}

static void specViolationsRejected() {
    Reassembler r;
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    uint8_t payload[154] = {};
    Fragment f;
    memcpy(f.sender, kA, 4);
    f.payload = payload;
    f.total = 5; f.index = 0; f.payloadLength = 154;                     // 5 fragments > 500-byte MTU
    CHECK(r.accept(f, 0, out, length) == Result::Rejected);
    f.total = 2; f.index = 0; f.payloadLength = 100;                     // short middle fragment
    CHECK(r.accept(f, 0, out, length) == Result::Rejected);
    f.total = 4; f.index = 3; f.payloadLength = 50;                      // 3*154+50 = 512 > 500
    CHECK(r.accept(f, 0, out, length) == Result::Rejected);
    f.total = 4; f.index = 3; f.payloadLength = 38;                      // exactly 500
    CHECK(r.accept(f, 0, out, length) == Result::Pending);
}

static void twoLossyCopiesComplete() {
    Reassembler r;
    const Packet p = makePacket(460, 6);                                  // 3 fragments
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    // First copy (id 20) loses fragment 1; the re-send (id 21) loses fragment 2.
    CHECK(feed(r, p, kA, 20, 0, 0, length, out) == Result::Pending);
    CHECK(feed(r, p, kA, 20, 2, 1, length, out) == Result::Pending);
    CHECK(feed(r, p, kA, 21, 0, 100, length, out) == Result::Pending);
    CHECK(feed(r, p, kA, 21, 1, 101, length, out) == Result::Complete);
    CHECK(length == 460 && memcmp(out, p.bytes, 460) == 0);
    CHECK(r.mergedCopies() == 1);
    CHECK(r.pending() == 0);                                              // the older copy is consumed
}

static void differentPacketsNeverMerge() {
    Reassembler r;
    const Packet p = makePacket(460, 6), q = makePacket(460, 7);
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    CHECK(feed(r, p, kA, 30, 0, 0, length, out) == Result::Pending);
    CHECK(feed(r, p, kA, 30, 2, 0, length, out) == Result::Pending);
    CHECK(feed(r, q, kA, 31, 0, 0, length, out) == Result::Pending);      // fragment 0 differs
    CHECK(feed(r, q, kA, 31, 1, 0, length, out) == Result::Pending);
    CHECK(r.mergedCopies() == 0);
}

static void fullTableEvictsOldest() {
    Reassembler r;
    const Packet p = makePacket(300, 8);
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    for (uint32_t id = 0; id < Reassembler::kSlots; ++id)
        CHECK(feed(r, p, kA, 100 + id, 0, id * 10, length, out) == Result::Pending);
    CHECK(feed(r, p, kB, 999, 0, 1000, length, out) == Result::Pending);
    CHECK(r.evictions() == 1 && r.pending() == Reassembler::kSlots);
    // id 101 survived and completes, freeing a slot.
    CHECK(feed(r, p, kA, 101, 1, 1001, length, out) == Result::Complete);
    // id 100 (the oldest) was evicted: its second fragment starts afresh.
    CHECK(feed(r, p, kA, 100, 1, 1002, length, out) == Result::Pending);
    CHECK(r.evictions() == 1);
}

static void staleAssembliesExpire() {
    Reassembler r;
    const Packet p = makePacket(300, 9);
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    CHECK(feed(r, p, kA, 40, 0, 0, length, out) == Result::Pending);
    r.expire(300000);
    CHECK(r.pending() == 1);                                              // at the limit
    r.expire(300001);
    CHECK(r.pending() == 0 && r.timeouts() == 1);
}

static void millisWrap() {
    Reassembler r;
    const Packet p = makePacket(300, 10);
    uint8_t out[Reassembler::kMaxPacket];
    size_t length = 0;
    CHECK(feed(r, p, kA, 50, 0, 0xFFFFFF00u, length, out) == Result::Pending);
    r.expire(0x00000100u);                                                // 512 ms later
    CHECK(r.pending() == 1);
    CHECK(feed(r, p, kA, 50, 1, 0x00000200u, length, out) == Result::Complete);
    CHECK(feed(r, p, kA, 50, 1, 0x00000300u, length, out) == Result::Duplicate);
}

int main() {
    completesInAnyOrder();
    singleFragmentPacket();
    duplicatesAreSuppressed();
    sendersAreSeparate();
    specViolationsRejected();
    twoLossyCopiesComplete();
    differentPacketsNeverMerge();
    fullTableEvictsOldest();
    staleAssembliesExpire();
    millisWrap();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("meshcore tunnel reassembler: PASS\n");
    return 0;
}
