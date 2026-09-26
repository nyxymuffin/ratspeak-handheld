#include "TunnelInterface.h"

#include <string.h>

#include "RnsPacket.h"

namespace handheld::meshcore::tunnel {

using handheld::TxOffer;
using handheld::TxReceiptEvent;

namespace {
constexpr uint32_t kExpireEveryMs = 30000;
constexpr uint32_t kRetryMs = 250;   // MeshCore refused a fragment (pool full): try again shortly
} // namespace

void TunnelInterface::configure(const Settings& settings) {
    _settings = settings;
    _throttle.setLimits(settings.throttle);
    _budget.setBytesPerHour(settings.airtimeBytesPerHour);
}

void TunnelInterface::start() {
    if (_online) return;
    if (_generation == UINT32_MAX) return;   // same exhaustion rule as LoRaInterface
    ++_generation;
    _online = true;
    _maintenance = false;
    _nextTxMs = _link.nowMs();
}

void TunnelInterface::stop() {
    _online = false;
    _current.active = false;
    const bool changing = _changingOwner;
    _changingOwner = true;
    while (_queued) dropQueued(0, "interface stopped");
    _changingOwner = changing;
}

// ── Owner wiring (same retire-on-change rules as LoRaInterface) ─────────────

void TunnelInterface::setTxValidator(void* context, TxValidator validator) {
    const bool changing = _changingOwner;
    _changingOwner = true;
    while (_queued) dropQueued(0, "validator changed");
    _validatorContext = context;
    _validator = validator;
    _changingOwner = changing;
}

void TunnelInterface::setReceiptHook(void* context, handheld::TxReceiptHook hook) {
    const bool changing = _changingOwner;
    _changingOwner = true;
    for (size_t i = 0; i < _queued;) {
        if (_queue[i].lease.receipt().valid()) dropQueued(i, "receipt owner changed");
        else ++i;
    }
    _receiptContext = context;
    _receiptHook = hook;
    _changingOwner = changing;
}

void TunnelInterface::notify(const handheld::TxLease& lease, TxReceiptEvent event) {
    const auto receipt = lease.receipt();
    if (!receipt.valid() || !_receiptHook) return;
    const bool notifying = _notifying;
    _notifying = true;
    _receiptHook(_receiptContext, receipt, event);
    _notifying = notifying;
}

bool TunnelInterface::leaseLive(const handheld::TxLease& lease) const {
    return lease.generation && _validator && _validator(_validatorContext, lease);
}

// ── Timing ──────────────────────────────────────────────────────────────────

// Start-to-start spacing of fragments. Flood: every repeater re-sends after a
// random 0-2 airtimes and each hop adds the same again, so a fragment sent
// inside that window collides with the previous one's echo (rns-gateway).
uint32_t TunnelInterface::gapMs(size_t bodyLength) const {
    uint32_t gap = _settings.fragmentGapMs;
    if (_link.floodReach()) {
        const uint32_t echo = _link.bodyAirtimeMs(bodyLength) * _settings.floodGapAirtimes;
        if (echo > gap) gap = echo;
    }
    return gap;
}

// Effective payload rate: one full fragment per gap. Reticulum sizes its link
// and retry timeouts from this, so it must reflect the pacing, not the radio.
uint32_t TunnelInterface::bitrate() const {
    if (!isOnline()) return 0;
    const uint32_t gap = gapMs(kMaxBody);
    return gap ? uint32_t(kMaxFragmentPayload * 8 * 1000 / gap) : 0;
}

uint32_t TunnelInterface::txWaitBudgetMs(uint32_t packets) const {
    uint32_t fragments = _current.active ? uint32_t(_current.total - _current.next) : 0;
    for (size_t i = 0; i < _queued; ++i) fragments += fragmentCount(_queue[i].length);
    fragments += packets * Reassembler::kMaxFragments;
    return (fragments + 1) * gapMs(kMaxBody);
}

// ── Transmit ────────────────────────────────────────────────────────────────

TxOffer TunnelInterface::reject(const handheld::TxLease& lease) {
    notify(lease, TxReceiptEvent::Dropped);
    return TxOffer::Rejected;
}

// The budget goes first: its only side effect is counting a shed, whereas the
// throttle records the packet as sent the moment it allows it.
bool TunnelInterface::admittedByPolicy(const uint8_t* data, size_t len) {
    const uint32_t now = _link.nowMs();
    const uint8_t total = fragmentCount(len);
    const size_t lastBody = kFragmentHeader + (len - size_t(total - 1) * kMaxFragmentPayload);
    const uint32_t estimate = uint32_t(total - 1) * grpDataOnAirBytes(kMaxBody) + grpDataOnAirBytes(lastBody);
    const bool discovery = rns::isAnnounce(data, len) || rns::isPathRequest(data, len);
    if (!_budget.admit(discovery, estimate, now) || !_throttle.allowOutbound(data, len, now)) {
        ++_counters.refusedByPolicy;
        return false;
    }
    return true;
}

TxOffer TunnelInterface::offerLeased(const uint8_t* data, size_t len, const handheld::TxLease& lease) {
    bool policyDrop = false;
    return offer(data, len, lease, policyDrop);
}

TxOffer TunnelInterface::offer(const uint8_t* data, size_t len, const handheld::TxLease& lease, bool& policyDrop) {
    policyDrop = false;
    if (_changingOwner || _notifying) return TxOffer::Blocked;
    if (!isOnline() || _maintenance || !data || len == 0 || len > Reassembler::kMaxPacket) return reject(lease);
    if (!leaseLive(lease)) return reject(lease);
    // Anything that returns Blocked (the caller keeps the packet and retries)
    // must come before the throttles, or the retry would be throttled.
    const bool startNow = !_current.active && _queued == 0 && pacingReady();
    if (!startNow && _queued >= kQueueDepth) return TxOffer::Blocked;
    if (!admittedByPolicy(data, len)) {
        policyDrop = true;
        return reject(lease);
    }

    if (startNow && begin(data, len, lease)) {
        notify(lease, TxReceiptEvent::Started);
        return TxOffer::Started;
    }
    // Pacing, a busy tunnel, or MeshCore refused the first fragment: retain it.
    Packet& slot = _queue[_queued++];
    memcpy(slot.data, data, len);
    slot.length = len;
    slot.lease = lease;
    return TxOffer::Queued;
}

// A policy refusal consumes the packet. The pump retries a false return while
// the lease is live and blocks slot 0 behind it (RustInterfacePump drainOutbound),
// and a throttled announce stays live for minutes: drop it the way Reticulum
// drops an announce over its own cap.
bool TunnelInterface::sendLeased(const uint8_t* data, size_t len, const handheld::TxLease& lease) {
    bool policyDrop = false;
    const TxOffer result = offer(data, len, lease, policyDrop);
    return result == TxOffer::Queued || result == TxOffer::Started || policyDrop;
}

// Makes `data` the current packet and sends its first fragment. On refusal
// the current slot is released again and the caller keeps the packet.
bool TunnelInterface::begin(const uint8_t* data, size_t len, const handheld::TxLease& lease) {
    memcpy(_current.packet.data, data, len);
    _current.packet.length = len;
    _current.packet.lease = lease;
    _current.packetId = _link.random32();
    _current.total = fragmentCount(len);
    _current.next = 0;
    _current.active = true;
    if (sendNextFragment()) return true;
    _current.active = false;
    return false;
}

bool TunnelInterface::sendNextFragment() {
    uint8_t sender[kSenderPrefix];
    if (!_link.senderPrefix(sender)) return false;
    uint8_t body[kMaxBody];
    const size_t n = encodeFragment(sender, _current.packetId, _current.next, _current.packet.data,
                                    _current.packet.length, body, sizeof(body));
    if (n == 0 || !_link.sendBody(body, n)) return false;
    const uint32_t now = _link.nowMs();
    _budget.note(grpDataOnAirBytes(n), now);
    _nextTxMs = now + gapMs(n);
    ++_counters.fragmentsSent;
    if (++_current.next == _current.total) {
        _current.active = false;
        ++_counters.packetsSent;
    }
    return true;
}

void TunnelInterface::dropQueued(size_t index, const char* /*cause*/) {
    const handheld::TxLease lease = _queue[index].lease;
    for (size_t i = index + 1; i < _queued; ++i) _queue[i - 1] = _queue[i];
    --_queued;
    ++_counters.queuedDropped;
    // No queue reference survives the callback, which may reenter the owner.
    notify(lease, TxReceiptEvent::Dropped);
}

void TunnelInterface::discardExpired() {
    for (size_t i = 0; i < _queued;) {
        if (!leaseLive(_queue[i].lease)) dropQueued(i, "lease expired or retired");
        else ++i;
    }
}

void TunnelInterface::startQueued() {
    if (!_queued) return;
    if (!begin(_queue[0].data, _queue[0].length, _queue[0].lease)) {
        _nextTxMs = _link.nowMs() + kRetryMs;   // stays at the front
        return;
    }
    const handheld::TxLease lease = _queue[0].lease;
    for (size_t i = 1; i < _queued; ++i) _queue[i - 1] = _queue[i];
    --_queued;
    notify(lease, TxReceiptEvent::Started);
}

void TunnelInterface::loop() {
    if (!_online || _maintenance || _changingOwner || _notifying) return;
    const uint32_t now = _link.nowMs();
    if (now - _lastExpireMs >= kExpireEveryMs) {
        _reassembler.expire(now);
        _lastExpireMs = now;
    }
    discardExpired();
    if (!_link.linkOnline() || !pacingReady()) return;
    if (_current.active) {
        if (!sendNextFragment()) _nextTxMs = now + kRetryMs;
        return;
    }
    startQueued();
}

void TunnelInterface::beginMaintenance() {
    if (_maintenance) return;
    _maintenance = true;   // gate first: callbacks may reenter
    _current.active = false;
    const bool changing = _changingOwner;
    _changingOwner = true;
    while (_queued) dropQueued(0, "maintenance");
    _changingOwner = changing;
}

// ── Receive ─────────────────────────────────────────────────────────────────

void TunnelInterface::onFragment(const Fragment& fragment) {
    if (!_online || _maintenance) return;
    size_t length = 0;
    const uint32_t now = _link.nowMs();
    if (_reassembler.accept(fragment, now, _rxPacket, length) != Reassembler::Result::Complete) return;
    _throttle.noteInbound(_rxPacket, length, now);
    ++_counters.packetsReceived;
    if (_rawSink) _rawSink(_rxPacket, length);
}

} // namespace handheld::meshcore::tunnel
