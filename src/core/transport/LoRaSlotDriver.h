#pragma once

#include <stddef.h>
#include <stdint.h>

#include <functional>

#include "transport/TxLease.h"

// The driver behind Reticulum interface 0, the LoRa slot (RustInterfacePump
// LORA_IFACE_ID). Exactly one exists per boot, chosen by Settings > LoRa mode:
// LoRaInterface (RNode framing, native Reticulum LoRa) or, on boards built
// with RATSPEAK_MESHCORE, the MeshCore tunnel. The Rust core only ever sees
// "interface 0"; its interface count and ABI are unchanged.
//
// These are the operations RustInterfacePump and ProtocolRuntime's maintenance
// path perform on slot 0; semantics are LoRaInterface's (see LoRaInterface.h).
class LoRaSlotDriver {
public:
    using RawSink = std::function<void(const uint8_t* data, size_t len)>;
    using TxValidator = bool (*)(void*, const handheld::TxLease&);

    virtual ~LoRaSlotDriver() = default;

    // Pump wiring: RX frames go to the sink; retained TX is validated and
    // receipted through the owner's callbacks.
    virtual void setRawSink(RawSink sink) = 0;
    virtual void setTxValidator(void* context, TxValidator validator) = 0;
    virtual void setReceiptHook(void* context, handheld::TxReceiptHook hook) = 0;
    virtual uint32_t generation() const = 0;

    virtual bool isOnline() const = 0;
    virtual uint32_t bitrate() const = 0;
    virtual uint32_t txWaitBudgetMs(uint32_t packets) const = 0;
    virtual int lastRxRssi() const = 0;
    virtual float lastRxSnr() const = 0;

    virtual void loop() = 0;
    virtual bool pollBeforeBlockingWork() = 0;
    virtual handheld::TxOffer offerLeased(const uint8_t* data, size_t len, const handheld::TxLease& lease) = 0;
    virtual bool sendLeased(const uint8_t* data, size_t len, const handheld::TxLease& lease) = 0;

    // Maintenance (firmware update, factory reset, power off): close admission
    // without aborting a started burst, then report when it has drained.
    virtual void beginMaintenance() = 0;
    virtual void pollMaintenance() = 0;
    virtual bool maintenanceDrained() const = 0;
    virtual bool maintenanceFailed() const = 0;
};
