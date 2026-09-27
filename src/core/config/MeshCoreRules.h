#pragma once

// Validation rules for the MeshCore LoRa settings. Pure (no Arduino types) so
// they are unit-tested on the host (test/host/meshcore/test_settings_rules.cpp).

#include <stddef.h>
#include <stdint.h>

namespace handheld::meshcore_rules {

// Node and channel names live in 32-byte MeshCore buffers (advert name,
// ChannelDetails::name), so 31 bytes plus the terminator.
inline constexpr size_t kMaxNameBytes = 31;
// Canonical base64 of a 16-byte key ("==" padded) or a 32-byte key ("=" padded).
inline constexpr size_t kPsk128Length = 24;
inline constexpr size_t kPsk256Length = 44;

inline constexpr bool isBase64Char(char c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
           c == '+' || c == '/';
}

// Empty means "no data channel". Otherwise the key must decode to exactly 16
// or 32 bytes, the only sizes BaseChatMesh::addChannel accepts.
inline bool validChannelPsk(const char* value, size_t length) {
    if (length == 0) return true;
    if (!value) return false;
    size_t padding = 0;
    if (length == kPsk128Length) padding = 2;
    else if (length == kPsk256Length) padding = 1;
    else return false;
    for (size_t i = 0; i < length - padding; ++i)
        if (!isBase64Char(value[i])) return false;
    for (size_t i = length - padding; i < length; ++i)
        if (value[i] != '=') return false;
    return true;
}

// Printable text that fits the MeshCore buffer; empty is allowed (the node
// name then falls back to the Ratspeak display name).
inline bool validName(const char* value, size_t length) {
    if (length > kMaxNameBytes) return false;
    for (size_t i = 0; i < length; ++i)
        if (static_cast<unsigned char>(value[i]) < 0x20 || value[i] == 0x7F) return false;
    return true;
}

// MeshCore region names are stored in a 31-byte field; the companion firmware
// accepts 1-30 characters for a default flood scope.
inline constexpr size_t kMaxFloodScopeBytes = 30;

// Flood scope: empty or "*" floods unscoped (MeshCore's wildcard region). Any
// other value names a public region ("name" or "#name"); repeaters derive its
// key from the name. Private "$" regions need a shared key, so are refused.
inline bool validFloodScope(const char* value, size_t length) {
    if (length == 0) return true;
    if (!value || length > kMaxFloodScopeBytes || value[0] == '$') return false;
    if (length == 1 && value[0] == '#') return false;
    for (size_t i = 0; i < length; ++i)
        if (static_cast<unsigned char>(value[i]) <= 0x20 || value[i] == 0x7F) return false;
    return true;
}

inline bool unscopedFlood(const char* value, size_t length) {
    return length == 0 || (length == 1 && value[0] == '*');
}

} // namespace handheld::meshcore_rules
