#pragma once

// Minimal Reticulum packet-header reading for the tunnel's airtime policy.
// Pure and header-only (host-tested in test/host/meshcore/test_tunnel_policy.cpp).
//
// Layout per the Reticulum Manual (Y:\Standards and Specs\Mesh, section 6.7.3
// "Wire Format"): [flags][hops][address 16 or 32][context][data].
// flags: bit 7 IFAC, bit 6 header type (1 = two addresses), bit 5 context
// flag, bit 4 propagation, bits 3-2 destination type, bits 1-0 packet type.
// With two addresses the first is the relaying transport's ID and the second
// the destination.

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace handheld::meshcore::rns {

enum class PacketType : uint8_t { Data = 0, Announce = 1, LinkRequest = 2, Proof = 3 };
enum class DestinationType : uint8_t { Single = 0, Group = 1, Plain = 2, Link = 3 };

inline constexpr size_t kAddressSize = 16;
inline constexpr size_t kHeaderSize = 2;
inline constexpr size_t kContextSize = 1;

inline PacketType packetType(uint8_t flags) { return static_cast<PacketType>(flags & 0x03); }
inline DestinationType destinationType(uint8_t flags) { return static_cast<DestinationType>((flags >> 2) & 0x03); }
inline bool twoAddresses(uint8_t flags) { return (flags & 0x40) != 0; }
inline bool hasIfac(uint8_t flags) { return (flags & 0x80) != 0; }

// Offset of the data field, or 0 if the packet is too short to have one.
// Interface access codes never cross the tunnel (IFAC is per physical
// interface), so a packet with the IFAC flag is treated as unparseable.
inline size_t dataOffset(const uint8_t* packet, size_t length) {
    if (!packet || length < kHeaderSize || hasIfac(packet[0])) return 0;
    const size_t offset = kHeaderSize + (twoAddresses(packet[0]) ? 2 : 1) * kAddressSize + kContextSize;
    return length >= offset ? offset : 0;
}

// The packet's destination address (the second address in a two-address header).
inline bool destination(const uint8_t* packet, size_t length, uint8_t out[kAddressSize]) {
    if (!dataOffset(packet, length)) return false;
    memcpy(out, packet + kHeaderSize + (twoAddresses(packet[0]) ? kAddressSize : 0), kAddressSize);
    return true;
}

// A path request is a DATA packet to a PLAIN destination (the well-known
// rnstransport.path.request hash, the same for every request).
inline bool isPathRequest(const uint8_t* packet, size_t length) {
    return packet && length >= 1 && packetType(packet[0]) == PacketType::Data &&
           destinationType(packet[0]) == DestinationType::Plain;
}

inline bool isAnnounce(const uint8_t* packet, size_t length) {
    return packet && length >= 1 && packetType(packet[0]) == PacketType::Announce;
}

// The destination a path request asks for: the first 16 bytes of its data.
// Throttles must key on this, not on the address field, which is the same for
// every path request (rns-gateway found this on hardware, 2026-08-23).
inline bool pathRequestTarget(const uint8_t* packet, size_t length, uint8_t out[kAddressSize]) {
    if (!isPathRequest(packet, length)) return false;
    const size_t offset = dataOffset(packet, length);
    if (!offset || length < offset + kAddressSize) return false;
    memcpy(out, packet + offset, kAddressSize);
    return true;
}

} // namespace handheld::meshcore::rns
