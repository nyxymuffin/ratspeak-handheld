#pragma once

// Public MeshCore types for the rest of Ratspeak. Deliberately free of MeshCore
// headers: those define bare macros (PUB_KEY_SIZE, MAX_TEXT_LEN, ...) and
// global class names, so they are included only from src/meshcore/*.cpp.

#include <stddef.h>
#include <stdint.h>

namespace handheld::meshcore {

// GRP_DATA data_type values (MeshCore Protocol Specification section 8; the
// 16-bit type is the first field of the decrypted GRP_DATA plaintext). Change
// the assigned value here and nowhere else.
enum class DataType : uint16_t {
    // MeshCore's developer namespace (DATA_TYPE_DEV in TxtDataHelpers.h),
    // used until an assigned value is agreed with the gateway side.
    RnsTunnel = 0xFFFF,
};

// Hop hash size in bytes carried in flood paths (spec section 3: 1-3 bytes,
// encoded as size-1 in bits 6-7 of path_len; code 3 is reserved).
enum class PathHashSize : uint8_t { OneByte = 1, TwoBytes = 2, ThreeBytes = 3 };

// Whether channel traffic may be repeated. ZeroHop keeps it to direct
// neighbours; Flood lets repeaters carry it across the mesh.
enum class ChannelReach : uint8_t { ZeroHop, Flood };

inline constexpr size_t kPublicKeySize = 32;    // Ed25519 (PUB_KEY_SIZE)
inline constexpr size_t kNodeNameMax = 32;      // ChannelDetails/advert name buffer
inline constexpr size_t kChannelNameMax = 32;   // ChannelDetails::name
inline constexpr size_t kChannelPskMax = 48;    // base64 of a 32-byte key, plus NUL
// Largest GRP_DATA body: MAX_GROUP_DATA_LENGTH in MeshCore.h (184 - 16 - 3).
inline constexpr size_t kMaxGroupData = 165;

struct RadioParams {
    uint32_t frequencyHz = 910525000;   // US preset, confirmed for this deployment
    uint32_t bandwidthHz = 62500;
    uint8_t spreadingFactor = 7;
    uint8_t codingRate = 5;             // 4/5
    int8_t txPowerDbm = 22;
};

struct Config {
    RadioParams radio;
    char nodeName[kNodeNameMax] = {};
    char channelName[kChannelNameMax] = {};
    // Base64 channel key (16 or 32 bytes decoded). Entered by the user at
    // runtime and stored with the settings; never compiled in.
    char channelPsk[kChannelPskMax] = {};
    PathHashSize pathHashSize = PathHashSize::OneByte;
    ChannelReach channelReach = ChannelReach::Flood;
};

struct Status {
    bool online = false;
    bool channelJoined = false;
    char publicKeyPrefix[9] = {};       // first 4 bytes of the node key, hex
    uint32_t rxPackets = 0;
    uint32_t txPackets = 0;
    uint32_t txFailures = 0;
    uint32_t dataReceived = 0;
    uint32_t dataSent = 0;
    float lastRssi = 0;
    float lastSnr = 0;
};

} // namespace handheld::meshcore
