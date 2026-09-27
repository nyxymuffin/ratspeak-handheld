// Host unit tests for src/core/config/MeshCoreRules.h.
// Build and run: make -C test/host/meshcore

#include <cstdio>
#include <cstring>

#include "MeshCoreRules.h"

using namespace handheld::meshcore_rules;

static int failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);          \
            ++failures;                                                          \
        }                                                                        \
    } while (0)

static bool psk(const char* value) { return validChannelPsk(value, std::strlen(value)); }
static bool name(const char* value) { return validName(value, std::strlen(value)); }

static void pskAcceptsOnlyCanonical128And256BitKeys() {
    CHECK(psk(""));                                               // no data channel
    // MeshCore's published public-channel key: a well-known value, not a secret.
    CHECK(psk("izOH6cXN6mrJ5e26oRXNcg=="));                        // 16 bytes
    CHECK(psk("AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8="));    // 32 bytes
    CHECK(!psk("izOH6cXN6mrJ5e26oRXNcg="));                        // wrong padding
    CHECK(!psk("izOH6cXN6mrJ5e26oRXNcgAA"));                       // 24 chars, no padding = 18 bytes
    CHECK(!psk("izOH6cXN6mrJ5e26oRXNc-=="));                       // base64url char
    CHECK(!psk("izOH6cXN 6mrJ5e26oRXNc=="));                       // space
    CHECK(!psk("AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh8=A"));   // 45 chars: would overrun the 32-byte secret
    CHECK(!psk("AAECAwQFBgcICQoLDA0ODxAREhMUFRYXGBkaGxwdHh=="));   // 44 chars, 31 bytes
    CHECK(!validChannelPsk(nullptr, 24));
}

static void namesArePrintableAndFit() {
    CHECK(name(""));
    CHECK(name("Nyx T-Pager"));
    CHECK(name("0123456789012345678901234567890"));     // 31 bytes
    CHECK(!name("01234567890123456789012345678901"));   // 32 bytes
    CHECK(!name("tab\there"));
    CHECK(!name("del\x7f"));
    CHECK(name("caf\xc3\xa9"));                          // UTF-8 bytes are printable here
}

static bool scope(const char* value) { return validFloodScope(value, std::strlen(value)); }
static bool unscoped(const char* value) { return unscopedFlood(value, std::strlen(value)); }

static void floodScopes() {
    CHECK(scope("") && unscoped(""));                         // unscoped
    CHECK(scope("*") && unscoped("*"));                       // MeshCore wildcard region
    CHECK(scope("ncmesh") && !unscoped("ncmesh"));
    CHECK(scope("#ncmesh"));
    CHECK(!scope("#"));
    CHECK(!scope("$private"));                                // needs a shared key
    CHECK(!scope("two words"));
    CHECK(scope("012345678901234567890123456789"));           // 30 bytes
    CHECK(!scope("0123456789012345678901234567890"));         // 31 bytes
}

int main() {
    pskAcceptsOnlyCanonical128And256BitKeys();
    namesArePrintableAndFit();
    floodScopes();
    if (failures) {
        std::printf("%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("meshcore settings rules: PASS\n");
    return 0;
}
