// Link probe for phase 2b: nothing starts the MeshCore service until the
// board is wired up in phase 2c, so without a reference the linker would
// discard it and the build would prove nothing. Nothing calls this function.
// `used` stops the compiler dropping it and `-Wl,-u,ratspeak_meshcore_link_probe`
// in platformio.ini stops --gc-sections dropping it, so the whole service,
// host, radio adapter and vendored MeshCore must resolve. Removed in phase 2c
// together with that flag.

#include "MeshCoreService.h"

extern "C" __attribute__((used)) void ratspeak_meshcore_link_probe(BoardRadio* radio, FlashStore* flash) {
    handheld::meshcore::Service service(*radio, *flash);
    service.begin(handheld::meshcore::Config{});
    service.loop();
    service.sendData(handheld::meshcore::DataType::RnsTunnel, nullptr, 0);
    service.sendAdvert(false);
    (void)service.status();
}
