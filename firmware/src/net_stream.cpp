#include <Arduino.h>
#include <WiFi.h>
#include "config.h"
#include "net_stream.h"

#if __has_include("wifi_local.h")
#include "wifi_local.h"
#else
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#endif

void NetStream::begin() {
    if (!WIFI_SSID[0]) return;
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

bool NetStream::ready() const {
    return WiFi.status() == WL_CONNECTED;
}

void NetStream::send(const ld::Frame& frame) {
    PolarPacket packet{};
    packet.sequence = sequence_++;
    packet.stampMs = frame.stampMs;
    for (size_t i = 0; i < ld::pointCount; ++i) {
        const float cdeg = ld::angleRad(frame, i) * (18000.0f / PI);
        packet.points[i] = {uint16_t(unsigned(cdeg + 0.5f) % 36000u), frame.points[i].mm,
                            frame.points[i].intensity};
    }
    const auto* bytes = reinterpret_cast<const uint8_t*>(&packet);
    if (!udp_.beginPacket(cfg::udpHost, cfg::udpPort)) return;
    udp_.write(bytes, sizeof(packet));
    udp_.endPacket();
}
