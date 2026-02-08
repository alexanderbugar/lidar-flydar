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

void NetStream::send(const scan::CloudFrame& frame, uint32_t queueDrops) {
    // FLY1. this is a rolling cloud, one lidar frame per datagram.
    WireCloud packet{};
    packet.header = {0x31594c46, sequence_++, uint64_t(frame.startUs),
                     queueDrops + sendDrops_, uint16_t(frame.count)};
    for (size_t i = 0; i < frame.count; ++i) {
        const auto& p = frame.points[i];
        packet.points[i] = {p.metres.x, p.metres.y, p.metres.z, p.offsetUs, p.intensity};
    }
    const auto* bytes = reinterpret_cast<const uint8_t*>(&packet);
    const size_t size = sizeof(CloudHeader) + frame.count * sizeof(WirePoint);
    if (!udp_.beginPacket(cfg::udpHost, cfg::udpPort)) {
        ++sendDrops_;
        return;
    }
    const size_t written = udp_.write(bytes, size);
    if (!udp_.endPacket() || written != size) ++sendDrops_;
}
