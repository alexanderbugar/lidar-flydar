#pragma once

namespace cfg {
constexpr char udpHost[] = "192.168.1.50";
constexpr unsigned udpPort = 4210;
constexpr int headSda = 8;
constexpr int headScl = 9;
constexpr unsigned headI2cHz = 100000;
constexpr unsigned char imuAddress = 0x68;
constexpr unsigned char magAddress = 0x0d;
constexpr int lidarRx = 18;
constexpr unsigned lidarBaud = 230400;
}
