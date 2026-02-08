#pragma once

namespace cfg {
constexpr bool stabilize = false;
constexpr bool driveMotor = false;
constexpr int motorA = 5;
constexpr int motorB = 6;
constexpr int motorC = 7;
constexpr int motorEn = 10;
constexpr float motorSupply = 12;
constexpr int motorPolePairs = 7;
constexpr float motorVoltage = 1.5f;
constexpr float motorSpeedRad = 0.75f;
constexpr char udpHost[] = "192.168.1.50";
constexpr unsigned udpPort = 4210;
constexpr int axisSda = 11;
constexpr int axisScl = 12;
constexpr unsigned axisI2cHz = 400000;
constexpr float axisZeroRad = 0.0f;
constexpr float axisSign = 1.0f;
constexpr unsigned char axisAddress = 0x36;
constexpr int headSda = 8;
constexpr int headScl = 9;
constexpr unsigned headI2cHz = 100000;
constexpr unsigned char imuAddress = 0x68;
constexpr bool dumpMag = false;
constexpr bool magCalibrated = false;
constexpr float magBias[3] = {};
constexpr float magMatrix[3][3] = {
    {1.0f/3000, 0, 0},
    {0, 1.0f/3000, 0},
    {0, 0, 1.0f/3000},
};
constexpr unsigned char magAddress = 0x0d;
constexpr int lidarRx = 18;
constexpr unsigned lidarBaud = 230400;
}
