#include <Arduino.h>
#include "config.h"
#include "ld_lidar.h"

HardwareSerial lidar(1);
ld::Parser parser;

void setup() {
    Serial.begin(921600);
    lidar.begin(cfg::lidarBaud, SERIAL_8N1, cfg::lidarRx, -1);
}

void loop() {
    ld::Frame frame;
    static unsigned frames = 0;
    for (int byte; (byte = lidar.read()) >= 0;) {
        if (!parser.push(byte, frame) || ++frames % 3 != 0) continue;
        for (size_t i = 0; i < ld::pointCount; ++i) {
            const float theta = ld::angleRad(frame, i);
            const auto point = frame.points[i];
            if (point.mm == 0) continue;
            Serial.printf("%.4f,%u,%.1f,%.1f,%u\n", theta, point.mm,
                          point.mm * cosf(theta), point.mm * sinf(theta),
                          point.intensity);
        }
    }
    delay(1);
}
