#include <Arduino.h>
#include "config.h"
#include "ld_lidar.h"

HardwareSerial lidar(1);
ld::Parser parser;

void setup() {
    Serial.begin(115200);
    lidar.begin(cfg::lidarBaud, SERIAL_8N1, cfg::lidarRx, -1);
}

void loop() {
    ld::Frame frame;
    static unsigned frames = 0;
    for (int byte; (byte = lidar.read()) >= 0;) {
        if (parser.push(byte, frame) && ++frames % 16 == 0) {
            Serial.printf("%u %u..%u %u mm\n", frame.stampMs,
                          frame.startCdeg, frame.endCdeg, frame.points[0].mm);
        }
    }
    delay(1);
}
