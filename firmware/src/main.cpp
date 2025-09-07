#include <Arduino.h>
#include "config.h"

HardwareSerial lidar(1);

void setup() {
    Serial.begin(115200);
    lidar.begin(cfg::lidarBaud, SERIAL_8N1, cfg::lidarRx, -1);
}

void loop() {
    // stop after a few lines or the debug port falls behind immediately
    static unsigned dumped = 0;
    while (lidar.available()) {
        const int byte = lidar.read();
        if (byte < 0 || dumped >= 192) continue;
        Serial.printf("%02X%c", byte, (++dumped % 16) ? ' ' : '\n');
    }
    delay(1);
}
