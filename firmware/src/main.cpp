#include <Arduino.h>
#include "config.h"

void setup() {
    pinMode(cfg::benchLed, OUTPUT);
}

void loop() {
    digitalWrite(cfg::benchLed, !digitalRead(cfg::benchLed));
    delay(250);
}
