#include <ArduinoOTA.h>
#include <WiFi.h>
#include "config.h"
#include "ota.h"

#if __has_include("wifi_local.h")
#include "wifi_local.h"
#endif

void ota::poll() {
#ifdef OTA_PASSWORD
    // leave the updater closed on motor builds. usb still works on those.
    if (cfg::driveMotor || !OTA_PASSWORD[0] || WiFi.status() != WL_CONNECTED) return;
    static bool started = false;
    if (!started) {
        ArduinoOTA.setHostname("lidar-flydar");
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.begin();
        started = true;
    }
    ArduinoOTA.handle();
#endif
}
