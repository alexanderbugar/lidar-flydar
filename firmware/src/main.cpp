#include <Arduino.h>
#include "config.h"
#include "ld_lidar.h"
#include "net_stream.h"
#include "imu.h"
#include "attitude.h"

HardwareSerial lidar(1);
ld::Parser parser;
NetStream net;
Imu imu(Wire);
Attitude attitude;
bool imuOk = false;
QueueHandle_t frames = nullptr;

void readLidar(void*) {
    ld::Frame frame;
    for (;;) {
        for (int byte; (byte = lidar.read()) >= 0;) {
            if (parser.push(byte, frame)) xQueueSend(frames, &frame, 0);
        }
        vTaskDelay(1);
    }
}

void reportTilt() {
    static uint32_t lastImu = 0;
    static unsigned reports = 0;
    const uint32_t now = micros();
    if (imuOk && now - lastImu >= 5000) {
        const float dt = (now - lastImu) * 1e-6f;
        lastImu = now;
        ImuSample sample;
        if (imu.read(sample)) {
            attitude.update(sample, dt);
            if (++reports % 200 == 0) {
                Serial.printf("# tilt %.2f %.2f\n", attitude.roll(), attitude.pitch());
            }
        }
    }

}

void work(void*) {
    ld::Frame frame;
    unsigned seen = 0;
    for (;;) {
        reportTilt();
        if (xQueueReceive(frames, &frame, 1) != pdTRUE) continue;
        if (net.ready()) {
            net.send(frame);
            continue;
        }
        if (++seen % 3 != 0) continue;
        for (size_t i = 0; i < ld::pointCount; ++i) {
            const float theta = ld::angleRad(frame, i);
            const auto point = frame.points[i];
            if (point.mm == 0) continue;
            Serial.printf("%.4f,%u,%.1f,%.1f,%u\n", theta, point.mm,
                          point.mm * cosf(theta), point.mm * sinf(theta),
                          point.intensity);
        }
    }
}

void setup() {
    net.begin();
    Serial.begin(921600);
    Wire.begin(cfg::headSda, cfg::headScl, cfg::headI2cHz);
    Wire.setTimeOut(4);
    imuOk = imu.begin() && imu.calibrate();
    if (!imuOk) Serial.println("# imu missing or moving at boot");
    lidar.setRxBufferSize(2048);
    lidar.begin(cfg::lidarBaud, SERIAL_8N1, cfg::lidarRx, -1);
    frames = xQueueCreate(16, sizeof(ld::Frame));
    if (!frames || xTaskCreatePinnedToCore(readLidar, "ld06", 3072, nullptr, 3,
                                           nullptr, 1) != pdPASS ||
        xTaskCreatePinnedToCore(work, "points", 6144, nullptr, 2,
                                nullptr, 0) != pdPASS) {
        Serial.println("# could not start tasks");
    }
}

void loop() {
    delay(1000);
}
