#include <Arduino.h>
#include <atomic>
#include "config.h"
#include "ld_lidar.h"
#include "net_stream.h"
#include "imu.h"
#include "attitude.h"
#include "mag.h"

HardwareSerial lidar(1);
ld::Parser parser;
NetStream net;
Imu imu(Wire);
Mag mag(Wire);
bool magOk = false;
Attitude attitude;
bool imuOk = false;
QueueHandle_t frames = nullptr;
std::atomic<uint32_t> queueDrops{0};

void readLidar(void*) {
    ld::Frame frame;
    for (;;) {
        for (int byte; (byte = lidar.read()) >= 0;) {
            if (!parser.push(byte, frame)) continue;
            if (xQueueSend(frames, &frame, 0) == pdTRUE) continue;

            // a stalled laptop should leave a hole, not bring back old walls.
            ld::Frame stale;
            if (xQueueReceive(frames, &stale, 0) == pdTRUE)
                queueDrops.fetch_add(1, std::memory_order_relaxed);
            if (xQueueSend(frames, &frame, 0) != pdTRUE)
                queueDrops.fetch_add(1, std::memory_order_relaxed);
        }
        vTaskDelay(1);
    }
}

void reportTilt() {
    static uint32_t lastPoll = 0;
    static uint32_t lastSample = 0;
    static unsigned reports = 0;
    const uint32_t now = micros();
    if (imuOk && now - lastPoll >= 5000) {
        lastPoll = now;
        ImuSample sample;
        if (imu.read(sample)) {
            const float dt = lastSample ? (now - lastSample) * 1e-6f : 0;
            lastSample = now;
            attitude.update(sample, dt);
            if (++reports % 200 == 0) {
                Serial.printf("# tilt %.2f %.2f qdrop=%lu\n", attitude.roll(),
                              attitude.pitch(), (unsigned long)queueDrops.load());
            }
        }
    }

}

void work(void*) {
    ld::Frame frame;
    unsigned seen = 0;
    for (;;) {
        reportTilt();
        static uint32_t lastMag = 0;
        static unsigned magReads = 0;
        if (magOk && millis() - lastMag >= 20) {
            lastMag = millis();
            Vec3 magCounts;
            if (mag.read(magCounts)) {
                if (cfg::dumpMag) {
                    Serial.printf("mag,%.0f,%.0f,%.0f\n", magCounts.x,
                                  magCounts.y, magCounts.z);
                } else if (cfg::magCalibrated && ++magReads % 50 == 0) {
                    Serial.printf("# heading %.3f\n", attitude.heading(Mag::corrected(magCounts)));
                }
            }
        }
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
    magOk = mag.begin();
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
