#include <Arduino.h>
#include <atomic>
#include <esp_timer.h>
#include "config.h"
#include "ld_lidar.h"
#include "net_stream.h"
#include "imu.h"
#include "attitude.h"
#include "mag.h"
#include "gimbal.h"
#include "fusion.h"

HardwareSerial lidar(1);
ld::Parser parser;
NetStream net;
Imu imu(Wire);
Mag mag(Wire);
bool magOk = false;
Attitude attitude;
Gimbal gimbal;
bool imuOk = false;
QueueHandle_t frames = nullptr;
QueueHandle_t axis = nullptr;
std::atomic<uint32_t> queueDrops{0};

void readLidar(void*) {
    scan::TimedFrame frame;
    scan::FrameClock clock;
    for (;;) {
        for (int byte; (byte = lidar.read()) >= 0;) {
            if (!parser.push(byte, frame.lidar)) continue;
            frame.endUs = clock.observe(frame.lidar.stampMs, esp_timer_get_time());
            if (xQueueSend(frames, &frame, 0) == pdTRUE) continue;

            // a stalled laptop should leave a hole, not bring back old walls.
            scan::TimedFrame stale;
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
    scan::TimedFrame timed;
    scan::PoseHistory poses;
    unsigned seen = 0;
    for (;;) {
        reportTilt();
        static uint32_t lastMag = 0;
        static uint32_t lastGoodMag = 0;
        if (magOk && millis() - lastMag >= 20) {
            lastMag = millis();
            Vec3 magCounts;
            if (mag.read(magCounts)) {
                if (cfg::dumpMag) {
                    Serial.printf("mag,%.0f,%.0f,%.0f\n", magCounts.x,
                                  magCounts.y, magCounts.z);
                } else if (cfg::magCalibrated) {
                    const float dt = lastGoodMag ? (lastMag - lastGoodMag) * 0.001f : 0;
                    attitude.correctHeading(Mag::corrected(magCounts), dt);
                    lastGoodMag = lastMag;
                }
            }
        }
        if (xQueueReceive(frames, &timed, 1) != pdTRUE) continue;
        scan::AxisSample sample;
        while (xQueueReceive(axis, &sample, 0) == pdTRUE) poses.add(sample);
        const auto& frame = timed.lidar;
        if (net.ready()) {
            net.send(frame);
            continue;
        }
        if (++seen % 120 == 0) {
            if (poses.at(timed.endUs, sample))
                Serial.printf("# frame axis %.3f rad at %lld us\n", sample.radians,
                              (long long)sample.us);
        }
        if (seen % 3 != 0) continue;
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

void readAxis(void*) {
    for (;;) {
        gimbal.tick();
        scan::AxisSample sample{};
        sample.us = esp_timer_get_time();
        if (gimbal.position(sample.radians)) {
            if (xQueueSend(axis, &sample, 0) != pdTRUE) {
                scan::AxisSample stale;
                xQueueReceive(axis, &stale, 0);
                xQueueSend(axis, &sample, 0);
            }
        }
        vTaskDelay(1);
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
    if (!gimbal.begin()) Serial.println("# gimbal init failed");
    lidar.setRxBufferSize(2048);
    lidar.begin(cfg::lidarBaud, SERIAL_8N1, cfg::lidarRx, -1);
    frames = xQueueCreate(16, sizeof(scan::TimedFrame));
    axis = xQueueCreate(48, sizeof(scan::AxisSample));
    if (!frames || !axis || xTaskCreatePinnedToCore(readLidar, "ld06", 3072, nullptr, 3,
                                           nullptr, 1) != pdPASS ||
        xTaskCreatePinnedToCore(work, "points", 6144, nullptr, 2,
                                nullptr, 0) != pdPASS ||
        xTaskCreatePinnedToCore(readAxis, "axis", 4096, nullptr, 3,
                                nullptr, 0) != pdPASS) {
        Serial.println("# could not start tasks");
    }
}

void loop() {
    delay(1000);
}
