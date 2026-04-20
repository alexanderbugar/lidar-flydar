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
#include "ota.h"

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
std::atomic<uint32_t> uartErrors{0};
std::atomic<uint32_t> uartOverflow{0};
std::atomic<uint32_t> crcErrors{0};
std::atomic<uint32_t> fieldErrors{0};

void uartError(hardwareSerial_error_t error) {
    if (error == UART_FIFO_OVF_ERROR || error == UART_BUFFER_FULL_ERROR)
        uartOverflow.fetch_add(1, std::memory_order_relaxed);
    else
        uartErrors.fetch_add(1, std::memory_order_relaxed);
}

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
        crcErrors.store(parser.badCrc(), std::memory_order_relaxed);
        fieldErrors.store(parser.badFields(), std::memory_order_relaxed);
        vTaskDelay(1);
    }
}

bool readAttitude(int64_t now) {
    static int64_t lastPoll = 0;
    static int64_t lastSample = 0;
    if (!imuOk || now - lastPoll < 5000) return false;
    lastPoll = now;
    ImuSample sample;
    if (!imu.read(sample)) return false;
    const float dt = lastSample ? (now - lastSample) * 1e-6f : 0;
    lastSample = now;
    attitude.update(sample, dt);
    return attitude.ready();
}

void readMag() {
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
}

void work(void*) {
    scan::TimedFrame timed;
    scan::PoseHistory poses;
    unsigned seen = 0;
    uint32_t lastAxisReport = 0;
    for (;;) {
        if (xQueueReceive(frames, &timed, 1) != pdTRUE) continue;
        scan::AxisSample sample;
        while (xQueueReceive(axis, &sample, 0) == pdTRUE) {
            poses.add(sample);
            if (millis() - lastAxisReport >= 1000) {
                lastAxisReport = millis();
                Serial.printf("# axis %.3f rad\n", sample.radians);
                Serial.printf("# rx crc=%u fields=%u uart=%u ovf=%u qdrop=%u\n",
                              unsigned(crcErrors.load()), unsigned(fieldErrors.load()),
                              unsigned(uartErrors.load()), unsigned(uartOverflow.load()),
                              unsigned(queueDrops.load()));
            }
        }
        const auto& frame = timed.lidar;
        scan::CloudFrame cloud{};
        scan::project(timed, poses, cloud);
        if (net.ready()) {
            if (cloud.count) net.send(cloud, queueDrops.load());
            continue;
        }
        if (++seen % 120 == 0) {
            if (poses.at(timed.endUs, sample))
                Serial.printf("# frame axis %.3f rad at %lld us\n", sample.radians,
                              (long long)sample.us);
        }
        if (seen % 120 == 0 && cloud.count) {
            const auto p = cloud.points[0].metres;
            Serial.printf("# xyz %.3f %.3f %.3f\n", p.x, p.y, p.z);
        }
        if (!cfg::serialPolar || seen % 3 != 0) continue;
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
    Quat worldFromBase;
    int64_t lastBaseUs = 0;
    for (;;) {
        gimbal.tick();
        scan::AxisSample sample{};
        sample.us = esp_timer_get_time();
        const bool haveAxis = gimbal.position(sample.radians);
        const bool haveAttitude = readAttitude(sample.us);
        readMag();

        if (cfg::stabilize && haveAxis && haveAttitude) {
            // the imu is already turning with the head. remove the shaft
            // before smoothing or the lag makes the ceiling breathe.
            const Quat baseFromHead = Quat::about({1, 0, 0}, sample.radians);
            const Quat estimate = attitude.rotation() * baseFromHead.conjugate();
            const float dt = (sample.us - lastBaseUs) * 1e-6f;
            worldFromBase = lastBaseUs && dt < 0.05f
                ? worldFromBase.toward(estimate, dt / (0.18f + dt))
                : estimate.unit();
            lastBaseUs = sample.us;
        }

        const bool fresh = lastBaseUs && sample.us - lastBaseUs < 25000;
        sample.worldFromBase = worldFromBase;
        if (haveAxis && (!cfg::stabilize || fresh)) {
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
    lidar.onReceiveError(uartError);
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
    ota::poll();
    delay(10);
}
