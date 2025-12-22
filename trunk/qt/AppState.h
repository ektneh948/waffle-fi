#pragma once
#include <QString>

enum class Mode { View, Measurement, Query, APSim };
enum class Metric { RSSI, SNR, APCount, Noise, ThroughputMax };

struct AppState {
    Mode mode = Mode::View;

    // Measurement: 화면 누적은 기존 그대로, DB 저장만 gate
    bool measuring = false;

    // Auto explore (goal 자동 발행)
    bool autoExplore = false;

    // DB
    QString currentSessionId;

    // Filter (MVP: Query/DB 저장에만 적용)
    QString ssid = "ALL";
    bool thrEnable = false;
    int thrRssi = -60;

    // Metric (MVP는 RSSI만)
    Metric metric = Metric::RSSI;

    // Layers
    bool showHeatmap = true;
    bool showRobot = true;
    bool showPins = true;

    // Sim (MVP에서는 파라미터 저장 + clear만)
    bool simEnable = false;
    double simTxPower = -40.0;
    int simChannel = 36;
    int simBandwidth = 80;
};
