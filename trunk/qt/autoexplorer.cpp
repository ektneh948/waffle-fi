#include "autoexplorer.h"
#include <QRandomGenerator>
#include <QLineF>

AutoExplorer::AutoExplorer(QObject* parent) : QObject(parent) {
    timer_.setInterval(2500);
    connect(&timer_, &QTimer::timeout, this, &AutoExplorer::tick);
}

void AutoExplorer::setEnabled(bool on) {
    enabled_ = on;
    if (enabled_) timer_.start();
    else timer_.stop();
}

void AutoExplorer::setMapBounds(double minx, double maxx, double miny, double maxy) {
    minx_ = minx; maxx_ = maxx; miny_ = miny; maxy_ = maxy;
}

void AutoExplorer::onUserGoal() {
    // 사용자 목표를 찍으면 자동탐색 잠깐 쉬기(2회 tick)
    pauseTicks_ = 2;
}

void AutoExplorer::onGoalStatus(const QString& msg) {
    // MVP: 결과를 기반으로 다음 goal 타이밍을 더 똑똑하게 만들 수 있음
    (void)msg;
}

bool AutoExplorer::isFarFromVisited(const QPointF& p) const {
    constexpr double minDist = 1.0; // meters
    for (const auto& v : visited_) {
        if (QLineF(p, v).length() < minDist) return false;
    }
    return true;
}

void AutoExplorer::tick() {
    if (!enabled_) return;
    if (pauseTicks_ > 0) { pauseTicks_--; return; }

    for (int tries=0; tries<50; ++tries) {
        auto* rng = QRandomGenerator::global();
        const double x = minx_ + (maxx_ - minx_) * rng->generateDouble();
        const double y = miny_ + (maxy_ - miny_) * rng->generateDouble();
        QPointF p(x, y);
        if (!isFarFromVisited(p)) continue;

        visited_.push_back(p);
        emit newGoal(x, y, 0.0);
        return;
    }
}
