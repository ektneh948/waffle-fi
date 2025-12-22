#pragma once
#include <QObject>
#include <QTimer>
#include <QPointF>
#include <QVector>

class AutoExplorer : public QObject {
    Q_OBJECT
public:
    explicit AutoExplorer(QObject* parent = nullptr);

    void setEnabled(bool on);
    void setMapBounds(double minx, double maxx, double miny, double maxy);

    // 사용자 목표가 들어오면 잠깐 쉬는 정책(권장)
    void onUserGoal();

public slots:
    void onGoalStatus(const QString& msg); // SUCCEEDED/ABORTED 감지(선택)

signals:
    void newGoal(double x, double y, double yaw);

private:
    void tick();
    bool isFarFromVisited(const QPointF& p) const;

    QTimer timer_;
    QVector<QPointF> visited_;
    double minx_=-3, maxx_=3, miny_=-3, maxy_=3;
    bool enabled_=false;
    int pauseTicks_ = 0;
};
