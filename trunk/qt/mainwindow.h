#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QImage>
#include <QTimer>
#include <QPointF>

#include "rosworker.h"
#include "heatmapper.h"
#include "gradientpalette.h"
#include "autoexplorer.h"
#include "dbmanager.h"
#include "heatlayer.h"


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct MapMeta
{
    QString imagePath;
    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onApplyFilter();
    void onRosStatus(const QString &msg);
    void onRobotPose(double x, double y, double yaw);
    void onSample(double x, double y, double yaw, float rssi);
    void flushHeatmap();
    void onLayerHeatmap(bool on);
    void onLayerRobot(bool on);
    void onLayerPins(bool on);
    void onAutoExploreToggled(bool on);
    void onGoalStatusMsg(const QString& msg);
    void onSessionRefresh();
    void onSessionLoad();
    void onSessionDelete();
    void onMeasureStart();
    void onMeasureStop();
    void onSampleToDb(double x, double y, double yaw, float rssi);
    void onSimEnable(bool on);
    void onSimParamsChanged();
    void onClearPins(); //지우기 버튼 빼자
    void onModeChanged(); //
    void applyModeUi();
    void onMetricChanged(int index);


    void on_btnSessionLoad_clicked();

protected:
    void showEvent(QShowEvent* e) override;
    void resizeEvent(QResizeEvent* event) override;
    bool eventFilter(QObject* obj, QEvent* ev) override;

private:
    void applyLayersPolicy();
    void reloadQueryLayer(const QString& sessionId);
    bool loadStaticMap(const QString& yamlPath);
    bool parseMapYaml(const QString& yamlPath, MapMeta& out);

    bool meterToPixel(double x, double y, int& px, int& py) const;
    bool pixelToMap(int px, int py, double& x_m, double& y_m) const;

    void applyViewTransform();

    bool sceneToMapPixel(const QPointF& scenePos, int& px, int& py) const;
    void onMapClicked(const QPointF& scenePos);

    void initHeatmapLayer();
    float rssiToIntensity01(float rssi) const;
    void onAutoGoal(double x, double y, double yaw);
    void addSimPinAt(double x_m, double y_m);
    void rebuildSimLayer();
    float simRssiAt(double d_m) const;      // 거리→RSSI(dBm)
    float simIntensityAt(double d_m) const; // 거리→intensity(0..1)
    float valueToIntensity01(const SampleRow& s) const; // DB/query용
    float simValueToIntensity01(double d_m) const;      // sim용



private:
    enum class Metric { RSSI, SNR, ApCount, Noise, ThroughputMax };
    Metric currentMetric_ = Metric::RSSI;
    enum class Mode { View, Measurement, Query, SimAP };
    Mode currentMode_ = Mode::View;
    bool simEnabled_ = false;
    double simTxPower_ = -40.0;
    int simChannel_ = 36;
    int simBandwidth_ = 80;
    HeatLayer simLayer_;

    struct SimPin {
        double x_m = 0.0;
        double y_m = 0.0;
    };
    QVector<SimPin> simPins_;

    QString filterSsid_ = "ALL";
    bool filterThrEnable_ = false;
    int filterThrRssi_ = -60;
    bool measuringDb_ = false;
    QString activeSessionId_;
    DbManager db_;
    HeatLayer queryLayer_;
    QString currentLoadedSession_; // state 구조 안 쓰면 이것만으로도 OK

    bool navBusy_ = false;
    QVector<QGraphicsItem*> apPins_;
    AutoExplorer autoExplorer_;
    Ui::MainWindow *ui = nullptr;
    RosWorker *rosThread = nullptr;

    QGraphicsScene *scene = nullptr;
    QGraphicsPixmapItem *mapItem = nullptr;
    QGraphicsEllipseItem *robotItem = nullptr;

    MapMeta mapMeta_;
    QSize mapImageSize_;
    bool mapReady_ = false;
    bool poseReady_ = false;

    QImage heatCanvas_;
    GradientPalette* palette_ = nullptr;
    HeatMapper* heatMapper_ = nullptr;
    QGraphicsPixmapItem* heatItem_ = nullptr;
    QTimer* heatFlushTimer_ = nullptr;
};

#endif
