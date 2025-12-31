#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QImage>
#include <QTimer>
#include <QPointF>
#include <QVector>
#include <QSize>
#include <QSet>
#include <QDir>


#include "rosworker.h"
#include "autoexplorer.h"
#include "heatlayer.h"
#include "heatmapper.h"
#include "gradientpalette.h"
#include "legendbarwidget.h"

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

    enum class Mode { View, Measurement, Query, SimAP };

private slots:
    // ROS
    void onRosStatus(const QString &msg);
    void onRobotPose(double x, double y, double yaw);

    // dummy (선택: test 모드에서만 사용)
    void onSample(double x, double y, double yaw, float rssi);

    // Live heatmap from /wifi/fused
    void onFusedSample(double x_m, double y_m, const QString& ssid, int rssi);

    // (옵션) gridPose를 쓰면 유지, 안 쓰면 제거 가능
    void onGridPose(double gx, double gy);

    // RL (현재는 수신 로그만)
    void onRlStateArrived();

    // Heatmap flush
    void flushHeatmap();

    // UI toggles
    void onLayerHeatmap(bool on);
    void onLayerRobot(bool on);
    void onLayerPins(bool on);

    void onAutoExploreToggled(bool on);

    // Snapshot Sessions
    void onSessionRefresh();
    void onSessionLoad();
    void onSessionDelete();
    void on_btnSessionLoad_clicked();   // 오토슬롯
    void onQueryClear();

    // Start/Stop (live 누적 ON/OFF + Start 시 snapshot 생성)
    void onMeasureStart();
    void onMeasureStop();

    // Filter
    void onApplyFilter();

    // Sim
    void onSimEnable(bool on);
    void onSimParamsChanged();
    void onClearPins();

    // Admin (비활성화됨)
    void onAdminClicked();

protected:
    void showEvent(QShowEvent* e) override;
    void resizeEvent(QResizeEvent* event) override;
    bool eventFilter(QObject* obj, QEvent* ev) override;

private:
    // Context/Policy
    Mode deriveModeFromContext() const;
    void updateUiByContext();
    void applyLayersPolicy();

    // Map
    bool loadStaticMap(const QString& yamlPath);
    bool parseMapYaml(const QString& yamlPath, MapMeta& out);
    void applyViewTransform();

    bool meterToPixel(double x, double y, int& px, int& py) const;
    bool pixelToMap(int px, int py, double& x_m, double& y_m) const;

    // Click / Nav
    bool sceneToMapPixel(const QPointF& scenePos, int& px, int& py) const;
    void onMapClicked(const QPointF& scenePos);

    // Live Heatmap
    void initHeatmapLayer();
    float rssiToIntensity01(float rssi) const;
    int radiusPx() const;

    // Snapshot query (sqlite3 READONLY)
    QString makeDbSnapshot();
    void reloadQueryLayerFromSnapshot(const QString& snapshotPath);

    // SSID combo live update
    void updateSsidComboLive(const QString& ssid);

    // Sim
    struct SimPin { double x_m = 0.0; double y_m = 0.0; };
    void addSimPinAt(double x_m, double y_m);
    void rebuildSimLayer();
    float simRssiAt(double d_m) const;

    // Legend overlay
    void initLegendOverlay();
    void updateLegendOverlayGeometry();

    int radiusPxLive() const;
    int simBrushRadiusPx() const;

private:
    Ui::MainWindow *ui = nullptr;
    RosWorker *rosThread = nullptr;

    QGraphicsScene *scene = nullptr;
    QGraphicsPixmapItem *mapItem = nullptr;
    QGraphicsEllipseItem *robotItem = nullptr;

    // Map
    MapMeta mapMeta_;
    QSize mapImageSize_;
    bool mapReady_ = false;
    bool poseReady_ = false;

    // Live heatmap (HeatMapper)
    QImage heatCanvas_;
    GradientPalette* palette_ = nullptr;
    HeatMapper* heatMapper_ = nullptr;
    QGraphicsPixmapItem* heatItem_ = nullptr;
    QTimer* heatFlushTimer_ = nullptr;

    // Query/Sim layers (HeatLayer)
    HeatLayer queryLayer_;
    HeatLayer simLayer_;

    // UI/State
    Mode currentMode_ = Mode::View;

    // ====== Snapshot 방식 상태 ======
    QString sourceDbPath_ = "/home/ros/turtlebot3_ws/data/my_wifi.db";
    QString snapshotDir_  = QDir::homePath() + "/wifi_qt_snapshots";
    QString loadedSnapshotPath_;

    // Start/Stop → live heatmap 누적 ON/OFF
    bool accumulating_ = false;

    // Live SSID set
    QSet<QString> ssidSetLive_;

    // Filter
    QString filterSsid_ = "ALL";
    bool filterThrEnable_ = true; // UI에서 chkThrEnable 숨김이므로 기본 true 추천
    int filterThrRssi_ = -70;

    // Sim
    bool simEnabled_ = false;
    double simTxPower_ = -40.0;
    int simChannel_ = 36;
    int simBandwidth_ = 80;
    QVector<SimPin> simPins_;
    QVector<QGraphicsItem*> apPins_;

    // Nav / Auto explorer
    bool navBusy_ = false;
    AutoExplorer autoExplorer_;

    // Legend
    LegendBarWidget* legendOverlay_ = nullptr;
};

#endif // MAINWINDOW_H
