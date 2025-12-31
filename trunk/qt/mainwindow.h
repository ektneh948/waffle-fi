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

#include "rosworker.h"
#include "autoexplorer.h"
#include "dbmanager.h"
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
    void onApplyFilter();
    void onRosStatus(const QString &msg);
    void onRobotPose(double x, double y, double yaw);
    void onSample(double x, double y, double yaw, float rssi);
    void flushHeatmap();

    void onLayerHeatmap(bool on);
    void onLayerRobot(bool on);
    void onLayerPins(bool on);

    void onAutoExploreToggled(bool on);

    void onSessionRefresh();
    void onSessionLoad();
    void onSessionDelete();

    void onMeasureStart();
    void onMeasureStop();
    void onSampleToDb(double x, double y, double yaw, float rssi);

    void onSimEnable(bool on);
    void onSimParamsChanged();
    void onClearPins();

    void on_btnSessionLoad_clicked();   // ✅ 오토슬롯만 사용(중복호출 방지)
    void onQueryClear();                // ✅ Clear 버튼
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

    // Query
    void reloadQueryLayer(const QString& sessionId);

    // Sim
    struct SimPin { double x_m = 0.0; double y_m = 0.0; };
    void addSimPinAt(double x_m, double y_m);
    void rebuildSimLayer();
    float simRssiAt(double d_m) const;

    // Legend overlay
    void initLegendOverlay();
    void updateLegendOverlayGeometry();

    int radiusPxLive() const;
    int radiusPxSim() const;
    int simBrushRadiusPx() const;
    float rssiToIntensity01_continuous(float rssi) const;


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

    bool measuringDb_ = false;
    QString activeSessionId_;

    QString currentLoadedSession_;

    QString filterSsid_ = "ALL";
    bool filterThrEnable_ = false;
    int filterThrRssi_ = -60;

    bool simEnabled_ = false;
    double simTxPower_ = -40.0;
    int simChannel_ = 36;
    int simBandwidth_ = 80;
    QVector<SimPin> simPins_;

    QVector<QGraphicsItem*> apPins_;

    bool navBusy_ = false;
    AutoExplorer autoExplorer_;

    DbManager db_;

    LegendBarWidget* legendOverlay_ = nullptr;
};

#endif
