#ifndef MAINWINDOW2_H
#define MAINWINDOW2_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include <QGraphicsEllipseItem>
#include <QTimer>
#include <QPointF>
#include <QVector>
#include <QDateTime>
#include <QResizeEvent>
#include <QShowEvent>
#include <QEvent>

#include "rosworker.h"
#include "heatmapper.h"
#include "gradientpalette.h"
#include "autoexplorer.h"
#include "dbmanager.h"
#include "heatlayer.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow2; }
QT_END_NAMESPACE

struct MapMeta
{
    QString imagePath;
    double resolution = 0.05;
    double origin_x = 0.0;
    double origin_y = 0.0;
    double origin_yaw = 0.0;
};

class MainWindow2 : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow2(QWidget *parent = nullptr);
    ~MainWindow2();

protected:
    void showEvent(QShowEvent* e) override;
    void resizeEvent(QResizeEvent* event) override;
    bool eventFilter(QObject* obj, QEvent* ev) override;

private slots:
    // 좌측 모드 버튼(겉 UI)
    void on_pBt_RunMode_clicked();    // Run = Measurement
    void on_pBt_ViewMode_clicked();   // View = DB(Query)
    void on_pBt_SimulMode_clicked();  // Simul = SimAP

    // Run(Start/Stop) (겉 UI)
    void on_pBt_Start_clicked(bool checked);

    // DB 버튼 (겉 UI)
    void on_pBt_DB_Refresh_clicked();
    void on_pBt_DB_Load_clicked();
    void on_pBt_DB_Delete_clicked();

    // SSID 콤보 변경 시 필터 적용(겉 UI)
    void onSsidChanged(int);

    // ROS
    void onRosStatus(const QString &msg);
    void onRobotPose(double x, double y, double yaw);
    void onSample(double x, double y, double yaw, float rssi);
    void onSampleToDb(double x, double y, double yaw, float rssi);

    // Heatmap flush
    void flushHeatmap();

private:
    // ====== 기존 기능 로직(유지) ======
    bool loadStaticMap(const QString& yamlPath);
    bool parseMapYaml(const QString& yamlPath, MapMeta& out);
    void initHeatmapLayer();
    void applyViewTransform();

    bool meterToPixel(double x, double y, int& px, int& py) const;
    bool pixelToMap(int px, int py, double& x_m, double& y_m) const;

    bool sceneToMapPixel(const QPointF& scenePos, int& px, int& py) const;
    void onMapClicked(const QPointF& scenePos);

    float rssiToIntensity01(float rssi) const;

    // DB
    void onMeasureStart();
    void onMeasureStop();
    void onSessionRefresh();
    void onSessionLoad();
    void onSessionDelete();
    void onApplyFilter();
    void reloadQueryLayer(const QString& sessionId);

    // Sim
    void addSimPinAt(double x_m, double y_m);
    void rebuildSimLayer();
    void onClearPins();

    float simRssiAt(double d_m) const;
    float simValueToIntensity01(double d_m) const;
    float valueToIntensity01(const SampleRow& s) const;

    // Layer policy (UI 체크박스 없음 → 내부 정책 변수로 제어)
    void applyLayersPolicy();
    void onSimEnable(bool on);

    //12_28
    QTimer* idleTimer_ = nullptr;
    QElapsedTimer idleClock_;
    bool uiHidden_ = false;
    bool swallowNextMapPress_ = false;

    void setupAutoHideUi();
    void setChromeVisible(bool on);
    void markActivity();
    bool wakeUiIfHidden();

private:
    enum class Metric { RSSI, SNR, ApCount, Noise, ThroughputMax };
    Metric currentMetric_ = Metric::RSSI; // UI 없음 → RSSI 고정

    enum class Mode { View, Measurement, Query, SimAP };
    Mode currentMode_ = Mode::Measurement; // Run 기본

    // UI 없는 옵션을 “모드 정책”으로 제어
    bool showRobot_   = true;
    bool showHeatmap_ = true;
    bool showPins_    = false;

    // Sim 파라미터 UI 없음 → 고정값으로 운용
    bool simEnabled_ = false;
    double simTxPower_ = -40.0;
    int simChannel_ = 36;
    int simBandwidth_ = 80;
    HeatLayer simLayer_;

    struct SimPin { double x_m = 0.0; double y_m = 0.0; };
    QVector<SimPin> simPins_;
    QVector<QGraphicsItem*> apPins_;

    // Filter
    QString filterSsid_ = "ALL";
    bool filterThrEnable_ = false;   // UI 없음 → 항상 false(필요하면 상수로 바꿔도 됨)
    int  filterThrRssi_ = -60;

    // DB state
    bool measuringDb_ = false;
    QString activeSessionId_;
    DbManager db_;
    HeatLayer queryLayer_;
    QString currentLoadedSession_;

    // Auto explore (UI 없음: 필요 시 코드로 enable 가능)
    bool navBusy_ = false;
    AutoExplorer autoExplorer_;

    // Qt / ROS / Graphics
    Ui::MainWindow2 *ui = nullptr;
    RosWorker *rosThread = nullptr;

    QGraphicsScene *scene = nullptr;
    QGraphicsPixmapItem *mapItem = nullptr;
    QGraphicsEllipseItem *robotItem = nullptr;

    MapMeta mapMeta_;
    QSize mapImageSize_;
    bool mapReady_ = false;
    bool poseReady_ = false;

    // Heatmap
    QImage heatCanvas_;
    GradientPalette* palette_ = nullptr;
    HeatMapper* heatMapper_ = nullptr;
    QGraphicsPixmapItem* heatItem_ = nullptr;
    QTimer* heatFlushTimer_ = nullptr;
};

#endif // MAINWINDOW2_H
