#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "dbmanager.h"

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QImage>
#include <QPixmap>
#include <QDebug>
#include <cmath>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QShowEvent>

static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->viewport()->installEventFilter(this);

    mapItem = scene->addPixmap(QPixmap());
    mapItem->setZValue(0);

    robotItem = scene->addEllipse(-5, -5, 10, 10, QPen(Qt::red), QBrush(Qt::red));
    robotItem->setZValue(10);
    robotItem->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
    robotItem->setVisible(true);

    //const QString yamlPath = "/home/ubuntu/ros2/maps/map_simul_world.yaml";
    const QString yamlPath ="/home/ubuntu/intel-edge-ai-sw-8/2601_4th_proj_dahyeon/docs/maps/map.yaml";


    if (!loadStaticMap(yamlPath)) {
        qDebug() << "Failed to load map yaml:" << yamlPath;
    } else {
        mapReady_ = true;
    }

    if (mapReady_) {
        simLayer_.init(scene, mapImageSize_, /*z=*/7);
        simLayer_.setVisible(false);
    }


    initHeatmapLayer();

    // DB open/init
    db_.open(QDir::homePath() + "/wifi_qt.db");
    db_.initSchema();

    if (mapReady_) {
        queryLayer_.init(scene, mapImageSize_, /*z=*/6); // heatItem_(z=5) 위에
        queryLayer_.setVisible(false); // Load 전엔 숨김
    }

    rosThread = new RosWorker(this);
    connect(rosThread, &RosWorker::statusChanged, this, &MainWindow::onRosStatus);
    connect(rosThread, &RosWorker::robotPose, this, &MainWindow::onRobotPose);
    connect(rosThread, &RosWorker::sample, this, &MainWindow::onSample);
    connect(rosThread, &RosWorker::sample, this, &MainWindow::onSampleToDb);
    rosThread->start();

    connect(ui->chkShowHeatmap, &QCheckBox::toggled,
            this, &MainWindow::onLayerHeatmap);

    connect(ui->chkShowRobot, &QCheckBox::toggled,
            this, &MainWindow::onLayerRobot);

    connect(ui->chkShowApPins, &QCheckBox::toggled,
            this, &MainWindow::onLayerPins);

    // AutoExplore checkbox
    connect(ui->chkAutoExplore, &QCheckBox::toggled,
            this, &MainWindow::onAutoExploreToggled);

    connect(ui->btnSessionRefresh, &QPushButton::clicked, this, &MainWindow::onSessionRefresh);
    connect(ui->btnSessionLoad,    &QPushButton::clicked, this, &MainWindow::onSessionLoad);
    connect(ui->btnSessionDelete,  &QPushButton::clicked, this, &MainWindow::onSessionDelete);
    connect(ui->btnMeasureStart, &QPushButton::clicked, this, &MainWindow::onMeasureStart);
    connect(ui->btnMeasureStop,  &QPushButton::clicked, this, &MainWindow::onMeasureStop);
    connect(ui->btnApplyFilter, &QPushButton::clicked,
            this, &MainWindow::onApplyFilter);
    connect(ui->chkSimEnable, &QCheckBox::toggled, this, &MainWindow::onSimEnable);
    connect(ui->spTxPower, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &MainWindow::onSimParamsChanged);
    connect(ui->spSimChannel, QOverload<int>::of(&QSpinBox::valueChanged), this, &MainWindow::onSimParamsChanged);
    connect(ui->cbSimBandwidth, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::onSimParamsChanged);
    connect(ui->btnClearPins, &QPushButton::clicked, this, &MainWindow::onClearPins);
    connect(ui->rbModeView,    &QRadioButton::toggled, this, &MainWindow::onModeChanged);
    connect(ui->rbModeMeasure, &QRadioButton::toggled, this, &MainWindow::onModeChanged);
    connect(ui->rbModeQuery,   &QRadioButton::toggled, this, &MainWindow::onModeChanged);
    connect(ui->rbModeSim,     &QRadioButton::toggled, this, &MainWindow::onModeChanged);
    connect(ui->cbMetric, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::onMetricChanged);

    onMetricChanged(ui->cbMetric->currentIndex());

    onSessionRefresh();
    applyLayersPolicy();

    connect(rosThread, &RosWorker::goalStatus,
            &autoExplorer_, &AutoExplorer::onGoalStatus);
    connect(&autoExplorer_, &AutoExplorer::newGoal,
            this, &MainWindow::onAutoGoal);

}

MainWindow::~MainWindow()
{
    if (rosThread) {
        rosThread->requestInterruption();
        rosThread->wait();
    }

    delete heatFlushTimer_;
    heatFlushTimer_ = nullptr;

    delete heatMapper_;
    heatMapper_ = nullptr;

    delete palette_;
    palette_ = nullptr;

    delete ui;
}

void MainWindow::onRosStatus(const QString &msg)
{
    qDebug() << msg;
}

bool MainWindow::loadStaticMap(const QString &yamlPath)
{
    MapMeta meta;
    if (!parseMapYaml(yamlPath, meta)) return false;

    QImage img(meta.imagePath);
    if (img.isNull()) return false;

    mapItem->setPixmap(QPixmap::fromImage(img));
    mapMeta_ = meta;
    mapImageSize_ = img.size();

    scene->setSceneRect(QRectF(0, 0, mapImageSize_.width(), mapImageSize_.height()));
    applyViewTransform();

    return true;
}

bool MainWindow::parseMapYaml(const QString &yamlPath, MapMeta &out)
{
    QFile f(yamlPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) return false;

    const QFileInfo yinfo(yamlPath);

    while (!f.atEnd()) {
        QString line = QString::fromUtf8(f.readLine()).trimmed();
        if (line.isEmpty() || line.startsWith("#")) continue;

        if (line.startsWith("image:")) {
            QString v = line.mid(QString("image:").size()).trimmed();
            v.remove("\""); v.remove("'");
            out.imagePath = QFileInfo(yinfo.dir(), v).absoluteFilePath();
        }
        else if (line.startsWith("resolution:")) {
            out.resolution = line.mid(QString("resolution:").size()).trimmed().toDouble();
        }
        else if (line.startsWith("origin:")) {
            int l = line.indexOf('['), r = line.indexOf(']');
            if (l >= 0 && r > l) {
                auto parts = line.mid(l + 1, r - l - 1).split(',', Qt::SkipEmptyParts);
                if (parts.size() >= 2) {
                    out.origin_x = parts[0].trimmed().toDouble();
                    out.origin_y = parts[1].trimmed().toDouble();
                }
                if (parts.size() >= 3) {
                    out.origin_yaw = parts[2].trimmed().toDouble();
                }
            }
        }
    }

    return !out.imagePath.isEmpty() && out.resolution > 0.0;
}

void MainWindow::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    applyViewTransform();
}

void MainWindow::showEvent(QShowEvent* e)
{
    QMainWindow::showEvent(e);
    QTimer::singleShot(0, this, [this]{
        applyViewTransform();
    });
}

void MainWindow::applyViewTransform()
{
    if (!ui->graphicsView || !scene) return;
    if (scene->sceneRect().isEmpty()) return;

    auto* v = ui->graphicsView;

    v->setTransformationAnchor(QGraphicsView::AnchorViewCenter);
    v->setResizeAnchor(QGraphicsView::AnchorViewCenter);
    v->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    v->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    v->setRenderHint(QPainter::SmoothPixmapTransform, true);
    v->resetTransform();

    QTransform rot;
    rot.rotate(-90.0);
    v->setTransform(rot, true);

    v->resetTransform();
    v->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);

    QTransform t = v->transform();
    t.rotate(-90.0);
    v->setTransform(t, false);

    v->centerOn(scene->sceneRect().center());
}

bool MainWindow::meterToPixel(double x, double y, int& px, int& py) const
{
    if (mapImageSize_.isEmpty()) return false;

    const double res = mapMeta_.resolution;
    const double ox  = mapMeta_.origin_x;
    const double oy  = mapMeta_.origin_y;
    const double th  = mapMeta_.origin_yaw;

    const double dx = x - ox;
    const double dy = y - oy;

    const double c = std::cos(th);
    const double s = std::sin(th);
    const double mx =  c * dx + s * dy;
    const double my = -s * dx + c * dy;

    const double fx = mx / res;
    const double fy = my / res;

    int gx = static_cast<int>(std::floor(fx));
    int gy = static_cast<int>(std::floor(fy));

    if (gx < 0 || gy < 0 || gx >= mapImageSize_.width() || gy >= mapImageSize_.height())
        return false;

    px = gx;
    py = (mapImageSize_.height() - 1) - gy;
    return true;
}

void MainWindow::onRobotPose(double x, double y, double yaw)
{

    qDebug() << "[POSE]" << x << y << yaw;
    if (!mapReady_) return;

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    robotItem->setPos(QPointF(px, py));
    robotItem->setRotation(-rad2deg(yaw));

    if (!poseReady_) {
        poseReady_ = true;
        robotItem->setVisible(true);
    }
}

void MainWindow::initHeatmapLayer()
{
    if (!mapReady_ || mapImageSize_.isEmpty() || !scene) return;

    const int w = mapImageSize_.width();
    const int h = mapImageSize_.height();

    heatCanvas_ = QImage(w, h, QImage::Format_ARGB32);
    heatCanvas_.fill(Qt::transparent);

    delete palette_;
    palette_ = new GradientPalette(256);
    palette_->setColorAt(0.0, QColor(255, 0, 0));
    palette_->setColorAt(0.5, QColor(255, 255, 0));
    palette_->setColorAt(1.0, QColor(0, 255, 0));

    delete heatMapper_;
    heatMapper_ = new HeatMapper(&heatCanvas_, palette_, /*radius=*/18, /*opacity=*/160);

    if (!heatItem_) {
        heatItem_ = scene->addPixmap(QPixmap::fromImage(heatCanvas_));
        heatItem_->setZValue(5);
        heatItem_->setPos(0, 0);
    } else {
        heatItem_->setPixmap(QPixmap::fromImage(heatCanvas_));
    }

    if (!heatFlushTimer_) {
        heatFlushTimer_ = new QTimer(this);
        connect(heatFlushTimer_, &QTimer::timeout, this, &MainWindow::flushHeatmap);
        heatFlushTimer_->start(100);
    }
}

float MainWindow::rssiToIntensity01(float rssi) const
{
    // clamp [-90, -30]
    if (rssi < -90.f) rssi = -90.f;
    if (rssi > -30.f) rssi = -30.f;

    // -90 -> 0 (bad), -30 -> 1 (good)
    return (rssi + 90.f) / 60.f;
}

void MainWindow::onSample(double x, double y, double /*yaw*/, float rssi)
{
    if (!mapReady_ || !heatMapper_) return;

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    const float intensity = rssiToIntensity01(rssi);
    heatMapper_->addPoint(px + 1, py + 1, intensity);
}

void MainWindow::flushHeatmap()
{
    if (!heatMapper_ || !heatItem_) return;

    heatMapper_->colorize();
    heatItem_->setPixmap(QPixmap::fromImage(heatCanvas_));
}

bool MainWindow::sceneToMapPixel(const QPointF& scenePos, int& px, int& py) const
{
    if (!mapReady_ || mapImageSize_.isEmpty() || !mapItem) return false;

    QPointF local = mapItem->mapFromScene(scenePos);
    int ix = static_cast<int>(std::floor(local.x()));
    int iy = static_cast<int>(std::floor(local.y()));

    if (ix < 0 || iy < 0 || ix >= mapImageSize_.width() || iy >= mapImageSize_.height())
        return false;

    px = ix;
    py = iy;
    return true;
}

bool MainWindow::eventFilter(QObject* obj, QEvent* ev)
{
    if (obj == ui->graphicsView->viewport() &&
        ev->type() == QEvent::MouseButtonPress) {

        auto* me = static_cast<QMouseEvent*>(ev);
        const QPointF scenePos = ui->graphicsView->mapToScene(me->pos());

        const bool simEnabled = (currentMode_ == Mode::SimAP) && simEnabled_;
        const bool simGesture = (me->button() == Qt::RightButton) ||
                                (me->button() == Qt::LeftButton && (me->modifiers() & Qt::ShiftModifier));

        if (simEnabled && simGesture) {
            int px=0, py=0;
            if (!sceneToMapPixel(scenePos, px, py)) return true;

            double x_m=0.0, y_m=0.0;
            if (!pixelToMap(px, py, x_m, y_m)) return true;

            addSimPinAt(x_m, y_m);
            return true;
        }
        onMapClicked(scenePos);
        return true;
    }
    return false;
}


void MainWindow::onMapClicked(const QPointF& scenePos)
{
    int px=0, py=0;
    if (!sceneToMapPixel(scenePos, px, py)) {
        qDebug() << "Clicked outside map";
        return;
    }

    double x_m=0.0, y_m=0.0;
    if (!pixelToMap(px, py, x_m, y_m)) {
        qDebug() << "pixelToMap failed";
        return;
    }

    rosThread->requestNavigateTo(x_m, y_m, 0.0);
    qDebug() << "NavigateTo:" << x_m << y_m;
}

bool MainWindow::pixelToMap(int px, int py, double& x_m, double& y_m) const
{
    if (mapImageSize_.isEmpty()) return false;

    const int W = mapImageSize_.width();
    const int H = mapImageSize_.height();

    if (px < 0 || py < 0 || px >= W || py >= H) return false;

    const double res = mapMeta_.resolution;
    const double ox  = mapMeta_.origin_x;
    const double oy  = mapMeta_.origin_y;
    const double th  = mapMeta_.origin_yaw;

    const int gx = px;
    const int gy = (H - 1) - py;

    const double mx = (static_cast<double>(gx) + 0.5) * res;
    const double my = (static_cast<double>(gy) + 0.5) * res;

    const double c = std::cos(th);
    const double s = std::sin(th);

    const double dx =  c * mx - s * my;
    const double dy =  s * mx + c * my;

    x_m = ox + dx;
    y_m = oy + dy;
    return true;
}


void MainWindow::onLayerHeatmap(bool on)
{
    // // 기존 실시간 heatmap (절대 수정 금지)
    // if (heatItem_)
    //     heatItem_->setVisible(on);
    applyLayersPolicy();
}

void MainWindow::onLayerRobot(bool on)
{
    // poseReady_ == false면 아직 안 보이게 유지
    if (robotItem)
        robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);
}

void MainWindow::onLayerPins(bool on)
{
    for (auto* pin : apPins_) {
        if (pin) pin->setVisible(on);
    }
}


void MainWindow::onAutoExploreToggled(bool on)
{
    // 지도 bounds: 최소 MVP는 하드코딩(환경에 맞게 나중에 조정)
    // map_simul_world 기준 대략 범위를 모르면 일단 작은 범위로 시작해서 튜닝
    autoExplorer_.setMapBounds(-3.0, 3.0, -3.0, 3.0);

    autoExplorer_.setEnabled(on);
    statusBar()->showMessage(QString("Auto Explore %1").arg(on ? "ON" : "OFF"));
}

void MainWindow::onGoalStatusMsg(const QString& msg)
{
    statusBar()->showMessage(msg);

    // 최소 파싱 규칙 (RosWorker가 emit하는 문자열 기준)
    if (msg.contains("Sending goal") || msg.contains("Goal accepted")) {
        navBusy_ = true;
    }
    if (msg.contains("Goal SUCCEEDED") || msg.contains("Goal ABORTED") || msg.contains("Goal CANCELED")) {
        navBusy_ = false;
    }
}

void MainWindow::onAutoGoal(double x, double y, double yaw)
{
    // nav2가 목표 수행 중이면 자동 goal 스킵
    if (navBusy_) return;

    // 자동탐색이 꺼져있으면 스킵 (안전장치)
    if (!ui->chkAutoExplore->isChecked()) return;

    rosThread->requestNavigateTo(x, y, yaw);
    navBusy_ = true; // 보수적으로 즉시 Busy 처리
}

void MainWindow::onSessionRefresh()
{
    ui->cbSessionId->clear();

    const auto list = db_.listSessions();
    for (const auto& p : list) {
        const QString id = p.first;
        const QString at = p.second;
        ui->cbSessionId->addItem(id + " (" + at + ")", id);
    }

    statusBar()->showMessage(QString("Sessions: %1").arg(ui->cbSessionId->count()));
}

void MainWindow::onSessionLoad()
{
    const QString sid = ui->cbSessionId->currentData().toString();
    if (sid.isEmpty()) {
        statusBar()->showMessage("Load failed: no session selected");
        return;
    }
    currentLoadedSession_ = sid;
    reloadQueryLayer(sid);
    statusBar()->showMessage("Loaded session: " + sid);
    applyLayersPolicy();
}

void MainWindow::onSessionDelete()
{
    const QString sid = ui->cbSessionId->currentData().toString();
    if (sid.isEmpty()) return;

    db_.deleteSession(sid);

    // query layer clear
    queryLayer_.clear();
    queryLayer_.flush();
    queryLayer_.setVisible(false);

    if (currentLoadedSession_ == sid) currentLoadedSession_.clear();

    onSessionRefresh();
    statusBar()->showMessage("Deleted session: " + sid);
}

void MainWindow::onMeasureStart()
{
    // 새 세션 발급
    const QString sid = db_.beginSession();
    if (sid.isEmpty()) {
        statusBar()->showMessage("Measure Start failed: cannot create session");
        return;
    }

    activeSessionId_ = sid;
    measuringDb_ = true;

    statusBar()->showMessage("Measure Start: session=" + sid);

    // UI 세션 목록 갱신(바로 콤보에 뜨게)
    onSessionRefresh();

    // 방금 만든 세션을 콤보에서 선택해주기(선택 기능)
    for (int i=0; i<ui->cbSessionId->count(); ++i) {
        if (ui->cbSessionId->itemData(i).toString() == sid) {
            ui->cbSessionId->setCurrentIndex(i);
            break;
        }
    }
}

void MainWindow::onMeasureStop()
{
    measuringDb_ = false;
    db_.endSession(activeSessionId_);
    statusBar()->showMessage("Measure Stop: session=" + activeSessionId_);
}
void MainWindow::onApplyFilter()
{
    filterSsid_ = ui->cbSsid->currentText();
    filterThrEnable_ = ui->chkThrEnable->isChecked();
    filterThrRssi_ = ui->cbThr->currentText().toInt();

    statusBar()->showMessage(
        QString("Filter Applied: SSID=%1, Thr=%2%3")
            .arg(filterSsid_)
            .arg(filterThrEnable_ ? "ON " : "OFF ")
            .arg(filterThrEnable_ ? QString::number(filterThrRssi_) : "")
        );

    if (!currentLoadedSession_.isEmpty()) {
        reloadQueryLayer(currentLoadedSession_);
    }
    applyLayersPolicy();
}

void MainWindow::onSampleToDb(double x, double y, double /*yaw*/, float rssi)
{
    if (!measuringDb_) return;
    if (activeSessionId_.isEmpty()) return;
    if (filterThrEnable_ && rssi < filterThrRssi_) return;

    SampleRow s;
    s.x = x;
    s.y = y;
    s.ssid = filterSsid_.isEmpty() ? "ALL" : filterSsid_;
    s.rssi = rssi;
    s.ts = QDateTime::currentDateTime().toString(Qt::ISODate);

    db_.insertSample(activeSessionId_, s);
}
void MainWindow::reloadQueryLayer(const QString& sessionId)
{
    if (!mapReady_) return;
    if (!queryLayer_.isReady()) queryLayer_.init(scene, mapImageSize_, 6);

    const auto rows = db_.loadSamples(sessionId, filterSsid_, filterThrEnable_, filterThrRssi_);

    queryLayer_.clear();
    for (const auto& r : rows) {
        int px=0, py=0;
        if (!meterToPixel(r.x, r.y, px, py)) continue;

        const float intensity = valueToIntensity01(r);
        queryLayer_.addPoint(px + 1, py + 1, intensity);

    }
    queryLayer_.flush();
    queryLayer_.setVisible(true);
}
float MainWindow::simRssiAt(double d_m) const
{
    const double tx = ui->spTxPower->value(); // dBm (예: -40)
    // 1m 기준이라고 가정하고, d=0일 때도 폭주하지 않게 +1
    const double rssi = tx - 20.0 * std::log10(d_m + 1.0);
    return static_cast<float>(rssi);
}

float MainWindow::simIntensityAt(double d_m) const
{
    // rssiToIntensity01 재사용 (이미 clamp [-90,-30])
    return rssiToIntensity01(simRssiAt(d_m));
}

void MainWindow::addSimPinAt(double x_m, double y_m)
{
    // 핀 저장
    simPins_.push_back({x_m, y_m});

    // 핀 아이템 표시(지도 위)
    int px=0, py=0;
    if (meterToPixel(x_m, y_m, px, py)) {
        auto* pin = scene->addEllipse(-4, -4, 8, 8, QPen(Qt::blue), QBrush(Qt::blue));
        pin->setPos(QPointF(px, py));
        pin->setZValue(20);
        pin->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
        pin->setVisible(ui->chkShowApPins->isChecked());
        apPins_.push_back(pin);
    }

    // Sim 레이어 재계산
    rebuildSimLayer();

    // Sim layer 표시
    simLayer_.setVisible(ui->chkShowHeatmap->isChecked());
    statusBar()->showMessage(QString("Sim pin added: (%1, %2)  pins=%3")
                                 .arg(x_m,0,'f',2).arg(y_m,0,'f',2).arg(simPins_.size()));
    applyLayersPolicy();

}
void MainWindow::rebuildSimLayer()
{
    if (!mapReady_) return;
    if (!simLayer_.isReady()) simLayer_.init(scene, mapImageSize_, 7);

    simLayer_.clear();

    // 샘플링 파라미터(가벼운 MVP)
    const double maxR_m = 6.0;      // 6m까지 영향
    const int rings = 10;           // 동심원 개수
    const int angles = 24;          // 각도 샘플 개수

    for (const auto& pin : simPins_) {
        // 중심점도 한 번 찍고
        int cpx=0, cpy=0;
        if (meterToPixel(pin.x_m, pin.y_m, cpx, cpy)) {
            simLayer_.addPoint(cpx+1, cpy+1, simIntensityAt(0.0));
        }

        // 동심원 샘플
        for (int ri=1; ri<=rings; ++ri) {
            const double r = maxR_m * (static_cast<double>(ri) / rings);
            const float inten = simValueToIntensity01(r);

            for (int ai=0; ai<angles; ++ai) {
                const double a = (2.0 * M_PI) * (static_cast<double>(ai) / angles);
                const double x = pin.x_m + r * std::cos(a);
                const double y = pin.y_m + r * std::sin(a);

                int px=0, py=0;
                if (!meterToPixel(x, y, px, py)) continue;
                simLayer_.addPoint(px+1, py+1, inten);
            }
        }
    }

    simLayer_.flush();
}

void MainWindow::onSimEnable(bool on)
{
    simEnabled_ = on;

    // 켜면 기존 핀/히트맵이 있으면 보이게
    if (!simPins_.isEmpty()) {
        simLayer_.setVisible(on && ui->chkShowHeatmap->isChecked());
    } else {
        simLayer_.setVisible(false);
    }

    statusBar()->showMessage(QString("Sim Click %1").arg(on ? "ON" : "OFF"));
}

void MainWindow::onSimParamsChanged()
{
    // UI 값 -> 멤버 상태로 저장
    simTxPower_ = ui->spTxPower->value();
    simChannel_ = ui->spSimChannel->value();
    simBandwidth_ = ui->cbSimBandwidth->currentText().toInt();

    // 핀이 이미 있으면 즉시 재계산(원하면 이 줄 제거 가능)
    if (!simPins_.isEmpty()) {
        rebuildSimLayer();
        simLayer_.setVisible(simEnabled_ && ui->chkShowHeatmap->isChecked());
    }
}

void MainWindow::onClearPins()
{
    // 핀 아이템 삭제
    for (auto* it : apPins_) {
        if (!it) continue;
        scene->removeItem(it);
        delete it;
    }
    apPins_.clear();

    // 핀 데이터 삭제
    simPins_.clear();

    // sim layer 제거
    simLayer_.clear();
    simLayer_.flush();
    simLayer_.setVisible(false);

    statusBar()->showMessage("Sim pins cleared");
    applyLayersPolicy();

}
void MainWindow::onModeChanged()
{
    if (ui->rbModeView->isChecked())         currentMode_ = Mode::View;
    else if (ui->rbModeMeasure->isChecked()) currentMode_ = Mode::Measurement;
    else if (ui->rbModeQuery->isChecked())   currentMode_ = Mode::Query;
    else if (ui->rbModeSim->isChecked())     currentMode_ = Mode::SimAP;

    applyModeUi();
}

void MainWindow::applyModeUi()
{

    ui->gbMeasure->setEnabled(currentMode_ == Mode::Measurement);
    ui->gbSession->setEnabled(currentMode_ == Mode::Query);
    ui->gbFilter->setEnabled(currentMode_ == Mode::Query);   // Query에서 주로 사용
    ui->gbSim->setEnabled(currentMode_ == Mode::SimAP);

    ui->gbMetric->setEnabled(currentMode_ == Mode::Query || currentMode_ == Mode::SimAP);

    if (currentMode_ != Mode::SimAP) {
        // 체크를 끄고 싶지 않으면 setEnabled(false)만 해도 됨
        ui->chkSimEnable->setChecked(false);
        simEnabled_ = false;
        simLayer_.setVisible(false); // sim 모드 아닐 때는 기본 숨김(겹침 방지)
    }

    const bool showHeat = ui->chkShowHeatmap->isChecked();

    if (currentMode_ == Mode::View || currentMode_ == Mode::Measurement) {
        if (heatItem_) heatItem_->setVisible(true);
        if (queryLayer_.isReady()) queryLayer_.setVisible(false);
        if (simLayer_.isReady())   simLayer_.setVisible(false);
    }
    else if (currentMode_ == Mode::Query) {
        if (heatItem_) heatItem_->setVisible(false);
        if (queryLayer_.isReady()) queryLayer_.setVisible(showHeat && !currentLoadedSession_.isEmpty());
        if (simLayer_.isReady())   simLayer_.setVisible(false);
        if (currentLoadedSession_.isEmpty()) {
            statusBar()->showMessage("Query mode: select a session and press Load");
        }
    }
    else if (currentMode_ == Mode::SimAP) {
        if (heatItem_) heatItem_->setVisible(false);
        if (queryLayer_.isReady()) queryLayer_.setVisible(false);
        if (simLayer_.isReady())   simLayer_.setVisible(showHeat && simEnabled_ && !simPins_.isEmpty());
        statusBar()->showMessage("Sim mode: Shift+Click or Right Click to drop AP pins");
    }

    if (robotItem) robotItem->setVisible(ui->chkShowRobot->isChecked() && poseReady_);
    applyLayersPolicy();
}

void MainWindow::onMetricChanged(int)
{
    const QString m = ui->cbMetric->currentText();

    if (m == "RSSI") currentMetric_ = Metric::RSSI;
    else if (m == "SNR") currentMetric_ = Metric::SNR;
    else if (m == "AP Count") currentMetric_ = Metric::ApCount;
    else if (m == "Interference/Noise") currentMetric_ = Metric::Noise;
    else if (m == "Throughput(Max)") currentMetric_ = Metric::ThroughputMax;
    else currentMetric_ = Metric::RSSI;

    // 현재 지원 상태 안내
    if (currentMetric_ != Metric::RSSI) {
        statusBar()->showMessage("Metric not implemented yet (only RSSI available).");
    } else {
        statusBar()->showMessage("Metric: RSSI");

    if (currentMode_ == Mode::Query && !currentLoadedSession_.isEmpty()) {
        reloadQueryLayer(currentLoadedSession_);
    }
    if (currentMode_ == Mode::SimAP && simEnabled_ && !simPins_.isEmpty()) {
        rebuildSimLayer();
    }
    }
}

float MainWindow::valueToIntensity01(const SampleRow& s) const
{
    switch (currentMetric_) {
    case Metric::RSSI:
        return rssiToIntensity01(s.rssi);

    // 아직 데이터가 없으니 RSSI로 fallback (앱 안정 우선)
    case Metric::SNR:
    case Metric::ApCount:
    case Metric::Noise:
    case Metric::ThroughputMax:
    default:
        return rssiToIntensity01(s.rssi);
    }
}

float MainWindow::simValueToIntensity01(double d_m) const
{
    const float rssi = simRssiAt(d_m);

    switch (currentMetric_) {
    case Metric::RSSI:
        return rssiToIntensity01(rssi);

    default:
        return rssiToIntensity01(rssi);
    }
}

//히트맵 겹침 방지
void MainWindow::applyLayersPolicy()
{
    const bool showHeat = ui->chkShowHeatmap->isChecked();

    // 일단 모두 끄고 시작 (겹침 방지 핵심)
    if (heatItem_) heatItem_->setVisible(false);
    if (queryLayer_.isReady()) queryLayer_.setVisible(false);
    if (simLayer_.isReady())   simLayer_.setVisible(false);

    if (!showHeat) {
        // heatmap 전체 OFF
        return;
    }

    // 모드별 우선순위
    switch (currentMode_) {
    case Mode::View:
    case Mode::Measurement:
        // 실시간 누적 히트맵만
        if (heatItem_) heatItem_->setVisible(true);
        break;

    case Mode::Query:
        if (!currentLoadedSession_.isEmpty() && queryLayer_.isReady())
            queryLayer_.setVisible(true);
        else if (heatItem_) // 로드 안됐으면 fallback(선택)
            heatItem_->setVisible(true);
        break;

    case Mode::SimAP:
        if (simEnabled_ && !simPins_.isEmpty() && simLayer_.isReady())
            simLayer_.setVisible(true);
        else if (heatItem_) // sim 조건이 안되면 fallback(선택)
            heatItem_->setVisible(true);
        break;
    }
}


void MainWindow::on_btnSessionLoad_clicked()
{

}

