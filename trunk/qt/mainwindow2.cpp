#include "mainwindow2.h"
#include "ui_mainwindow2.h"

#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QPixmap>
#include <QDebug>
#include <QMouseEvent>
#include <cmath>
#include <QTimer>
#include <QElapsedTimer>

static inline double rad2deg(double r) { return r * 180.0 / M_PI; }

MainWindow2::MainWindow2(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow2)
{
    ui->setupUi(this);

    // ===== Graphics =====
    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->viewport()->installEventFilter(this);

    mapItem = scene->addPixmap(QPixmap());
    mapItem->setZValue(0);

    robotItem = scene->addEllipse(-5, -5, 10, 10, QPen(Qt::red), QBrush(Qt::red));
    robotItem->setZValue(10);
    robotItem->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
    robotItem->setVisible(false);

    // ===== Map =====
    const QString yamlPath = "/home/ubuntu/ros2/maps/map_simul_world.yaml";
    if (!loadStaticMap(yamlPath)) {
        qDebug() << "Failed to load map yaml:" << yamlPath;
    } else {
        mapReady_ = true;
    }

    // ===== Layers =====
    if (mapReady_) {
        queryLayer_.init(scene, mapImageSize_, 6);
        queryLayer_.setVisible(false);

        simLayer_.init(scene, mapImageSize_, 7);
        simLayer_.setVisible(false);
    }

    initHeatmapLayer();

    // ===== DB =====
    db_.open(QDir::homePath() + "/wifi_qt.db");
    db_.initSchema();

    // ===== ROS =====
    rosThread = new RosWorker(this);
    connect(rosThread, &RosWorker::statusChanged, this, &MainWindow2::onRosStatus);
    connect(rosThread, &RosWorker::robotPose,     this, &MainWindow2::onRobotPose);
    connect(rosThread, &RosWorker::sample,        this, &MainWindow2::onSample);
    connect(rosThread, &RosWorker::sample,        this, &MainWindow2::onSampleToDb);
    rosThread->start();

    // ===== UI(겉) 연결 =====
    connect(ui->pBt_RunMode,   &QPushButton::clicked, this, &MainWindow2::on_pBt_RunMode_clicked);
    connect(ui->pBt_ViewMode,  &QPushButton::clicked, this, &MainWindow2::on_pBt_ViewMode_clicked);
    connect(ui->pBt_SimulMode, &QPushButton::clicked, this, &MainWindow2::on_pBt_SimulMode_clicked);

    //connect(ui->pBt_Start, &QPushButton::clicked, this, &MainWindow2::on_pBt_Start_clicked);
    connect(ui->pBt_Start, &QPushButton::toggled,
            this, &MainWindow2::on_pBt_Start_clicked);

    connect(ui->pBt_DB_Refresh, &QPushButton::clicked, this, &MainWindow2::on_pBt_DB_Refresh_clicked);
    connect(ui->pBt_DB_Load,    &QPushButton::clicked, this, &MainWindow2::on_pBt_DB_Load_clicked);
    connect(ui->pBt_DB_Delete,  &QPushButton::clicked, this, &MainWindow2::on_pBt_DB_Delete_clicked);

    connect(ui->pComboBox_SSID, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow2::onSsidChanged);
    connect(ui->pBt_AP_Clear, &QPushButton::clicked, this, &MainWindow2::onClearPins);
    connect(ui->pCB_AP_Able, &QCheckBox::toggled, this, &MainWindow2::onSimEnable);

    // ===== 초기 데이터 =====
    onSessionRefresh();

    // 기본은 Run(Measurement)
    on_pBt_RunMode_clicked();

    // 12_29
    setupAutoHideUi();
    markActivity();
}

MainWindow2::~MainWindow2()
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

// ============================ UI 모드 정책 ============================

void MainWindow2::on_pBt_RunMode_clicked()
{
    ui->pStackedWidget->setCurrentIndex(0);
    currentMode_ = Mode::Measurement;

    simEnabled_ = false;
    showHeatmap_ = true;
    showRobot_   = true;
    showPins_    = false;

    applyLayersPolicy();
}

void MainWindow2::on_pBt_ViewMode_clicked()
{
    ui->pStackedWidget->setCurrentIndex(1);
    currentMode_ = Mode::Query; // 요구사항: ViewMode는 DB(Query)

    simEnabled_ = false;
    showHeatmap_ = true;
    showRobot_   = true;
    showPins_    = false;

    applyLayersPolicy();
}

void MainWindow2::on_pBt_SimulMode_clicked()
{
    ui->pStackedWidget->setCurrentIndex(2);
    currentMode_ = Mode::SimAP;

    // UI에 Sim enable/params 없으니 Sim 모드에서는 자동 ON + 고정 파라미터 사용
    simEnabled_ = true;
    showHeatmap_ = true;
    showRobot_   = true;
    showPins_    = true;

    // sim layer는 핀 없으면 표시 안 됨(정책에서 처리)
    applyLayersPolicy();
}

void MainWindow2::on_pBt_Start_clicked(bool checked)
{
    if (checked) {
        ui->pBt_Start->setText("Stop");
        onMeasureStart();
    } else {
        ui->pBt_Start->setText("Start");
        onMeasureStop();
    }
}

void MainWindow2::on_pBt_DB_Refresh_clicked()
{
    if (measuringDb_) return;
    onSessionRefresh();
}

void MainWindow2::on_pBt_DB_Load_clicked()
{
    onSessionLoad();
}

void MainWindow2::on_pBt_DB_Delete_clicked()
{
    onSessionDelete();
}

void MainWindow2::onSsidChanged(int)
{
    // SSID 변경 즉시 필터 적용(버튼 UI가 없으므로)
    onApplyFilter();
}

// ============================ ROS callbacks ============================

void MainWindow2::onRosStatus(const QString &msg)
{
    qDebug() << msg;
}

void MainWindow2::onRobotPose(double x, double y, double yaw)
{
    if (!mapReady_) return;

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    robotItem->setPos(QPointF(px, py));
    robotItem->setRotation(-rad2deg(yaw));

    if (!poseReady_) {
        poseReady_ = true;
        // 표시 정책에 따라
        robotItem->setVisible(showRobot_);
    }

    applyLayersPolicy();
}

void MainWindow2::onSample(double x, double y, double /*yaw*/, float rssi)
{
    // Measurement에서만 실시간 누적 히트맵을 “시각적으로 의미 있게” 사용
    // (원하면 모드 상관 없이 누적되게 해도 됨)
    if (!mapReady_ || !heatMapper_) return;
    if (currentMode_ != Mode::Measurement && currentMode_ != Mode::View && currentMode_ != Mode::Query && currentMode_ != Mode::SimAP) return;

    int px=0, py=0;
    if (!meterToPixel(x, y, px, py)) return;

    const float intensity = rssiToIntensity01(rssi);
    heatMapper_->addPoint(px + 1, py + 1, intensity);
}

void MainWindow2::onSampleToDb(double x, double y, double /*yaw*/, float rssi)
{
    // Start(측정) 안 했으면 저장 금지
    if (!measuringDb_) return;
    if (activeSessionId_.isEmpty()) return;

    // threshold UI 없음 → 기본 OFF 유지
    if (filterThrEnable_ && rssi < filterThrRssi_) return;

    SampleRow s;
    s.x = x;
    s.y = y;
    s.ssid = filterSsid_.isEmpty() ? "ALL" : filterSsid_;
    s.rssi = rssi;
    s.ts = QDateTime::currentDateTime().toString(Qt::ISODate);

    db_.insertSample(activeSessionId_, s);
}

// ============================ Heatmap ============================

void MainWindow2::initHeatmapLayer()
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
        connect(heatFlushTimer_, &QTimer::timeout, this, &MainWindow2::flushHeatmap);
        heatFlushTimer_->start(100);
    }
}

void MainWindow2::flushHeatmap()
{
    if (!heatMapper_ || !heatItem_) return;

    heatMapper_->colorize();
    heatItem_->setPixmap(QPixmap::fromImage(heatCanvas_));
    applyLayersPolicy();
}

float MainWindow2::rssiToIntensity01(float rssi) const
{
    // clamp [-90, -30]
    if (rssi < -90.f) rssi = -90.f;
    if (rssi > -30.f) rssi = -30.f;

    // -90 -> 0(bad), -30 -> 1(good)
    return (rssi + 90.f) / 60.f;
}

// ============================ DB (Measurement / Query) ============================

void MainWindow2::onMeasureStart()
{
    // 새 세션 생성
    const QString sid = db_.beginSession();
    if (sid.isEmpty()) {
        statusBar()->showMessage("Measure Start failed: cannot create session");
        ui->pBt_Start->setChecked(false);
        ui->pBt_Start->setText("Start");
        return;
    }

    activeSessionId_ = sid;
    measuringDb_ = true;

    statusBar()->showMessage("Measure Start: session=" + sid);

    ui->pComboBox_Session->blockSignals(true);
    ui->pComboBox_Session->clear();
    ui->pComboBox_Session->addItem(sid, sid);
    ui->pComboBox_Session->setCurrentIndex(0);
    ui->pComboBox_Session->blockSignals(false);
}

void MainWindow2::onMeasureStop()
{
    measuringDb_ = false;
    if (!activeSessionId_.isEmpty()) {
        //db_.endSession(activeSessionId_);
        statusBar()->showMessage("Measure Stop: session=" + activeSessionId_);
    } else {
        statusBar()->showMessage("Measure Stop");
    }
}

void MainWindow2::onSessionRefresh()
{
    ui->pComboBox_Session->clear();

    const auto list = db_.listSessions();
    for (const auto& p : list) {
        const QString id = p.first;
        const QString at = p.second;
        ui->pComboBox_Session->addItem(id + " (" + at + ")", id);
    }

    statusBar()->showMessage(QString("Sessions: %1").arg(ui->pComboBox_Session->count()));
}

void MainWindow2::onSessionLoad()
{
    const QString sid = ui->pComboBox_Session->currentData().toString();
    if (sid.isEmpty()) {
        statusBar()->showMessage("Load failed: no session selected");
        return;
    }

    currentLoadedSession_ = sid;
    reloadQueryLayer(sid);

    statusBar()->showMessage("Loaded session: " + sid);
    applyLayersPolicy();
}

void MainWindow2::onSessionDelete()
{
    const QString sid = ui->pComboBox_Session->currentData().toString();
    if (sid.isEmpty()) return;

    db_.deleteSession(sid);

    // query clear
    queryLayer_.clear();
    queryLayer_.flush();
    queryLayer_.setVisible(false);

    if (currentLoadedSession_ == sid) currentLoadedSession_.clear();

    onSessionRefresh();
    statusBar()->showMessage("Deleted session: " + sid);

    applyLayersPolicy();
}

void MainWindow2::onApplyFilter()
{
    // SSID 콤보가 비어있으면 ALL로
    QString ssid = ui->pComboBox_SSID->currentText().trimmed();
    if (ssid.isEmpty()) ssid = "ALL";
    filterSsid_ = ssid;

    // Threshold UI 없음 → OFF 고정
    filterThrEnable_ = false;

    statusBar()->showMessage(QString("Filter Applied: SSID=%1").arg(filterSsid_));

    if (!currentLoadedSession_.isEmpty()) {
        reloadQueryLayer(currentLoadedSession_);
    }
    applyLayersPolicy();
}

float MainWindow2::valueToIntensity01(const SampleRow& s) const
{
    // Metric UI 없음 → RSSI 고정
    return rssiToIntensity01(s.rssi);
}

void MainWindow2::reloadQueryLayer(const QString& sessionId)
{
    if (!mapReady_) return;
    if (!queryLayer_.isReady()) queryLayer_.init(scene, mapImageSize_, 6);

    const auto rows = db_.loadSamples(sessionId, filterSsid_, filterThrEnable_, filterThrRssi_);

    queryLayer_.clear();
    for (const auto& r : rows) {
        int px=0, py=0;
        if (!meterToPixel(r.x, r.y, px, py)) continue;
        queryLayer_.addPoint(px + 1, py + 1, valueToIntensity01(r));
    }
    queryLayer_.flush();
}

// ============================ Sim ============================

float MainWindow2::simRssiAt(double d_m) const
{
    // 1m 기준 간단 모델, d=0 발산 방지
    const double rssi = simTxPower_ - 20.0 * std::log10(d_m + 1.0);
    return static_cast<float>(rssi);
}

float MainWindow2::simValueToIntensity01(double d_m) const
{
    return rssiToIntensity01(simRssiAt(d_m));
}

void MainWindow2::addSimPinAt(double x_m, double y_m)
{
    simPins_.push_back({x_m, y_m});

    int px=0, py=0;
    if (meterToPixel(x_m, y_m, px, py)) {
        auto* pin = scene->addEllipse(-4, -4, 8, 8, QPen(Qt::blue), QBrush(Qt::blue));
        pin->setPos(QPointF(px, py));
        pin->setZValue(20);
        pin->setFlag(QGraphicsItem::ItemIgnoresTransformations, true);
        apPins_.push_back(pin);
    }

    rebuildSimLayer();
    statusBar()->showMessage(QString("Sim pin added: (%1, %2), pins=%3")
                                 .arg(x_m,0,'f',2).arg(y_m,0,'f',2).arg(simPins_.size()));
    applyLayersPolicy();
}

void MainWindow2::rebuildSimLayer()
{
    if (!mapReady_) return;
    if (!simLayer_.isReady()) simLayer_.init(scene, mapImageSize_, 7);

    simLayer_.clear();

    const double maxR_m = 6.0;
    const int rings = 10;
    const int angles = 24;

    for (const auto& pin : simPins_) {
        int cpx=0, cpy=0;
        if (meterToPixel(pin.x_m, pin.y_m, cpx, cpy)) {
            simLayer_.addPoint(cpx+1, cpy+1, simValueToIntensity01(0.0));
        }

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

void MainWindow2::onClearPins()
{
    for (auto* it : apPins_) {
        if (!it) continue;
        scene->removeItem(it);
        delete it;
    }
    apPins_.clear();
    simPins_.clear();

    simLayer_.clear();
    simLayer_.flush();
    simLayer_.setVisible(false);

    statusBar()->showMessage("Sim pins cleared");
    applyLayersPolicy();
}

// ============================ Layer Policy ============================

void MainWindow2::applyLayersPolicy()
{
    // robot
    if (robotItem) robotItem->setVisible(showRobot_ && poseReady_);

    // pins
    for (auto* pin : apPins_) {
        if (pin) pin->setVisible(showPins_);
    }

    // heat layers: 우선 모두 OFF
    if (heatItem_) heatItem_->setVisible(false);
    if (queryLayer_.isReady()) queryLayer_.setVisible(false);
    if (simLayer_.isReady())   simLayer_.setVisible(false);

    if (!showHeatmap_) return;

    switch (currentMode_) {
    case Mode::View:
    case Mode::Measurement:
        if (heatItem_) heatItem_->setVisible(true);
        break;

    case Mode::Query:
        if (!currentLoadedSession_.isEmpty() && queryLayer_.isReady())
            queryLayer_.setVisible(true);
        else if (heatItem_)
            heatItem_->setVisible(true);
        break;

    case Mode::SimAP:
        if (simEnabled_ && !simPins_.isEmpty() && simLayer_.isReady())
            simLayer_.setVisible(true);
        else if (heatItem_)
            heatItem_->setVisible(true);
        break;
    }
}

// ============================ Map / Transform / Click ============================

bool MainWindow2::loadStaticMap(const QString &yamlPath)
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

bool MainWindow2::parseMapYaml(const QString &yamlPath, MapMeta &out)
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

void MainWindow2::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    applyViewTransform();
}

void MainWindow2::showEvent(QShowEvent* e)
{
    QMainWindow::showEvent(e);
    QTimer::singleShot(0, this, [this]{
        applyViewTransform();
    });
}

void MainWindow2::applyViewTransform()
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

    // 기존과 동일하게 -90도 회전이 필요하면 유지
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

bool MainWindow2::meterToPixel(double x, double y, int& px, int& py) const
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

bool MainWindow2::pixelToMap(int px, int py, double& x_m, double& y_m) const
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

bool MainWindow2::sceneToMapPixel(const QPointF& scenePos, int& px, int& py) const
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

void MainWindow2::onMapClicked(const QPointF& scenePos)
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

    // nav2 goal
    if (rosThread) {
        rosThread->requestNavigateTo(x_m, y_m, 0.0);
        qDebug() << "NavigateTo:" << x_m << y_m;
    }
}


bool MainWindow2::eventFilter(QObject* obj, QEvent* ev)
{
    // 1) 입력 이벤트면 activity로 간주
    switch (ev->type()) {
    case QEvent::MouseButtonPress:
    case QEvent::MouseButtonRelease:
    case QEvent::MouseMove:
    case QEvent::Wheel:
    case QEvent::KeyPress:
    case QEvent::TouchBegin:
    case QEvent::TouchUpdate:
    case QEvent::TouchEnd:
        markActivity();
        // 숨김 상태였다면 깨우고 "이번 입력"은 소비
        if (wakeUiIfHidden()) return true;
        break;
    default:
        break;
    }

    // 2) graphicsView 클릭 처리 (기존 로직)
    if (obj == ui->graphicsView->viewport() &&
        ev->type() == QEvent::MouseButtonPress) {

        // 숨김에서 막 깨어난 직후 첫 클릭은 목적지 설정 금지
        if (swallowNextMapPress_) {
            swallowNextMapPress_ = false;
            return true; // consume
        }

        auto* me = static_cast<QMouseEvent*>(ev);
        const QPointF scenePos = ui->graphicsView->mapToScene(me->pos());

        // Sim 모드 핀 제스처
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

        // 일반 클릭: 목표 이동
        onMapClicked(scenePos);
        return true;
    }

    return QMainWindow::eventFilter(obj, ev);
}


void MainWindow2::onSimEnable(bool on)
{
    simEnabled_ = on;

    // // 켜면 기존 핀/히트맵이 있으면 보이게
    // if (!simPins_.isEmpty()) {
    //     simLayer_.setVisible(on && ui->chkShowHeatmap->isChecked());
    // } else {
    //     simLayer_.setVisible(false);
    // }

    simLayer_.setVisible(false);
    statusBar()->showMessage(QString("Sim Click %1").arg(on ? "ON" : "OFF"));
    //if (filterThrEnable_ && rssi < filterThrRssi_) return;
}

void MainWindow2::setupAutoHideUi()
{
    // 앱 전체 입력 감지 (graphicsView 밖 클릭도 포함)
    qApp->installEventFilter(this);

    idleClock_.start();

    idleTimer_ = new QTimer(this);
    idleTimer_->setInterval(200);
    connect(idleTimer_, &QTimer::timeout, this, [this]{
        if (!uiHidden_ && idleClock_.elapsed() >= 10000) {
            setChromeVisible(false);
            uiHidden_ = true;
        }
    });
    idleTimer_->start();
}

void MainWindow2::setChromeVisible(bool on)
{
    // verticalLayout_2 안의 "실제 위젯"을 숨김/표시 → graphicsView가 자동 확대됨
    ui->pBt_RunMode->setVisible(on);
    ui->pBt_ViewMode->setVisible(on);
    ui->pBt_SimulMode->setVisible(on);

    // 하단 패널 숨김/표시 → graphicsView가 세로로 자동 확대됨
    ui->pStackedWidget->setVisible(on);

    // 레이아웃 갱신 (즉시 반영)
    ui->centralwidget->updateGeometry();
    ui->centralwidget->layout()->activate();
}

void MainWindow2::markActivity()
{
    idleClock_.restart();
}

bool MainWindow2::wakeUiIfHidden()
{
    if (!uiHidden_) return false;

    setChromeVisible(true);
    uiHidden_ = false;

    // “깨우기용 입력”이 지도 클릭으로 이어지지 않게 1회 소비 플래그
    swallowNextMapPress_ = true;

    markActivity();
    return true;
}
