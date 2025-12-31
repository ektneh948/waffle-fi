#include "heatlayer.h"
#include <QPixmap>
#include <QColor>

HeatLayer::~HeatLayer() {
    delete mapper_;
    delete palette_;
}

bool HeatLayer::init(QGraphicsScene* scene, const QSize& size, int z, int radius_px, int opacity)
{
    if (!scene || size.isEmpty()) return false;

    radius_px_ = qMax(1, radius_px);
    opacity_   = qBound(1, opacity, 255);

    canvas_ = QImage(size, QImage::Format_ARGB32);
    canvas_.fill(Qt::transparent);

    delete palette_;
    palette_ = new GradientPalette(256);
    palette_->setColorAt(0.0, QColor(255, 0, 0));
    palette_->setColorAt(0.5, QColor(255, 255, 0));
    palette_->setColorAt(1.0, QColor(0, 255, 0));

    delete mapper_;
    mapper_ = new HeatMapper(&canvas_, palette_, radius_px_, opacity_, true, true);

    if (!item_) {
        item_ = scene->addPixmap(QPixmap::fromImage(canvas_));
        item_->setZValue(z);
        item_->setPos(0, 0);
    } else {
        item_->setPixmap(QPixmap::fromImage(canvas_));
        item_->setZValue(z);
    }
    return true;
}

void HeatLayer::clearCanvas()
{
    if (canvas_.isNull()) return;
    canvas_.fill(Qt::transparent);
    if (item_) item_->setPixmap(QPixmap::fromImage(canvas_));
    // ✅ mapper는 유지 (여기서 재생성하면 resetBrush가 무력화됨)
}

void HeatLayer::clear(bool clearCanvasToo)
{
    if (clearCanvasToo) clearCanvas();
}

void HeatLayer::resetBrush(int radius_px, int opacity)
{
    if (!mapper_ || canvas_.isNull()) return;
    delete mapper_;
    mapper_ = new HeatMapper(&canvas_, palette_, radius_px, opacity, true, true);
}
void HeatLayer::addPoint(int px, int py, float intensity01)
{
    if (mapper_) mapper_->addPoint(px, py, intensity01);
}

void HeatLayer::flush()
{
    if (!mapper_ || !item_) return;
    mapper_->colorize();
    item_->setPixmap(QPixmap::fromImage(canvas_));
}

void HeatLayer::setVisible(bool v)
{
    if (item_) item_->setVisible(v);
}
