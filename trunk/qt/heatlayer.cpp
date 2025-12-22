#include "heatlayer.h"
#include <QPixmap>

HeatLayer::~HeatLayer() {
    delete mapper_;
    delete palette_;
}

bool HeatLayer::init(QGraphicsScene* scene, const QSize& size, int z) {
    if (!scene || size.isEmpty()) return false;

    canvas_ = QImage(size, QImage::Format_ARGB32);
    canvas_.fill(Qt::transparent);

    delete palette_;
    palette_ = new GradientPalette(256);
    palette_->setColorAt(0.0, QColor(255, 0, 0));
    palette_->setColorAt(0.5, QColor(255, 255, 0));
    palette_->setColorAt(1.0, QColor(0, 255, 0));

    delete mapper_;
    mapper_ = new HeatMapper(&canvas_, palette_, /*radius=*/18, /*opacity=*/160);

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

void HeatLayer::clear() {
    if (!canvas_.isNull()) canvas_.fill(Qt::transparent);
}

void HeatLayer::addPoint(int px, int py, float intensity01) {
    if (mapper_) mapper_->addPoint(px, py, intensity01);
}

void HeatLayer::flush() {
    if (!mapper_ || !item_) return;
    mapper_->colorize();
    item_->setPixmap(QPixmap::fromImage(canvas_));
}

void HeatLayer::setVisible(bool v) {
    if (item_) item_->setVisible(v);
}
