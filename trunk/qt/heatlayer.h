#pragma once
#include <QImage>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>

#include "heatmapper.h"
#include "gradientpalette.h"

class HeatLayer {
public:
    ~HeatLayer();

    // radius_px, opacity를 외부에서 받도록 확장
    bool init(QGraphicsScene* scene, const QSize& size, int z,
              int radius_px, int opacity);

    void clear();
    void addPoint(int px, int py, float intensity01);
    void flush();
    void setVisible(bool v);
    bool isReady() const { return mapper_ && item_; }

private:
    QImage canvas_;
    GradientPalette* palette_ = nullptr;
    HeatMapper* mapper_ = nullptr;
    QGraphicsPixmapItem* item_ = nullptr;
};
