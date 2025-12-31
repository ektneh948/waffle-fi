#pragma once
#include <QImage>
#include <QGraphicsScene>
#include <QGraphicsPixmapItem>
#include "heatmapper.h"
#include "gradientpalette.h"

class HeatLayer {
public:
    ~HeatLayer();

    bool init(QGraphicsScene* scene, const QSize& size, int z, int radius_px, int opacity);

    // ✅ 캔버스만 비우기 (mapper 유지)
    void clearCanvas();

    // ✅ 점 데이터만 비우는 용도(사실 addPoint가 mapper에 직접 찍으니 clearCanvas만 써도 됨)
    void clear(bool clearCanvasToo);

    void addPoint(int px, int py, float intensity01);
    void flush();
    void setVisible(bool v);
    bool isReady() const { return mapper_ && item_; }

    // ✅ 브러시 변경(=HeatMapper 재생성)
    void resetBrush(int radius_px, int opacity);

private:
    QImage canvas_;
    GradientPalette* palette_ = nullptr;
    HeatMapper* mapper_ = nullptr;
    QGraphicsPixmapItem* item_ = nullptr;

    // ✅ 현재 브러시 파라미터 저장 (clear에서 재생성하지 않더라도 상태 추적용)
    int radius_px_ = 1;
    int opacity_ = 160;
};
