#pragma once
#include <QVector>

class QImage;
class QColor;
class GradientPalette;

class HeatMapper
{
public:
    HeatMapper(QImage *image, GradientPalette *palette,
               int radius, int opacity,
               bool absoluteMode = false,
               bool cap01 = false,
               bool useSaturCurve = false,
               double saturK = 1.0);


    ~HeatMapper();

    void addPoint(int x, int y);
    void addPoint(int x, int y, qreal value);

    void redraw();
    void colorize();
    void colorize(int x, int y);
    void colorize(int left, int top, int right, int bottom);

    void setPalette(GradientPalette *palette);
    qreal getCount(int x, int y);

private:
    qreal increase(int x, int y, qreal delta);
    void drawAlpha(int x, int y, qreal count, bool colorize_now = true);

private:
    int radius_ = 18;
    int opacity_ = 160;

    qreal max_ = 1.0;

    bool absoluteMode_ = false;
    bool cap01_ = false;

    bool useSaturCurve_ = false;
    qreal saturK_ = 1.0;

    int width_ = 0;
    int height_ = 0;

    QVector<qreal> data_;

    GradientPalette *palette_ = nullptr;
    QImage *mainCanvas_ = nullptr;
    QImage *alphaCanvas_ = nullptr;
};
