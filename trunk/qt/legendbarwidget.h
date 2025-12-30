#ifndef LEGENDBARWIDGET_H
#define LEGENDBARWIDGET_H

#pragma once
#include <QWidget>

class LegendBarWidget : public QWidget
{
    Q_OBJECT
public:
    explicit LegendBarWidget(QWidget* parent=nullptr);

    void setRangeDbm(double minDbm, double maxDbm); // 예: -80 ~ 0
    void setValueDbm(double v);                    // 예: -67
    void setTitle(const QString& t);               // "Voice + Data | dBm"

protected:
    void paintEvent(QPaintEvent*) override;

private:
    double minDbm_ = -80.0;
    double maxDbm_ = 0.0;
    double valueDbm_ = -67.0;
    QString title_ = "Voice + Data | dBm";

    double clamp(double v, double a, double b) const;
};


#endif // LEGENDBARWIDGET_H
