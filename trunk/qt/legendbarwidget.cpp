#include "legendbarwidget.h"
#include <QPainter>
#include <QFontMetrics>
#include <QPainterPath>
#include <QtMath>

LegendBarWidget::LegendBarWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumHeight(44);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
}

void LegendBarWidget::setRangeDbm(double minDbm, double maxDbm)
{
    if (maxDbm <= minDbm) return;
    minDbm_ = minDbm;
    maxDbm_ = maxDbm;
    update();
}

void LegendBarWidget::setValueDbm(double v)
{
    valueDbm_ = v;
    update();
}

void LegendBarWidget::setTitle(const QString& t)
{
    title_ = t;
    update();
}

double LegendBarWidget::clamp(double v, double a, double b) const
{
    if (v < a) return a;
    if (v > b) return b;
    return v;
}

void LegendBarWidget::paintEvent(QPaintEvent*)
{
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    const QRectF r = rect().adjusted(6, 6, -6, -6);

    // 레이아웃(이미지처럼)
    const qreal titleH = 14;
    const qreal labelH = 14;
    const qreal barH   = r.height() - titleH - 4;  // 제목 아래 바
    const QRectF titleRect(r.left(), r.top(), r.width(), titleH);
    const QRectF barRect(r.left(), r.top() + titleH + 2, r.width(), barH);

    // 배경(회색 라운드)
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(220,220,220));
    p.drawRoundedRect(r, 6, 6);

    // 제목(우측 정렬 느낌)
    p.setPen(QColor(0,0,0,160));
    QFont f = p.font();
    f.setPointSizeF(9.0);
    p.setFont(f);
    p.drawText(titleRect, Qt::AlignRight | Qt::AlignVCenter, title_);

    // ===== 그라데이션 바 (빨강-노랑-초록) =====
    QRectF innerBar = barRect.adjusted(8, 6, -8, -6); // 바 패딩
    const qreal radius = 6;

    // 바 배경(흰색)
    p.setPen(QPen(QColor(0,0,0,70), 1));
    p.setBrush(Qt::white);
    p.drawRoundedRect(innerBar, radius, radius);

    // 그라데이션 채우기 영역(테두리 안쪽)
    QRectF fillRect = innerBar.adjusted(1.5, 1.5, -1.5, -1.5);

    QLinearGradient g(fillRect.left(), fillRect.center().y(),
                      fillRect.right(), fillRect.center().y());
    // 이미지 느낌: 빨강(나쁨) -> 노랑 -> 초록(좋음)
    g.setColorAt(0.00, QColor(255, 110, 90));
    g.setColorAt(0.50, QColor(255, 255, 120));
    g.setColorAt(1.00, QColor(120, 255, 120));

    p.setPen(Qt::NoPen);
    p.setBrush(g);
    p.drawRoundedRect(fillRect, radius-1, radius-1);

    // ===== 좌/우 라벨 (-80, >=0) =====
    p.setPen(QColor(0,0,0,200));
    QFont lf = p.font();
    lf.setPointSizeF(9.0);
    p.setFont(lf);

    // 작은 라벨 박스
    auto drawSmallTag = [&](const QString& text, QPointF anchor, Qt::Alignment align){
        QFontMetrics fm(p.font());
        const int padX = 6, padY = 2;
        QSize sz = fm.size(Qt::TextSingleLine, text) + QSize(padX*2, padY*2);
        QRectF box(0,0, sz.width(), sz.height());

        if (align & Qt::AlignLeft)  box.moveLeft(anchor.x());
        if (align & Qt::AlignRight) box.moveRight(anchor.x());
        if (align & Qt::AlignHCenter) box.moveCenter(QPointF(anchor.x(), box.center().y()));

        box.moveTop(anchor.y());

        p.setPen(QPen(QColor(0,0,0,120), 1));
        p.setBrush(Qt::white);
        p.drawRoundedRect(box, 3, 3);

        p.setPen(QColor(0,0,0,220));
        p.drawText(box, Qt::AlignCenter, text);
    };

    drawSmallTag(QString::number(int(minDbm_)), QPointF(fillRect.left(), fillRect.bottom()-labelH+2), Qt::AlignLeft);
    drawSmallTag(QString(">= %1").arg(int(maxDbm_)), QPointF(fillRect.right(), fillRect.bottom()-labelH+2), Qt::AlignRight);

    // ===== 현재값 마커(-67) =====
    const double v = clamp(valueDbm_, minDbm_, maxDbm_);
    const double t = (v - minDbm_) / (maxDbm_ - minDbm_); // 0..1
    const qreal x = fillRect.left() + qreal(t) * fillRect.width();

    // 말풍선(위)
    const QString vText = QString::number(int(std::round(valueDbm_)));
    QFontMetrics vm(p.font());
    const int padX = 6, padY = 2;
    QSize tagSz = vm.size(Qt::TextSingleLine, vText) + QSize(padX*2, padY*2);
    QRectF tagRect(0,0, tagSz.width(), tagSz.height());

    // 바 위쪽에 위치(이미지처럼)
    qreal tagY = fillRect.top() - tagRect.height() - 6;
    tagRect.moveCenter(QPointF(x, tagY + tagRect.height()/2.0));

    // 화면 밖으로 튀지 않게 클램프
    if (tagRect.left() < r.left()+2) tagRect.moveLeft(r.left()+2);
    if (tagRect.right() > r.right()-2) tagRect.moveRight(r.right()-2);

    // 말풍선 박스
    p.setPen(QPen(QColor(0,0,0,120), 1));
    p.setBrush(QColor(245,245,245));
    p.drawRoundedRect(tagRect, 3, 3);

    // 텍스트
    p.setPen(QColor(0,0,0,220));
    p.drawText(tagRect, Qt::AlignCenter, vText);

    // 삼각 포인터(말풍선->바)
    QPointF tip(x, fillRect.top() - 1);
    QPointF a(tagRect.center().x() - 5, tagRect.bottom());
    QPointF b(tagRect.center().x() + 5, tagRect.bottom());

    QPainterPath tri;
    tri.moveTo(a);
    tri.lineTo(b);
    tri.lineTo(tip);
    tri.closeSubpath();

    p.setPen(QPen(QColor(0,0,0,120), 1));
    p.setBrush(QColor(245,245,245));
    p.drawPath(tri);

    // 가는 세로 기준선(옵션: 현재 위치 강조)
    p.setPen(QPen(QColor(0,0,0,80), 1));
    p.drawLine(QPointF(x, fillRect.top()), QPointF(x, fillRect.bottom()));
}
