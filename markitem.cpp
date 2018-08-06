#include "markitem.h"
#include <QPainter>
#include <QStyleOptionGraphicsItem>

MarkItem::MarkItem(qreal x_, qreal y_, qreal r_, qreal w_, int i, QColor c)
{
    x = x_;
    y = y_;
    r = r_;
    color = c;
    w = w_;
    id = i;
    ifSaved = false;
}

QRectF MarkItem::boundingRect() const
{
    return QRectF(x - r, y - r, r * 2, r * 2);
}

void MarkItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setPen(QPen(color, w));
    painter->drawEllipse(x - r, y - r, r * 2, r * 2);
    painter->setPen(QPen(color, w + 3));
    painter->drawLine(x, y, x, y);
//    painter->drawPoints(QPointF(x, y), 1);
//    painter->drawText(QPoint(x - r * 2, y), "P" + QString::number(id, 10));
}

bool MarkItem::getSavedFlag()
{
    return ifSaved;
}

void MarkItem::setSavedFlag(bool f)
{
    ifSaved = f;
}
