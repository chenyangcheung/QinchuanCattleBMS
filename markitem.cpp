#include "markitem.h"
#include <QPainter>
#include <QStyleOptionGraphicsItem>

MarkItem::MarkItem(qreal x_, qreal y_, qreal r_, qreal w_, QColor c)
{
    x = x_;
    y = y_;
    r = r_;
    color = c;
    w = w_;
    ifSaved = false;
}

QRectF MarkItem::boundingRect() const
{
    return QRectF(x, y, r * 2, r * 2);
}

void MarkItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setPen(QPen(color, w));
    painter->drawEllipse(x - 20, y - 20, r * 2, r * 2);
    painter->setPen(QPen(color, w + 3));
    painter->drawLine(x, y, x, y);
}

bool MarkItem::getSavedFlag()
{
    return ifSaved;
}

void MarkItem::setSavedFlag(bool f)
{
    ifSaved = f;
}
