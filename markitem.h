#ifndef MARKITEM_H
#define MARKITEM_H
#include <QGraphicsItem>

class MarkItem : public QGraphicsItem
{
public:
    MarkItem(qreal x_ = 0, qreal y_ = 0, qreal r_ = 20, qreal w_ = 5, QColor c = QColor(219, 226, 234));
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void setSavedFlag(bool f);
    bool getSavedFlag();
private:
    qreal x, y;
    qreal r;
    qreal w;
    QColor color;
    bool ifSaved;
};

#endif // MARKITEM_H
