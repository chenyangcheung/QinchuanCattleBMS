#include "imgmarkscene.h"
#include <QGraphicsSceneMouseEvent>

ImgMarkScene::ImgMarkScene(QObject *parent)
    : QGraphicsScene(parent)
{
    ifSelectedPoint = false;
}

void ImgMarkScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if ((event->button() == Qt::LeftButton) && ifSelectedPoint)
    {
        qreal x = event->scenePos().x();  qreal y = event->scenePos().y();
        addMark2Img(x, y);
        emit pointInfo(x, y);
    }
}

void ImgMarkScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{

}

void ImgMarkScene::addMark2Img(qreal x, qreal y)
{
    QColor cattleColorR(219, 226, 234);
    int pW = 5; int R = 20;
    addLine(x, y, x, y, QPen(cattleColorR, pW));
    addEllipse(x - 20, y - 20, R * 2, R * 2, QPen(cattleColorR, pW));
}

void ImgMarkScene::setSelectedFlag(bool f)
{
    ifSelectedPoint = f;
}

ImgMarkScene::~ImgMarkScene()
{

}
