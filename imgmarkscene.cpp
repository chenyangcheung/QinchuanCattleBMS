#include "imgmarkscene.h"
#include <QGraphicsSceneMouseEvent>
#include "markitem.h"
#include <QDebug>

ImgMarkScene::ImgMarkScene(QObject *parent)
    : QGraphicsScene(parent),
      prevItem(0)
{
    ifSelectedPoint = false;
}

void ImgMarkScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if ((event->button() == Qt::LeftButton) && ifSelectedPoint)
    {
        qreal x = event->scenePos().x();  qreal y = event->scenePos().y();
        if ((prevItem != Q_NULLPTR) && (!prevItem->getSavedFlag()))
            removeItem(prevItem);

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

    MarkItem *mi = new MarkItem(x, y, R, pW, cattleColorR);
    addItem(mi);
    prevItem = mi;
    update();
}

void ImgMarkScene::setSelectedFlag(bool f)
{
    ifSelectedPoint = f;
}

MarkItem* ImgMarkScene::getPrevItem()
{
    return prevItem;
}

void ImgMarkScene::resetPrevItem()
{
    prevItem = Q_NULLPTR;
}

ImgMarkScene::~ImgMarkScene()
{

}
