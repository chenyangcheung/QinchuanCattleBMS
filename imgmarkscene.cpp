#include "imgmarkscene.h"
#include <QGraphicsSceneMouseEvent>
#include "markitem.h"
#include <QDebug>

ImgMarkScene::ImgMarkScene(QObject *parent)
    : QGraphicsScene(parent),
      prevItem(0)
{
    ifSelectedPoint = false;
    allItemSaved = false;
}

void ImgMarkScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    if ((event->button() == Qt::LeftButton))
    {
        qreal x = event->scenePos().x();  qreal y = event->scenePos().y();

        if ((prevItem != Q_NULLPTR))
        {
            removeItem(prevItem);
        }

        if (!allItemSaved)
        {
            addMark2Img(x, y);
            emit pointInfo(x, y);
        }
        qDebug() << allItemSaved;
    }
}

void ImgMarkScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{

}

void ImgMarkScene::addMark2Img(qreal x, qreal y)
{
    QColor cattleColorR(219, 226, 234);
    int pW = 5; int R = 20;

    MarkItem *mi = new MarkItem(x, y, R, pW, curItemID, cattleColorR);
    addItem(mi);
    prevItem = mi;
//    update();
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
//    delete prevItem;
    prevItem = Q_NULLPTR;
}

bool ImgMarkScene::getSelectedFlag()
{
    return ifSelectedPoint;
}

bool ImgMarkScene::ifAllSaved()
{
    return allItemSaved;
}

void ImgMarkScene::resetAllSaved()
{
    allItemSaved = false;
}

void ImgMarkScene::setAllSaved()
{
    allItemSaved = true;
}

ImgMarkScene::~ImgMarkScene()
{

}
