#ifndef IMGMARKSCENE_H
#define IMGMARKSCENE_H
#include <QGraphicsScene>
#include "markitem.h"

class ImgMarkScene : public QGraphicsScene
{
    Q_OBJECT

public:
    explicit ImgMarkScene(QObject *parent = 0);
    void addMark2Img(qreal x, qreal y);
    void setSelectedFlag(bool f);
    bool getSelectedFlag();
    MarkItem* getPrevItem();
    void resetPrevItem();
    bool ifAllSaved();
    void resetAllSaved();
    void setAllSaved();
    int curItemID;
    ~ImgMarkScene();
protected:
    void mousePressEvent(QGraphicsSceneMouseEvent *event);
    void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
//    bool event(QEvent *event);
private:
    bool ifSelectedPoint;
//    QGraphicsItem *prevItem;
    MarkItem *prevItem;
    bool allItemSaved;
signals:
    void pointInfo(qreal x, qreal y);
};

#endif // IMGMARKSCENE_H
