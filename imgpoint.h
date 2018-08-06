#ifndef IMGPOINT_H
#define IMGPOINT_H
#include <QRadioButton>
#include <QCheckBox>
#include "markitem.h"

class ImgPoint
{
public:
    QPoint pos;
    bool ifSaved;
    int pID;
    QRadioButton *radioBtnPtr;
    QCheckBox *checkBoxPtr;
    MarkItem *markItemPtr;
public:
    ImgPoint(QPoint p = QPoint(0, 0), bool ifs = false, int i = 0);
};

#endif // IMGPOINT_H
