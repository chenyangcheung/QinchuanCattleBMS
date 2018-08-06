#include "imgpoint.h"

ImgPoint::ImgPoint(QPoint p, bool ifs, int i)
{
    pos = p;
    ifSaved = ifs;
    pID = i;
    markItemPtr = Q_NULLPTR;
}
