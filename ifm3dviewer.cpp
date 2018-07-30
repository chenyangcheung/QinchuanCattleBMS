#include "ifm3dviewer.h"

IFM3DViewer::IFM3DViewer(QObject *parent)
    : QThread(parent)
{
//    vtkDisplay = vd;
}

void IFM3DViewer::initViewer(QVTKWidget *&vd)
{
    vtkDisplay = vd;
}

void IFM3DViewer::openCamera()
{

}

void IFM3DViewer::openLocal()
{

}

void IFM3DViewer::run()
{

}
