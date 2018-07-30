#ifndef IFM3DVIEWER_H
#define IFM3DVIEWER_H
#include <QVTKWidget.h>
#include <QString>
#include <QThread>

class IFM3DViewer : public QThread
{
    Q_OBJECT
public:
    IFM3DViewer(QObject *parent = 0);
    void initViewer(QVTKWidget *&vd);
    void openLocal();
    void openCamera();
protected:
    void virtual run() override;
private:
    QVTKWidget *vtkDisplay;
    QString filename;
};

#endif // IFM3DVIEWER_H
