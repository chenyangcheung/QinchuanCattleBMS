#include "mainwindow.h"
#include <QApplication>

using namespace cv;

int main(int argc, char *argv[])
{
    QCoreApplication::setApplicationName("QinchuanCattle BMS");
    QCoreApplication::setAttribute(Qt::AA_X11InitThreads);

    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
