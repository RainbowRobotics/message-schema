#include "global_defines.h"

#include "mainwindow.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // VTK
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    vtkObject::GlobalWarningDisplayOff();

    MainWindow w;
    w.show();
    return a.exec();
}
