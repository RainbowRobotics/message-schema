#include "global_defines.h"

#include "mainwindow.h"

#include <QApplication>

Q_DECLARE_METATYPE(Eigen::Matrix4d)

int main(int argc, char *argv[])
{
    qRegisterMetaType<Eigen::Matrix4d>();

    QApplication a(argc, argv);

    // VTK
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    vtkObject::GlobalWarningDisplayOff();

    MainWindow w;
    w.show();
    return a.exec();
}
