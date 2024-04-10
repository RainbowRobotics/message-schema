#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "global_defines.h"

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void setup_vtk();

private:
    Ui::MainWindow *ui;

    // pcl viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

private Q_SLOTS:

};
#endif // MAINWINDOW_H
