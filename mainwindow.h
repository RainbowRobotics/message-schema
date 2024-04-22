#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "global_defines.h"
#include "utils.h"

// modules
#include "config.h"
#include "mobile.h"

// qt
#include <QMainWindow>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // modules
    PICKING pick;
    CONFIG config;
    MOBILE mobile;

    void init_modules();

    // funcs
    void setup_vtk();
    void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir);
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);

    void draw_picking(Eigen::Vector3d pose);

private:
    Ui::MainWindow *ui;
    std::mutex mtx;

    // pcl viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // plot timer
    QTimer plot_timer;

    // watchdog timer
    QTimer watchdog_timer;

protected:
    bool eventFilter(QObject *object, QEvent *ev);

private Q_SLOTS:
    void plot_loop();
    void watchdog_loop();

    void bt_ConfigLoad();
    void bt_ConfigSave();

    void bt_MotorInit();

    void bt_Sync();

    void bt_JogF();
    void bt_JogB();
    void bt_JogL();
    void bt_JogR();
    void bt_JogReleased();

};
#endif // MAINWINDOW_H
