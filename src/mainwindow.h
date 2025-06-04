#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "global_defines.h"
#include "my_utils.h"
#include "comm_data.h"

// modules
#include "config.h"
#include "logger.h"
#include "unimap.h"

// qt
#include <QMainWindow>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTouchEvent>
#include <QTimer>
#include <QProcess>
#include <QMessageBox>
#include <QGraphicsOpacityEffect>

#include <QMediaPlayer>
#include <QMediaPlaylist>

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
    LOGGER logger;
    UNIMAP unimap;

    // pcl viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // funcs
    void setup_vtk();
    void init_modules();
    void init_ui_effect();
    void all_plot_clear();

    void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer);
    void viewer_camera_relative_control(double tx, double ty, double tz, double rx, double ry, double rz);
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);


    std::atomic<double> plot_proc_t = {0};

    // flags
    std::atomic<bool> is_map_update = {false};
    std::atomic<bool> is_topo_update = {false};
    std::atomic<bool> is_pick_update = {false};
    std::atomic<bool> is_obs_update = {false};
    std::atomic<bool> is_local_path_update = {false};
    std::atomic<bool> is_global_path_update = {false};
    std::atomic<bool> is_3d_update = {false};

    // plot object names
    std::vector<QString> last_plot_nodes;
    std::vector<QString> last_plot_names;

    // plot funcs
    void map_plot();
    void topo_plot();
    void pick_plot();

    // plot timer
    QTimer plot_timer;

protected:
    bool eventFilter(QObject *object, QEvent *ev);

public Q_SLOTS:

    // replot
    void map_update();
    void obs_update();
    void topo_update();
    void pick_update();
    void all_update();

    // mapping & localization
    void bt_MapLoad();

    // for simulation
    void bt_SimInit();

    // timer loops
    void plot_loop();

public:
    Ui::MainWindow *ui;
    std::recursive_mutex mtx;

};
#endif // MAINWINDOW_H
