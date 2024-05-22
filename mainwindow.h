#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "global_defines.h"
#include "utils.h"

// modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "slam_2d.h"
#include "unimap.h"

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
    LOGGER logger;
    MOBILE mobile;
    LIDAR_2D lidar;
    SLAM_2D slam;
    UNIMAP unimap;

    void init_modules();

    // funcs
    void setup_vtk();
    void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir);
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);
    void draw_picking(Eigen::Vector3d pose);
    pcl::PolygonMesh make_donut(double donut_radius, double tube_radius, Eigen::Matrix4d tf,
                                double r=1.0, double g=1.0, double b=1.0, double a=1.0, int num_segments=30);

    QString map_dir = "";

private:
    Ui::MainWindow *ui;
    std::mutex mtx;

    // pcl viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;

    // plot timer
    QTimer plot_timer;
    QTimer plot_timer2;

    // watchdog timer
    QTimer watchdog_timer;

    // plot object names
    std::vector<QString> plot_kfrm_names;

    // vars
    std::atomic<bool> is_map_update = {false};
    std::atomic<bool> is_topo_update = {false};
    std::atomic<bool> is_pick_update = {false};

    std::vector<QString> last_plot_nodes;
    std::vector<QString> last_plot_edges;
    std::vector<QString> last_plot_global_path;
    std::vector<QString> last_plot_predictions;
    std::vector<QString> last_plot_keyframe;

protected:
    bool eventFilter(QObject *object, QEvent *ev);

private Q_SLOTS:
    void plot_loop();
    void plot_loop2();
    void watchdog_loop();

    void bt_ConfigLoad();
    void bt_ConfigSave();

    void bt_MotorInit();

    void bt_Sync();
    void bt_MoveLinear();
    void bt_MoveRotate();

    void bt_JogF();
    void bt_JogB();
    void bt_JogL();
    void bt_JogR();
    void bt_JogReleased();

    // annotation
    void bt_MapEditLoad();
    void bt_MapEditSave();
    void bt_AddNode();
    void bt_AddLink1();
    void bt_AddLink2();
    void bt_EditNodePos();
    void bt_EditNodeType();
    void bt_EditNodeInfo();
    void bt_MinGapNodeX();
    void bt_MinGapNodeY();
    void bt_MinGapNodeZ();
    void bt_AlignNodeX();
    void bt_AlignNodeY();
    void bt_AlignNodeZ();
    void bt_AlignNodeTh();
    void bt_ClearTopo();
    void updateMap(int val);
    void updateTopo(int val);

    void bt_NodePoseXUp();
    void bt_NodePoseYUp();
    void bt_NodePoseThUp();
    void bt_NodePoseXDown();
    void bt_NodePoseYDown();
    void bt_NodePoseThDown();

    void ckb_PlotKfrm();

    // mapping
    void bt_MapBuild();
    void bt_MapStop();
    void bt_MapSave();
    void bt_MapLoad();

    void bt_LocInit();
    void bt_LocInit2();
    void bt_LocStart();
    void bt_LocStop();

};
#endif // MAINWINDOW_H

