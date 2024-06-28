#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "global_defines.h"
#include "utils.h"

// modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "cam.h"
#include "code_reader.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"
#include "autocontrol.h"
#include "ws_client.h"
#include "sim.h"

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
    CAM cam;
    CODE_READER code;
    SLAM_2D slam;
    UNIMAP unimap;
    OBSMAP obsmap;
    AUTOCONTROL ctrl;
    WS_CLIENT ws;
    SIM sim;

    // funcs
    void init_modules();
    void setup_vtk();
    void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer);
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);

private:
    Ui::MainWindow *ui;
    std::mutex mtx;

    // vars
    QString map_dir = "";

    // pcl viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;

    // plot timer
    QTimer plot_timer;
    QTimer plot_timer2;

    // watchdog
    std::atomic<bool> watch_flag = {false};
    std::thread *watch_thread = NULL;
    void watch_loop();

    // quick annotation timer
    QTimer qa_timer;
    QString qa_last_node = "";

    // flags
    std::atomic<bool> is_map_update = {false};
    std::atomic<bool> is_topo_update = {false};
    std::atomic<bool> is_pick_update = {false};
    std::atomic<bool> is_path_update = {false};
    std::atomic<bool> is_obs_update = {false};

    std::atomic<bool> is_map_update2 = {false};
    std::atomic<bool> is_topo_update2 = {false};
    std::atomic<bool> is_pick_update2 = {false};

    // plot object names
    std::vector<QString> last_plot_kfrms;

    std::vector<QString> last_plot_nodes;
    std::vector<QString> last_plot_edges;

    std::vector<QString> last_plot_nodes2;
    std::vector<QString> last_plot_edges2;

    std::vector<QString> last_plot_global_path;
    std::vector<QString> last_plot_local_path;
    std::vector<QString> last_plot_tactile;

protected:
    bool eventFilter(QObject *object, QEvent *ev);

private Q_SLOTS:
    // replot
    void map_update();
    void obs_update();
    void topo_update();
    void pick_update();    
    void all_update();

    // timer loops
    void plot_loop();
    void plot_loop2();    
    void qa_loop();

    // config
    void bt_ConfigLoad();
    void bt_ConfigSave();

    // mobile
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
    void bt_MapReload();
    void bt_MapSave2();
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

    void bt_NodePoseXUp();
    void bt_NodePoseYUp();
    void bt_NodePoseThUp();
    void bt_NodePoseXDown();
    void bt_NodePoseYDown();
    void bt_NodePoseThDown();

    void bt_QuickAnnotStart();
    void bt_QuickAnnotStop();

    // mapping & localization
    void bt_MapBuild();    
    void bt_MapSave();
    void bt_MapLoad();

    void bt_LocInit();
    void bt_LocInit2();
    void bt_LocStart();
    void bt_LocStop();

    // for simulation
    void bt_SimInit();

    // for autocontrol
    void bt_AutoMove();
    void bt_AutoMove2();
    void bt_AutoMove3();
    void bt_AutoStop();

    // for ws
    void ws_motorinit(double time);
    void ws_move(double time, double vx, double vy, double wz);
    void ws_mapping_start(double time);
    void ws_mapping_stop(double time);
    void ws_mapping_save(double time, QString name);

    // for obsmap
    void bt_ObsClear();

};
#endif // MAINWINDOW_H

