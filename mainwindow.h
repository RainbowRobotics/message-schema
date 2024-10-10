#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "global_defines.h"
#include "utils.h"

// modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "lidar_bottom.h"
#include "cam.h"
#include "code_reader.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"
#include "autocontrol.h"
#include "dockingcontrol.h"
#include "task.h"
#include "sim.h"
#include "comm_fms.h"
#include "comm_ms.h"
#include "comm_ui.h"

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
    LIDAR_BOTTOM blidar;
    CAM cam;
    CODE_READER code;
    SLAM_2D slam;
    UNIMAP unimap;
    OBSMAP obsmap;
    AUTOCONTROL ctrl;
    DOCKINGCONTROL dctrl;
    TASK task;    
    SIM sim;
    COMM_FMS cfms;
    COMM_MS cms;
    COMM_UI cui;

    // system logger
    LOGGER system_logger;
    std::atomic<int> log_cnt = {0};

    // funcs
    void init_modules();
    void setup_vtk();
    void ui_tasks_update();
    void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer);
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);
    void update_jog_values(double vx, double vy, double wz);
    double apply_jog_acc(double cur_vel, double tar_vel, double acc, double decel, double dt);

public:
    Ui::MainWindow *ui;
    std::mutex mtx;

    // vars
    QString map_dir = "";
    std::atomic<double> plot_proc_t = {0};

    // pcl viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2;

    // plot timer
    QTimer plot_timer;
    QTimer plot_timer2;

    // semi auto init
    std::atomic<bool> semi_auto_init_flag = {false};
    std::thread *semi_auto_init_thread = NULL;
    void semi_auto_init_loop();

    // auto init
    std::atomic<bool> auto_init_flag = {false};
    std::thread *auto_init_thread = NULL;
    void auto_init_loop();

    // watchdog
    std::atomic<bool> watch_flag = {false};
    std::thread *watch_thread = NULL;
    void watch_loop();

    // jog
    std::atomic<bool> jog_flag = {false};
    std::thread *jog_thread = NULL;
    void jog_loop();

    // quick annotation timer
    QTimer qa_timer;
    QString qa_last_node = "";

    // flags
    std::atomic<bool> is_map_update = {false};
    std::atomic<bool> is_topo_update = {false};
    std::atomic<bool> is_pick_update = {false};
    std::atomic<bool> is_obs_update = {false};
    std::atomic<bool> is_local_path_update = {false};
    std::atomic<bool> is_global_path_update = {false};

    std::atomic<bool> is_map_update2 = {false};
    std::atomic<bool> is_topo_update2 = {false};
    std::atomic<bool> is_pick_update2 = {false};
    std::atomic<bool> is_local_path_update2 = {false};
    std::atomic<bool> is_global_path_update2 = {false};

    std::atomic<bool> is_view_reset = {false};
    std::atomic<bool> is_set_top_view = {false};

    // plot object names
    std::vector<QString> last_plot_kfrms;

    std::vector<QString> last_plot_nodes;
    std::vector<QString> last_plot_edges;
    std::vector<QString> last_plot_names;

    std::vector<QString> last_plot_nodes2;
    std::vector<QString> last_plot_edges2;
    std::vector<QString> last_plot_names2;

    std::vector<QString> last_plot_global_path;
    std::vector<QString> last_plot_local_path;
    std::vector<QString> last_plot_tactile;

    // jog
    std::atomic<bool> key_active    = {false};
    std::atomic<bool> button_active = {false};

    double vx_target = 0;
    double vy_target = 0;
    double wz_target = 0;

    double vx_current = 0;
    double vy_current = 0;
    double wz_current = 0;

    // 3d plot funcs
    void map_plot();
    void obs_plot();
    void topo_plot();
    void pick_plot();
    void slam_plot();
    void loc_plot();
    void raw_plot();
    void ctrl_plot();

    void map_plot2();
    void topo_plot2();
    void pick_plot2();
    void loc_plot2();

protected:
    bool eventFilter(QObject *object, QEvent *ev);

Q_SIGNALS:
    void signal_send_status();

public Q_SLOTS:

    // replot
    void map_update();
    void obs_update();
    void topo_update();
    void pick_update();    
    void all_update();

    // for view control
    void bt_ViewReset();
    void bt_SetTopView();

    // timer loops
    void plot_loop();
    void plot_loop2();
    void qa_loop();

    // config
    void bt_ConfigLoad();

    // mobile
    void bt_MotorInit();
    void bt_Emergency();

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
    void bt_AlignNodeX();
    void bt_AlignNodeY();    
    void bt_AlignNodeTh();
    void bt_ClearTopo();

    void bt_QuickAnnotStart();
    void bt_QuickAnnotStop();

    // mapping & localization
    void bt_MapBuild();    
    void bt_MapSave();
    void bt_MapLoad();

    void bt_LocInit();
    void bt_LocInitSemiAuto();
    void bt_LocInitAuto();
    void bt_LocStart();
    void bt_LocStop();

    // for simulation
    void bt_SimInit();

    // for test
    void bt_Test();

    // for autocontrol
    void bt_AutoMove();
    void bt_AutoMove2();
    void bt_AutoMove3();
    void bt_AutoStop();
    void bt_AutoPause();
    void bt_AutoResume();

    void slot_local_path_updated();
    void slot_global_path_updated();

    // for obsmap
    void bt_ObsClear();

    // for task
    void bt_TaskAdd();
    void bt_TaskDel();
    void bt_TaskSave();
    void bt_TaskLoad();
    void bt_TaskPlay();
    void bt_TaskPause();    
    void bt_TaskCancel();

};
#endif // MAINWINDOW_H

