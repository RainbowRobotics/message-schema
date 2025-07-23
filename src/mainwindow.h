#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "global_defines.h"
#include "my_utils.h"
#include "comm_data.h"

// modules
#include "config.h"
#include "logger.h"
#include "unimap.h"
#include "obsmap.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "lidar_3d.h"
#include "localization.h"
#include "mapping.h"
#include "autocontrol.h"
#include "sim.h"
#include "dockcontrol.h"


#include "comm/comm_coop.h"
#include "comm/comm_rrs.h"

// qt
#include <QMainWindow>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTouchEvent>
#include <QTimer>
#include <QProcess>
#include <QMessageBox>
#include <QGraphicsOpacityEffect>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // picking module: qt gui click event
    PICKING pick;

    // pcl viewer (point cloud viewer)
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;

    // qt gui jog func
    std::atomic<bool> jog_flag = {false};
    std::unique_ptr<std::thread> jog_thread;
    void jog_loop();

    // watchdog loop -> planned for future modularization
    std::atomic<bool> watch_flag = {false};
    std::unique_ptr<std::thread> watch_thread;
    void watch_loop();

    // funcs
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);
    double apply_jog_acc(double cur_vel, double tgt_vel, double acc, double dcc, double dt);
    void viewer_camera_relative_control(double tx, double ty, double tz, double rx, double ry, double rz);
    void update_jog_values(double vx, double vy, double wz);
    void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer);
    void all_plot_clear();
    void init_ui_effect();
    void set_opacity(QWidget* w, double opacity);
    void init_modules();
    void setup_vtk();

    QString get_map_path();
    void set_map_path(const QString& path);

    // vars
    std::atomic<double> plot_dt = {0.};

    // flags
    std::atomic<bool> is_map_update = {false};
    std::atomic<bool> is_topo_update = {false};
    std::atomic<bool> is_pick_update = {false};
    std::atomic<bool> is_obs_update = {false};
    std::atomic<bool> is_local_path_update = {false};
    std::atomic<bool> is_global_path_update = {false};
    std::atomic<bool> is_3d_update = {false};

    std::atomic<bool> is_view_reset = {false};
    std::atomic<bool> is_set_top_view = {false};

    // jog
    std::atomic<bool> is_jog_pressed = {false};
    std::atomic<double> last_jog_update_time = {0};

    std::atomic<double> vx_target = {0.};
    std::atomic<double> vy_target = {0.};
    std::atomic<double> wz_target = {0.};

    std::atomic<double> vx_current = {0.};
    std::atomic<double> vy_current = {0.};
    std::atomic<double> wz_current = {0.};

    // plot object names
    std::vector<QString> last_plot_kfrms;
    std::vector<QString> last_plot_nodes;
    std::vector<QString> last_plot_names;
    std::vector<QString> last_plot_local_path;
    std::vector<QString> last_plot_tactile;
    std::vector<QString> last_obs_plot_tactile;

    atomic<double> last_sync_time = {0.};

    // plot cur loc pts
    std::vector<Eigen::Vector3d> plot_cur_pts;

    // for user led
    std::atomic<bool> is_user_led = {false};
    std::atomic<int> user_led_color = {LED_OFF};

    // usb temp sensor
    std::atomic<float> temperature_value = {0.0};

    // plot funcs
    void plot_map();
    void plot_node();
    void plot_pick();
    void plot_info();
    void plot_safety();
    void plot_raw_2d();
    void plot_raw_3d();
    void plot_loc();
    void plot_mapping();
    void plot_obs();
    void plot_ctrl();
    void plot_tractile();
    void plot_process_time();

    // plot timer
    QTimer* plot_timer;

protected:
    bool eventFilter(QObject *object, QEvent *ev);

Q_SIGNALS:
    void signal_move_response(DATA_MOVE msg);

public Q_SLOTS:

    // replot
    void map_update();
    void obs_update();
    void topo_update();
    void pick_update();
    void all_update();

    void vtk_viewer_update(int val);

    // config
    void bt_ConfigLoad();

    // mobile
    void bt_Emergency();
    void bt_MotorInit();

    void bt_Sync();
    void bt_MoveStop();
    void bt_MoveLinearX();
    void bt_MoveLinearY();
    void bt_MoveRotate();

    void bt_JogF();
    void bt_JogB();
    void bt_JogL();
    void bt_JogR();
    void bt_JogReleased();

    // for simulation
    void bt_SimInit();

    // mapping
    void bt_MapBuild();
    void bt_MapSave();
    void bt_MapLoad();
    void bt_MapLastLc();

    // localization
    void bt_LocInit();
    void bt_LocStart();
    void bt_LocStop();

    // for obsmap
    void bt_ObsClear();

    // for autocontrol
    void bt_AutoMove();
    void bt_AutoMove2();
    void bt_AutoMove3();
    void bt_AutoStop();
    void bt_AutoPause();
    void bt_AutoResume();
    void bt_ReturnToCharging();

    // for dockcontrol
    void bt_DockStart();
    void bt_DockStop();
    void bt_UnDockStart();

    // for annotation
    void bt_DelNode();
    void bt_AnnotSave();
    void bt_QuickAddNode();

    // for safety debug
    void bt_ClearMismatch();
    void bt_ClearOverSpd();
    void bt_ClearObs();
    void bt_ClearFieldMis();
    void bt_ClearInterlockStop();
    void bt_BumperStop();
    void bt_Recover();
    void bt_SetDetectModeT();
    void bt_SetDetectModeF();
    void bt_Request();
    void bt_LiftPowerOn();
    void bt_LiftPowerOff();
    void bt_SetLidarField();

    // for test
    void bt_TestLed();

    void slot_local_path_updated();
    void slot_global_path_updated();

    // timer loops
    void plot_loop();

private:
    Ui::MainWindow *ui;
    std::shared_mutex mtx;

    QString map_path = "";
};
#endif // MAINWINDOW_H
