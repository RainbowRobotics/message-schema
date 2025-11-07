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
#include "policy.h"
#include "safety.h"
#include "task.h"
#include "error_manager.h"

#include "comm/comm_coop.h"
#include "comm/comm_rrs.h"
#include "comm/comm_msa.h"

// qt
#include <QMainWindow>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTouchEvent>
#include <QTimer>
#include <QProcess>
#include <QMessageBox>
#include <QGraphicsOpacityEffect>
#include <QGamepad>
#include <QGamepadManager>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct MAINWINDOW_INFO
{
    static constexpr int plot_point_size = 3;
    static constexpr double opacity_val = 0.75;
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    // setter
    void set_temperature_value(float val);
    void set_is_change_map_name(bool val);
    void set_map_dir(const QString& val);
    void set_manual_led_color(const LedState& val);
    void set_is_manual_led(bool val);

    bool get_is_change_map_name();
    bool get_is_manual_led();
    float get_temperature_value();
    QString get_map_dir();
    LedState get_manual_led_color();

    void update_jog_values(double vx, double vy, double wz);

private:
    Ui::MainWindow *ui;

    // mutex
    std::shared_mutex mtx;

    // initialization other modules
    void init_modules();

    // initialization gamepad
    void init_gamepad();

    // initialization vtk
    void init_vtk();

    // initialization ui effect
    void init_ui_effect();

    // qt gui jog func
    void jog_loop();
    double apply_jog_acc(double cur_vel, double tgt_vel, double acc, double dcc, double dt);

    // ----> planned for future modularization
    void watch_loop();

    // vtk funcs
    void set_opacity(QWidget* w, double opacity);
    void picking_ray(int u, int v, int w, int h,
                     Eigen::Vector3d& center, Eigen::Vector3d& dir,
                     boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer);
    void viewer_camera_relative_control(double tx, double ty, double tz, double rx, double ry, double rz);
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction,
                                     Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);

    // plot funcs
    void plot_map();           // plot loaded map
    void plot_node();          // plot topo.json info
    void plot_pick();          // plot picking info (mouse clicked)
    void plot_info();          // plot robot's basic info
    void plot_safety();        // plot safety field info
    void plot_raw_2d();        // plot 2d lidar info
    void plot_raw_3d();        // plot 3d lidar info
    void plot_loc();           // plot localization info
    void plot_mapping();       // plot mapping info
    void plot_obs();           // plot obstacle
    void plot_ctrl();          // plot auto-control info
    void plot_cam();           // plot cam(RGB, depth) info
    void plot_tractile();      //
    void plot_process_time();  // plot thread process time
    void all_plot_clear();

    // for task
    void ui_tasks_update();

    // ----> delete soon
    void speaker_handler(int cnt);

    // handler funcs
    int led_handler();

    // utils funcs
    void get_cur_IP();

    // picking module: qt gui click event
    PICKING pick;

    // pcl viewer (point cloud viewer)
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer;

    // thread
    std::unique_ptr<std::thread> jog_thread;
    std::unique_ptr<std::thread> watch_thread;

    // thread flags
    std::atomic<bool> jog_flag   = {false};   // jog thread
    std::atomic<bool> watch_flag = {false};   // watchdog thread

    // plot update
    std::atomic<bool> is_map_update  = {false};        // To plot changes once
    std::atomic<bool> is_obs_update  = {false};        // To plot changes once
    std::atomic<bool> is_topo_update = {false};        // To plot changes once
    std::atomic<bool> is_pick_update = {false};        // To plot changes once
    std::atomic<bool> is_local_path_update = {false};  // To plot changes once
    std::atomic<bool> is_global_path_update = {false}; // To plot changes once
    std::atomic<double> plot_dt = {0.};

    // viewing flags
    std::atomic<bool> is_view_reset   = {false};  // reset vtk camera view
    std::atomic<bool> is_set_top_view = {false};  // set camera top view

    // jog flags
    std::atomic<bool> is_jog_pressed = {false};     // check jog button pressed
    std::atomic<double> last_jog_update_time = {0}; // check last update time

    // jog current & target velocity
    std::atomic<double> vx_target = {0.};
    std::atomic<double> vx_current = {0.};
    std::atomic<double> vy_target = {0.};
    std::atomic<double> vy_current = {0.};
    std::atomic<double> wz_target = {0.};
    std::atomic<double> wz_current = {0.};

    // plot object names
    std::vector<QString> last_plot_kfrms;
    std::vector<QString> last_plot_nodes;
    std::vector<QString> last_plot_names;
    std::vector<QString> last_plot_tactile;
    std::vector<QString> last_plot_local_path;
    std::vector<QString> last_obs_plot_tactile;

    // mobile odometry & lidar sync time
    atomic<double> last_sync_time = {0.};

    // for change map name
    std::atomic<bool> is_change_map_name = {false};

    // for mileage
    float old_total_mileage;
    double mileage = {0.0};
    double prev_move_distance = {0.0};
    QString mileage_sum;

    // vars
    QString map_dir = "";

    QString path_append_id = "";

    // QA value
    double cur_lx = 0.0; // left stick X  -> vy
    double cur_ly = 0.0; // left stick Y  -> vx
    double cur_rx = 0.0; // right stick X -> wz
    std::atomic<bool> lb_pressed = {false};
    std::atomic<bool> rb_pressed = {false};

    // Gamepad
    QGamepad* gamepad;

    // timer
    QTimer* plot_timer; // plot timer
    QTimer* qa_timer;   // quick annotation timer

    // for QA
    bool is_qa_running = false;
    int qa_interval_ms = 1000;
    QString qa_last_node = "";

    // etccc......
    float temperature_value = 0.f;
    LedState manual_led_color = LedState::OFF;
    std::atomic<bool> is_manual_led = {false};

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

    void bt_JogMecaL();
    void bt_JogMecaR();

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
    void bt_MapPause();
    void bt_MapResume();

    // localization
    void bt_LocInit();
    void bt_LocStart();
    void bt_LocStop();
    void bt_LocInitSemiAuto();

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

    // for autocontrol
    void bt_AutoMove();
    void bt_AutoBackMove();
    void bt_AutoMove2();
    void bt_AutoMove3();
    void bt_AutoPath();
    void bt_AutoStop();
    void bt_AutoPause();
    void bt_AutoResume();
    void bt_ReturnToCharging();
    void bt_AutoPathAppend();
    void bt_AutoPathErase();

    // for dockcontrol
    void bt_DockStart();
    void bt_DockStop();
    void bt_UnDockStart();
    void bt_ChgTrig();
    void bt_ChgStop();

    // for annotation
    void bt_AddLink1();
    void bt_AddLink2();
    void bt_DelNode();
    void bt_AnnotSave();
    void bt_QuickAddNode();
    void bt_QuickAnnotStart();
    void bt_QuickAnnotStop();
    void bt_QuickAddAruco();
    void bt_QuickAddCloud();

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
    void bt_RequestPduInfo();
    void bt_LiftPowerOn();
    void bt_LiftPowerOff();
    void bt_SetLidarField();

    // others
    void bt_Test();
    void bt_TestLed();
    void ckb_PlotKfrm();

    void slot_local_path_updated();
    void slot_global_path_updated();

    // timer loops
    void plot_loop();
    void qa_loop();

};
#endif // MAINWINDOW_H
