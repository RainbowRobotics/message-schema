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
#include "lidar_3d.h"
#include "localization.h"
#include "autocontrol.h"
#include "sim.h"

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
    OBSMAP obsmap;
    MOBILE mobile;
    LIDAR_3D lidar_3d;
    LOCALIZATION loc;
    AUTOCONTROL ctrl;
    SIM sim;

    // pcl viewer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

    // jog
    std::atomic<bool> jog_flag = {false};
    std::thread *jog_thread = NULL;
    void jog_loop();

    // watchdog
    std::atomic<bool> watch_flag = {false};
    std::thread *watch_thread = NULL;
    void watch_loop();

    // funcs
    void setup_vtk();
    void init_modules();
    void init_ui_effect();
    void all_plot_clear();

    void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer);
    void viewer_camera_relative_control(double tx, double ty, double tz, double rx, double ry, double rz);
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);
    void update_jog_values(double vx, double vy, double wz);
    double apply_jog_acc(double cur_vel, double tgt_vel, double acc, double dcc, double dt);

    std::atomic<double> plot_proc_t = {0};

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
    std::atomic<double> vx_target = {0.};
    std::atomic<double> vy_target = {0.};
    std::atomic<double> wz_target = {0.};

    std::atomic<double> vx_current = {0.};
    std::atomic<double> vy_current = {0.};
    std::atomic<double> wz_current = {0.};

    std::atomic<double> last_jog_update_t = {0};
    std::atomic<bool> is_jog_pressed = {false};

    // plot object names
    std::vector<QString> last_plot_nodes;
    std::vector<QString> last_plot_names;

    std::vector<QString> last_plot_local_path;
    std::vector<QString> last_plot_tactile;

    // plot cur loc pts
    std::vector<Eigen::Vector3d> plot_cur_pts;

    // for user led
    std::atomic<bool> is_user_led = {false};
    std::atomic<int> user_led_color = {LED_OFF};

    // plot funcs
    void plot_map();
    void plot_topo();
    void plot_pick();
    void plot_info();
    void plot_raw();
    void plot_loc();
    void plot_obs();
    void plot_ctrl();

    // plot timer
    QTimer plot_timer;

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

    // mapping & localization
    void bt_MapLoad();

    // for simulation
    void bt_SimInit();

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

    void slot_local_path_updated();
    void slot_global_path_updated();

    // timer loops
    void plot_loop();

public:
    Ui::MainWindow *ui;
    std::recursive_mutex mtx;

};
#endif // MAINWINDOW_H
