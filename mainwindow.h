#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "global_defines.h"
#include "my_utils.h"
#include "comm_data.h"

// modules
#include "config.h"
#include "logger.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "lidar_bottom.h"
#include "cam.h"
#include "bqr_sensor.h"
#include "slam_2d.h"
#include "unimap.h"
#include "obsmap.h"
#include "aruco.h"
#include "autocontrol.h"
#include "dockcontrol.h"
#include "task.h"
#include "sim.h"
#include "lvx_loc.h"

#include "comm_fms.h"
#include "comm_rrs.h"
#include "comm_coop.h"
#include "comm_old.h"
#include "comm_ui.h"

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
    MOBILE mobile;
    LIDAR_2D lidar;
    LIDAR_BOTTOM blidar;
    CAM cam;
    BQR_SENSOR bqr;
    UNIMAP unimap;
    OBSMAP obsmap;
    SLAM_2D slam;
    ARUCO aruco;
    AUTOCONTROL ctrl;
    DOCKCONTROL dctrl;
    TASK task;    
    SIM sim;
    LVX_LOC lvx;
    COMM_FMS comm_fms;
    COMM_RRS comm_rrs;
    COMM_COOP comm_coop;
    COMM_OLD comm_old;
    COMM_UI comm_ui;

    // system logger
    LOGGER system_logger;
    std::atomic<int> log_cnt = {0};

    // funcs    
    void init_ui_effect();
    void init_modules();
    void setup_vtk();
    void all_plot_clear();
    void ui_tasks_update();
    void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer);
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);
    void update_jog_values(double vx, double vy, double wz);
    double apply_jog_acc(double cur_vel, double tgt_vel, double acc, double dcc, double dt);
    void set_mapping_view();    
    void viewer_camera_relative_control(double tx, double ty, double tz, double rx, double ry, double rz);
    void viewer_camera_relative_control2(double tx, double ty, double tz, double rx, double ry, double rz);
    
public:
    Ui::MainWindow *ui;
    std::recursive_mutex mtx;

    // vars
    QString map_dir = "";
    std::atomic<double> plot_proc_t = {0};
    std::atomic<int> test_img_save_cnt = {0};

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

    // comm loop
    std::atomic<bool> comm_flag = {false};
    std::thread *comm_thread = NULL;
    void comm_loop();

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

    // bqr localization timer
    QTimer bqr_localization_timer;

    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    // sound (only use mecanum)
    QTimer sound_timer;
    QMediaPlayer *media_player = NULL;
    QMediaPlaylist *playlist = NULL;
    std::atomic<bool> is_play = {false};
    std::atomic<bool> is_play_obs = {false};
    #endif

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

    // for copy & paste
    double area_lt_x = 0.0;
    double area_lt_y = 0.0;
    double area_rb_x = 0.0;
    double area_rb_y = 0.0;

    // for measure
    double measure_lt_x = 0.0;
    double measure_lt_y = 0.0;
    double measure_rb_x = 0.0;
    double measure_rb_y = 0.0;

    std::atomic<bool> is_select_multi = {false};
    std::atomic<bool> is_select_all = {false};
    std::atomic<bool> is_quick_move = {false};
    std::atomic<bool> is_grab = {false};
    std::atomic<bool> is_copy = {false};
    std::atomic<bool> is_pressed_btn_ctrl = {false};

    std::atomic<int> saved_select_idx = {0};
    std::vector<COPY_INFO> copy_infos;
    std::vector<QString> select_nodes;
    std::vector<std::vector<QString>> pre_select_nodes;

    // plot object names
    std::vector<QString> last_plot_kfrms;

    std::vector<QString> last_plot_nodes;
    std::vector<QString> last_plot_nodes2;
    std::vector<QString> last_plot_names;

    std::vector<QString> last_plot_local_path;
    std::vector<QString> last_plot_tactile;
    std::vector<QString> last_plot_select;

    // jog
    std::atomic<double> vx_target = {0.};
    std::atomic<double> vy_target = {0.};
    std::atomic<double> wz_target = {0.};

    std::atomic<double> vx_current = {0.};
    std::atomic<double> vy_current = {0.};
    std::atomic<double> wz_current = {0.};

    std::atomic<double> last_jog_update_t = {0};
    std::atomic<bool> is_jog_pressed = {false};

    // aruco
    std::atomic<double> last_plot_aruco_t = {0};

    // for variable loop
    std::atomic<int> lidar_view_frequency = {1};
    std::atomic<int> path_view_frequency = {2};

    // for touch event
    double lastScaleFactor = 1.0;
    QPointF lastTouchPoint;
    bool isPanning = false;

    // for user led
    std::atomic<bool> is_user_led = {false};
    std::atomic<int> user_led_color = {LED_OFF};

    // 3d plot funcs
    void map_plot();
    void topo_plot();
    void pick_plot();
    void slam_plot();
    void loc_plot();
    void obs_plot();
    void ctrl_plot();
    void raw_plot();
    void lvx_plot();

    void map_plot2();
    void topo_plot2();
    void pick_plot2();
    void loc_plot2();

protected:
    bool eventFilter(QObject *object, QEvent *ev);

public Q_SLOTS:
    // for log
    void slot_write_log(QString user_log, QString color_code);

    // for tab
    void slot_main_tab_changed();
    void slot_menu_tab_changed();

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
    void bqr_localization_loop();
    #if defined(USE_MECANUM_OLD) || defined(USE_MECANUM)
    void sound_loop();
    #endif

    // config
    void bt_ConfigLoad();

    // mobile
    void bt_MotorInit();
    void bt_Emergency();

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

    // annotation
    void bt_MapReload();
    void bt_MapSave2();
    void bt_AddNode();
    void bt_DelNode();
    void bt_DelLink();
    void bt_AddLink1();
    void bt_AddLink2();
    void bt_AutoLink();
    void bt_AutoNode();
    void bt_AddPoints();

    void bt_EditNodePos();
    void bt_EditNodeType();
    void bt_EditNodeInfo();
    void bt_EditNodeName();
    void bt_MinGapNodeX();
    void bt_MinGapNodeY();    
    void bt_AlignNodeX();
    void bt_AlignNodeY();    
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
    void bt_QuickAddNode();
    void bt_QuickAddAruco();
    void bt_QuickAddCloud();

    void bt_SetMapTf();
    void bt_CheckNodes();

    // mapping & localization
    void bt_MapBuild();    
    void bt_MapSave();
    void bt_MapLoad();
    void bt_MapLastLc();

    void bt_LocInit();
    void bt_LocInitSemiAuto();
    void bt_LocInitAuto();
    void bt_LocStart();
    void bt_LocStop();

    // for simulation
    void bt_SimInit();

    // for test
    void bt_Test();
    void bt_TestLed();
    void ckb_TestDebug();
    void bt_TestImgSave();

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

    // for advanced annot
    void cb_NodeType(QString type);

    void bt_SelectPreNodes();
    void bt_SelectPostNodes();

    // for fms
    void bt_SendMap();
    void slot_sim_random_init(QString seed);
    void slot_sim_random_seq();

    // for ldock
    void bt_DockStart();
    void bt_DockStop();
    void bt_UnDockStart();

    #if defined(USE_MECANUM)
    void slot_comm_ui_view_control(double time, QString val);
    void slot_comm_ui_view_type(double time, QString val);
    #endif

    #if defined(USE_MECANUM_OLD)
    void slot_found_obs();
    #endif
};
#endif // MAINWINDOW_H

