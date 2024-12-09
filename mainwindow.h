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
#include "aruco.h"
#include "autocontrol.h"
#include "docking.h"
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
#include <QProcess>
#include <QMessageBox>
#include <QGestureEvent>

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
    UNIMAP unimap;
    OBSMAP obsmap;
    SLAM_2D slam;
    ARUCO aruco;
    AUTOCONTROL ctrl;
    DOCKING dctrl;
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
    void all_plot_clear();
    void ui_tasks_update();
    void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer);
    Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);
    void update_jog_values(double vx, double vy, double wz);
    double apply_jog_acc(double cur_vel, double tgt_vel, double acc, double dcc, double dt);

    void handlePinchGesture(QPinchGesture* pinchGesture, QObject* object);
    void viewer_camera_relative_control(double tx, double ty, double tz, double rx, double ry, double rz);
    void viewer_camera_relative_control2(double tx, double ty, double tz, double rx, double ry, double rz);

    void handleTouchEvent(QTouchEvent* touchEvent, QObject* object);
    void viewer_camera_pan_control(double dx, double dy);
    void viewer_camera_pan_control2(double dx, double dy);

    void viewer_pan_screen(double dx, double dy,boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer,QWidget* widget);

    void syncViewerCameras(boost::shared_ptr<pcl::visualization::PCLVisualizer> sourceViewer, boost::shared_ptr<pcl::visualization::PCLVisualizer> targetViewer);


    void synchronizeViewersIfNeeded(QObject* currentWidget);
    bool wasSwitchedFromWidget(QObject* fromWidget, QObject* toWidget);
    QObject* lastActiveWidget = nullptr;
    
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
    std::vector<QString> last_plot_edges;
    std::vector<QString> last_plot_names;

    std::vector<QString> last_plot_nodes2;
    std::vector<QString> last_plot_edges2;
    std::vector<QString> last_plot_names2;

    std::vector<QString> last_plot_global_path;
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
    std::atomic<double> aruco_prev_t = {0};

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
    void signal_send_info();

public Q_SLOTS:

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
    void bt_DelNode();
    void bt_AddLink1();
    void bt_AddLink2();
    void bt_AutoLink();
    void bt_AutoNode();
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
    void slot_resist_id(QString id);
    void slot_sim_random_init(QString seed);
    void slot_sim_random_seq();

    // for log
    void slot_write_log(QString user_log, QString color_code);

    // for ldock
    void bt_DockingMove();
    void bt_DockingStop();
    void bt_Undock();
};
#endif // MAINWINDOW_H

