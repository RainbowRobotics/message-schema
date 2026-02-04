#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include "my_utils.h"
#include "comm_data.h"
#include "slamnav_ui_types.h"

#include "tinycolormap.hpp"

// modules
#include "config.h"
#include "logger.h"
#include "unimap.h"
#include "obsmap.h"
#include "mobile.h"
#include "lidar_2d.h"
#include "lidar_3d.h"
#include "lidar_bottom.h"
#include "localization.h"
#include "mapping.h"
#include "autocontrol.h"
#include "sim.h"
#include "dockcontrol.h"
#include "policy.h"
#include "safety.h"
#include "task.h"
#include "error_manager.h"
#include "aruco.h"

#include "comm_msa.h"
#include "comm_fms.h"
#include "comm_zenoh.h"

// qt
#include <QMainWindow>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTouchEvent>
#include <QTimer>
#include <QProcess>
#include <QMessageBox>
#include <QGraphicsOpacityEffect>
#include <QNetworkInterface>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

struct MAINWINDOW_INFO
{
  static constexpr int plot_point_size = 3;
  static constexpr double opacity_val = 0.75;
  static constexpr double duration_sync_time = 1200.0; // second
};

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

  // watchdog loop -> planned for future modularization
  std::atomic<bool> watch_flag = {false};
  std::unique_ptr<std::thread> watch_thread;
  void watch_loop();

  // funcs
  Eigen::Vector3d ray_intersection(Eigen::Vector3d ray_center, Eigen::Vector3d ray_direction, Eigen::Vector3d plane_center, Eigen::Vector3d plane_normal);
  void viewer_camera_relative_control(double tx, double ty, double tz, double rx, double ry, double rz);
  void picking_ray(int u, int v, int w, int h, Eigen::Vector3d& center, Eigen::Vector3d& dir, boost::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer);
  void all_plot_clear();
  void ui_tasks_update();
  void init_ui_effect();
  void set_opacity(QWidget* w, double opacity);
  void init_modules();
  void setup_vtk();
  void speaker_handler(int cnt);
  int led_handler();

  void get_cur_ip_adddress();

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

  // plot object names
  std::vector<QString> last_plot_kfrms;
  std::vector<QString> last_plot_nodes;
  std::vector<QString> last_plot_names;
  std::vector<QString> last_plot_local_path;
  std::vector<QString> last_plot_tactile;
  std::vector<QString> last_obs_plot_tactile;

  std::atomic<double> last_plot_aruco_t = {get_time()};

  atomic<double> last_sync_time = {0.};

  // for mileage
   double mileage = {0.0};
   double prev_move_distance = {0.0};

   QString mileage_sum;

   // vars
   QString map_dir = "";

   QString path_append_id = "";
   int dock_retry_count = 0;
   
  // plot funcs
  void plot_map();
  void plot_node();
  void plot_pick();
  void plot_info();
  void plot_safety();
  void plot_raw_2d();
  void plot_raw_3d();
  void plot_blidar();
  void plot_loc();
  void plot_mapping();
  void plot_obs();
  void plot_ctrl();
  void plot_cam();
  void plot_tractile();
  void plot_process_time();

  // init localization timer (aruco)
  QTimer* init_aruco_loc_timer;
  std::atomic<int> found_aruco_cnt = {0};

  // plot timer
  QTimer* plot_timer;

  // QA timer
  QTimer* qa_timer;
  bool is_qa_running = false;
  int qa_interval_ms = 1000;
  QString qa_last_node = "";

  float old_total_mileage;

private:
  double get_cpu_usage();
  double get_cpu_temperature();

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
  void spb_AccVChanged(double val);
  void spb_DecelVChanged(double val);
  void spb_AccWChanged(double val);
  void spb_DecelWChanged(double val);

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
  void bt_AnnotReload();
  void bt_QuickAddNode();
  void bt_QuickEditPos();
  void bt_ReplaceNode();
  void bt_QuickAnnotStart();
  void bt_QuickAnnotStop();
  void bt_QuickAddCloud();
  void bt_QuickAddCloud2();
  void bt_QuickAddAruco();

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

  // others
  void bt_Test();
  void bt_TestLed();
  void ckb_PlotKfrm();

  void slot_local_path_updated();
  void slot_global_path_updated();

  // timer loops
  void plot_loop();
  void qa_loop();
  void init_aruco_loc_loop();

  // Livox on / off
  void cb_LivoxSwitch();

private:
  Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
