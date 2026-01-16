#ifndef CONFIG_H
#define CONFIG_H

#include <algorithm>
#include <cmath>
#include <shared_mutex>

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QMessageBox>
#include <QMutex>
#include <QMutexLocker>
#include <QObject>
#include <QStringList>
#include <QCoreApplication>

#include "error_manager.h"
#include "slamnav_common_types.h"

constexpr int max_cam_cnt = 10;

class CONFIG : public QObject
{
  Q_OBJECT
  Q_DISABLE_COPY(CONFIG)

public:
  QRecursiveMutex q_mtx;

  // make singleton
  static CONFIG* instance(QObject* parent = nullptr);

  // Get base path that works whether binary is in root or bin/ directory
  static QString getBasePath();

  // load robot type
  bool load_common(QString path);

  // set config path
  void set_config_path(const QString& path);
  bool set_cam_order(const std::vector<QString> &_cam_serial_number);
  bool set_value_change(QString key, QString value);
  void set_version_path(const QString& path);
  void set_serial_number_path(const QString& path);

  // load config.json
  void load();
  void load_version();
  void load_cam_serial_number();

  /***********************
   * robot unique
   ***********************/
  QString get_robot_type_str();     // robot type_str
  QString get_robot_model_str();      // robot model_str
  QString get_robot_wheel_type();     // robot wheel type
  QString get_robot_serial_number();    // robot serial number: in the case of manufactured goods, the production team manages them. Otherwise, leave it blank.
  RobotType get_robot_type();       // robot type  : S100-A-3D
  RobotModel get_robot_model();     // robot model : S100, D400, Mecanum, SEM ...

  /***********************
   * robot hardware
   ***********************/
  double get_robot_size_x_min();      // robot x-axis minimum size
  double get_robot_size_x_max();      // robot x-axis maximum size
  double get_robot_size_y_min();      // robot y-axis minimum size
  double get_robot_size_y_max();      // robot y-axis maximum size
  double get_robot_size_z_min();      // robot z-axis minimum size
  double get_robot_size_z_max();      // robot z-axis maximum size
  double get_robot_size_add_x();      // robot x-axis maximum size plus
  double get_robot_size_add_y();      // robot y-axis maximum size plus
  double get_robot_size_add_z();      // robot y-axis maximum size plus
  double get_robot_wheel_base();      // the distance between the robot's two wheels
  double get_robot_wheel_radius();    // robot wheel radius (mm)
  double get_robot_radius();        // robot radius (auto calc)
  bool get_robot_use_speaker();     // robot io-speaker use
  double get_robot_lx();          // robot lx
  double get_robot_ly();          // robot ly
  double get_robot_alarm_bat_low();   // robot battery low alarm threshold (%)
  double get_robot_alarm_bat_critical();  // robot battery critical alarm threshold (%)
  bool get_robot_use_alarm();       // robot use robot alarm

  /***********************
   * sensor common
   ***********************/
  bool  get_use_lidar_2d();           // check if the robot uses 2d lidar
  bool  get_use_lidar_2d_obstacle();      // check if the robot uses 2d lidar
  QString get_lidar_2d_type();          // check 2d lidar type
  QString get_lidar_2d_dev(int idx);        // check 2d lidar dev
  int   get_lidar_2d_num();           // check 2d lidar number
  bool  get_use_lidar_3d();           // check if the robot uses 3d lidar
  bool  get_use_lidar_3d_obstacle();      // check if the robot uses 3d lidar
  QString get_lidar_3d_type();          // check 3d lidar type
  int   get_lidar_3d_num();           // check 3d lidar number
  bool  get_use_blidar();            // check if the robot uses bottom lidar
  bool  get_use_cam();              // check if the robot uses cam
  bool  get_use_cam_obstacle();         // check if the robot uses 3d lidar
  bool  get_use_cam_rgb();            // check if the robot uses cam rgb
  bool  get_use_cam_depth();          // check if the robot uses cam depth
  bool  get_use_cam_filter();         // check if the robot uses cam filter
  double  get_cam_filter_ror_radius();      // cam filter radius of removal radius
  int   get_cam_filter_ror_min_neighbors();   // cam filter radius of removal min neighbors
  int   get_cam_filter_cluster_min_size();    // cam filter cluster min size
  QString get_cam_type();             // check cam type
  int   get_cam_num();              // check cam number
  bool  get_use_bqr();              // check if the robot uses bottom QR sensor
  bool  get_use_imu();              // check if the robot uses external IMU
  bool  get_use_aruco();            // check if the robot uses aruco marker (must used cam)
  bool  get_use_ekf();              // check if the robot uses ekf(Extended-Kalman-Filter)


  /***********************
   * localization
   ***********************/
  QString get_loc_mode();             // localization mode : 2D, 3D

  /***********************
   * networking
   ***********************/
  bool get_use_multi();             // check if the robot mode is multi control
  bool get_use_rtsp();              // check if the robot uses rtsp streaming (cam img)
  bool get_use_msa();               // check if the robot uses msa communication
  bool get_use_fms();               // check if the robot uses fms direct communication (must used test or simulation)

  /***********************
   * debug & test
   ***********************/
  bool get_use_sim();               // check if the robot uses simulation mode
  bool get_use_beep();              // check if the robot uses beep sound (internal beep sound when obstacle is detected)
  QString get_server_ip();            // get server ip address
  QString get_server_id();            // get server id
  QString get_server_pw();            // get server password

  /***********************
   * logging
   ***********************/
  QString get_log_level();            // get spdlog level (trace, debug, info, warn, error, critical, off)
  bool get_debug_lidar_2d();            // enable/disable LIDAR 2D debug logs
  bool get_debug_lidar_3d();            // enable/disable LIDAR 3D debug logs
  bool get_debug_mobile();            // enable/disable MOBILE debug logs
  bool get_debug_comm_rrs();            // enable/disable COMM RRS debug logs
  bool get_debug_autocontrol();         // enable/disable AUTOCONTROL debug logs
  bool get_debug_localization();          // enable/disable LOCALIZATION debug logs
  bool get_debug_obsmap();            // enable/disable OBSMAP debug logs

  bool get_log_enable_file_output();        // enable/disable file output
  QString get_log_file_path();          // get log file path
  void set_spdlog_level();            // setup spdlog level from config

  /***********************
   * lidar bottom sensor
   ***********************/
  double get_lidar_bottom_min_range();    // bottom lidar min range (defalut: 0.05~0.1m)
  double get_lidar_bottom_max_range();    // bottom lidar max range (defalut: depends on the lidar)
  QString get_lidar_bottom_ip(int idx);   // bottom lidar ip list (maximum lidar number is 2)
  QString get_lidar_bottom_tf(int idx);   // bottom lidar tf list (maximum lidar number is 2)
  QString get_lidar_bottom_dev(int idx);    // bottom lidar tty dev

  /***********************
   * 2d lidar sensor
   ***********************/
  double get_lidar_2d_min_range();      // 2d lidar min range (defalut: 0.05~0.1m)
  double get_lidar_2d_max_range();      // 2d lidar max range (defalut: depends on the lidar)
  QString get_lidar_2d_ip(int idx);     // 2d lidar ip list (maximum lidar number is 2)
  QString get_lidar_2d_tf(int idx);     // 2d lidar tf list (maximum lidar number is 2)

  /***********************
   * 3d lidar sensor
   ***********************/
  double get_lidar_3d_min_range();      // 3d lidar min range (defalut: 0.05~0.1m)
  double get_lidar_3d_max_range();      // 3d lidar max range (defalut: depends on the lidar)
  QString get_lidar_3d_ip(int idx);     // 3d lidar ip list (maximum lidar number is 2)
  QString get_lidar_3d_tf(int idx);     // 3d lidar tf list (maximum lidar number is 2)

  /***********************
   * cam sensor
   ***********************/
  double get_cam_height_min();        // depth camera min height
  double get_cam_height_max();        // depth camera max height
  QString get_cam_serial_number(int idx);   // cam serial number list (maximum cam number is 2)
  QString get_cam_tf(int idx);        // cam tf list (maximum cam number is 2)
  int   get_cam_color_profile(int idx);   // cam color profile list (maximum cam number is 4)
  int   get_cam_depth_profile(int idx);   // cam depth profile list (maximum cam number is 4)

  /***********************
   * motor
   ***********************/
  int get_motor_id_left();          // get left motor id (0,1,2,3)
  int get_motor_id_right();         // get right motor id (0,1,2,3)
  double get_motor_direction();       // get motor direction (1, -1)
  double get_motor_gear_ratio();        // get motor gear ratio
  double get_motor_limit_v();         // get motor limit v (hardware limit, Software speed limit cannot exceed this value)
  double get_motor_limit_v_acc();       // get motor limit v acc (hardware limit, Software speed limit cannot exceed this value)
  double get_motor_limit_w();         // get motor limit w (hardware limit, Software speed limit cannot exceed this value)
  double get_motor_limit_w_acc();       // get motor limit w acc (hardware limit, Software speed limit cannot exceed this value)
  double get_motor_gain_kp();         // get motor internal position control pid p gain
  double get_motor_gain_ki();         // get motor internal position control pid i gain
  double get_motor_gain_kd();         // get motor internal position control pid d gain
  double get_motor_safety_limit_v();      // get motor safety limit velocity : If the motor speed exceeds this variable, the system will stop under the SS1 category.
  double get_motor_safety_limit_w();      // get motor safety limit w_velocity : If the motor speed exceeds this variable, the system will stop under the SS1 category.

  /***********************
   * mapping
   ***********************/
  int get_mapping_icp_max_feature_num();    // Number of points involved in calculation when performing ICP (iterate closest points)
  int get_mapping_icp_do_erase_gap();     // Dynamic Obstacle removal gap
  int get_mapping_icp_do_accum_num();     // Cumulative number of dynamic obstacle judgments
  int get_mapping_kfrm_update_num();      // Number of frames required to update a keyframe
  int get_mapping_window_size();        // Mapping window size
  double get_mapping_icp_cost_threshold();  // ICP cost threadhold
  double get_mapping_icp_error_threshold(); // ICP error threadhold
  double get_mapping_icp_view_threshold();  // ICP view threadhold
  double get_mapping_kfrm_lc_try_dist();    // Loop Closing try distance
  double get_mapping_kfrm_lc_try_overlap(); // Loop Closing try overlap
  double get_mapping_voxel_size();      // common voxel size

  /***********************
   * localization 2d
   ***********************/
  int get_loc_2d_icp_max_feature_num();       // Number of points involved in calculation when performing ICP (iterate closest points)
  int get_loc_2d_surfel_num();            // Number of surfel judgment points
  double get_loc_2d_icp_odometry_fusion_ratio();    // Odometry(wheel) and Lidar positioning fusion rate
  double get_loc_2d_icp_cost_threshold();       // ICP cost threadhold
  double get_loc_2d_icp_cost_threshold0();      // ICP cost threadhold (Initial location estimation)
  double get_loc_2d_icp_error_threshold();      // ICP error threadhold
  double get_loc_2d_aruco_odometry_fusion_ratio();  // Odometry(aruco marker) and Lidar positioning fusion rate
  double get_loc_2d_aruco_odometry_fusion_dist();   // Odometry(aruco marker) and Lidar positioning fusion possible distance
  double get_loc_2d_surfel_range();         // Surfel judgment range
  double get_loc_2d_check_dist();           // ICP possible distance
  double get_loc_2d_check_inlier_ratio();       // ICP inlier ratio
  double get_loc_2d_check_inlier_error();       // ICP inlier error
  bool get_loc_2d_use_rotation_fallback();      // enable rotation fallback search
  double get_loc_2d_rotation_fallback_step();     // rotation fallback step(deg)
  double get_loc_2d_rotation_fallback_range();    // rotation fallback range(deg)

  /***********************
   * localization 3d
   ***********************/
  int get_loc_3d_icp_max_feature_num();       // Number of points involved in calculation when performing ICP (iterate closest points)
  int get_loc_3d_surfel_nn_num();           // Number of surfel judgment points
  double get_loc_3d_surfel_balance();         // surfel balance (To avoid being biased towards one axis)
  double get_loc_3d_cost_threshold();         // ICP cost threadhold
  double get_loc_3d_inlier_check_dist();        // ICP possible distance

  /***********************
   * obstacle map
   ***********************/
  int get_obs_avoid_mode();           // is avoid mode (0: not avoie, 1: avoid mode)
  double get_obs_deadzone();            // obstacle stopping distance
  double get_obs_local_goal_dist();       // local goal distance
  double get_obs_safe_margin_x();         // safe margin x-axis
  double get_obs_safe_margin_y();         // safe margin y-axis
  double get_obs_path_margin_x();         // path margin x-axis
  double get_obs_path_margin_y();         // path margin y-axis
  double get_obs_map_grid_size();         // obstacle map grid size
  double get_obs_map_range();           // obstacle map range
  double get_obs_map_min_v();           // obstacle map minimum velocity
  double get_obs_map_min_z();           // obstacle map minimum z height
  double get_obs_map_max_z();           // obstacle map maximum z height
  double get_obs_predict_time();          // obstacle prediction time
  double get_obs_distance_led_near();       // LED near distance
  double get_obs_distance_led_far();        // LED far distance

  /***********************
   * drive
   ***********************/
  double get_drive_goal_approach_gain();      // drive goal approach gain
  double get_drive_goal_dist();         // drive goal distance threshold
  double get_drive_goal_th();           // drive goal angle threshold
  double get_drive_extended_control_time();   // drive extended control time
  double get_drive_v_deadzone();          // drive linear velocity deadzone
  double get_drive_w_deadzone();          // drive angular velocity deadzone

  /***********************
   * docking
   ***********************/
  int get_docking_type();                 // docking type (0: L_dock, 1: FQR_dock, 2: bqr)
  double get_docking_pointdock_margin();          // point dock margin distance
  double get_docking_goal_dist();             // docking goal distance threshold
  double get_docking_goal_th();             // docking goal angle threshold
  double get_docking_kp_dist();             // docking PD controller proportional gain for distance
  double get_docking_kd_dist();             // docking PD controller derivative gain for distance
  double get_docking_kp_th();               // docking PD controller proportional gain for angle
  double get_docking_kd_th();               // docking PD controller derivative gain for angle
  double get_docking_clust_dist_threshold();        // docking cluster distance threshold
  double get_docking_clust_dist_threshold_min();      // docking cluster distance threshold minimum
  double get_docking_clust_dist_threshold_max();      // docking cluster distance threshold maximum
  double get_docking_clust_angle_threshold();       // docking cluster angle threshold
  double get_docking_size_x_min();            // docking size x-axis minimum
  double get_docking_size_x_max();            // docking size x-axis maximum
  double get_docking_icp_err_threshold();         // docking ICP error threshold
  double get_docking_find_vmark_dist_threshold_max();   // docking find vmark distance threshold maximum
  double get_docking_chg_length();            // docking charge length
  double get_docking_dwa_yaw_weight();          // docking DWA yaw weight
  double get_docking_check_motor_a();           // docking check motor acceleration
  double get_docking_waiting_time();            // docking waiting time
  double get_docking_final_icp_err_threshold();     // docking final ICP error threshold
  double get_docking_x_offset();              // docking x-axis offset
  double get_docking_y_offset();              // docking y-axis offset
  double get_docking_linear_x_offset();         // docking linear x-axis offset
  bool get_docking_reverse_mode();            // docking reverse mode
  QString get_charge_type();                // charge type (RAINBOW, XNERGY, etc)
  int get_docking_field();                // docking field number
  double get_xnergy_set_current();            // xnergy set current
  int get_docking_retry_count();              // docking retry count
  /***********************
   * map
   ***********************/
  QString get_map_path();               // get map file path
  void set_map_path(const QString& path);       // set map file path

  /***********************
   * safety monitoring
   ***********************/
  bool get_use_monitoring_field();          // check if the robot uses monitoring field
  int get_monitoring_field_num();           // get number of monitoring fields
  double get_monitoring_traj_dt();          // get monitoring trajectory delta time
  double get_monitoring_predict_t();          // get monitoring prediction time
  std::vector<double> get_field_size_min_x();     // get field size minimum x values
  std::vector<double> get_field_size_max_x();     // get field size maximum x values
  std::vector<double> get_field_size_min_y();     // get field size minimum y values
  std::vector<double> get_field_size_max_y();     // get field size maximum y values
  bool get_use_safety_cross_monitor();        // check if the robot uses cross monitor safety
  bool get_use_safety_speed_control();        // check if the robot uses speed control safety
  bool get_use_safety_obstacle_detect();        // check if the robot uses obstacle detection safety
  bool get_use_safety_bumper();           // check if the robot uses bumper safety
  bool get_use_safety_interlock();          // check if the robot uses interlock safety

  /***********************
   * mileage
   ***********************/
  double get_mileage();               // get robot mileage
  void set_mileage(const QString &mileage);     // set robot mileage

  /***********************
   * missing variables check
   ***********************/
  QStringList get_missing_variables();        // get list of missing variables
  bool has_missing_variables();           // check if there are missing variables
  void show_missing_variables_dialog();       // show missing variables dialog

  /***********************
   * QA
   ***********************/
  double get_qa_step_distance();            // get QA step distance
  double get_qa_stop_min_distance();          // get QA stop minimum distance

  /***********************
   * auto update config
   ***********************/
  void set_update_config_file();            // update config file
  bool set_backup_config_file();            // backup config file
  void set_restore_config_file_backup();        // restore config file from backup
  void set_default_config_template();         // set default config template
  void cleanup_old_backups(const QFileInfo& config_info, const QDir& config_dir, int max_backups);  // cleanup old backup files

  /***********************
   * update
   ***********************/
  bool get_update_use_config();           // check if the robot uses config update

private:
  /***********************
   * loading by category
   ***********************/
  void load_robot_config(const QJsonObject& obj);
  void load_sensors_config(const QJsonObject& obj);
  void load_localization_config(const QJsonObject& obj);
  void load_network_config(const QJsonObject& obj);
  void load_debug_config(const QJsonObject& obj);
  void load_logging_config(const QJsonObject& obj);
  void load_motor_config(const QJsonObject& obj);
  void load_localization_2d_config(const QJsonObject& obj);
  void load_localization_3d_config(const QJsonObject& obj);
  void load_mapping_config(const QJsonObject& obj);
  void load_obstacle_config(const QJsonObject& obj);
  void load_control_config(const QJsonObject& obj);
  void load_docking_config(const QJsonObject& obj);
  void load_map_config(const QJsonObject& obj);
  void load_lidar_configs(const QJsonObject& obj);
  void load_camera_configs(const QJsonObject& obj);
  void load_sensor_specific_configs(const QJsonObject& obj);
  void load_safety_config(const QJsonObject& obj);
  void load_qa_config(const QJsonObject& obj);
  void load_update_config(const QJsonObject& obj);

  /***********************
   * helper functions for loading json
   ***********************/
  void check_and_set_string(const QJsonObject& obj, const QString& key, QString& target, const QString& section);
  void check_and_set_bool(const QJsonObject& obj, const QString& key, bool& target, const QString& section);
  void check_and_set_int(const QJsonObject& obj, const QString& key, int& target, const QString& section);
  void check_and_set_double(const QJsonObject& obj, const QString& key, double& target, const QString& section);

  void add_missing_variable(const QString& section, const QString& variable);

  /***********************
   * default config objects
   ***********************/
  QJsonObject set_default_config_object();
  QJsonObject merge_config_objects(const QJsonObject& existing, const QJsonObject& defaults);

  QJsonObject set_default_robot_config();
  QJsonObject set_default_sensors_config();
  QJsonObject set_default_localization_config();
  QJsonObject set_default_network_config();
  QJsonObject set_default_debug_config();
  QJsonObject set_default_logging_config();
  QJsonObject set_default_motor_config();
  QJsonObject set_default_localization_2d_config();
  QJsonObject set_default_localization_3d_config();
  QJsonObject set_default_mapping_config();
  QJsonObject set_default_obstacle_config();
  QJsonObject set_default_control_config();
  QJsonObject set_default_docking_config();
  QJsonObject set_default_safety_config();
  QJsonObject set_default_qa_config();
  QJsonObject set_default_update_config();

  QJsonObject set_default_sick_config();
  QJsonObject set_default_laki_config();
  QJsonObject set_default_rplidar_config();
  QJsonObject set_default_livox_config();
  QJsonObject set_default_orbbec_config();

private:
  explicit CONFIG(QObject *parent = nullptr);
  ~CONFIG();

  // mutex
  std::shared_mutex mtx;
  QString path_config;
  QString path_version;
  QString path_cam_serial_number;
  QString common_path;

  /* unit : meter, degree, second
   * params (initial value ref from AMR200)
   */

  // robot
  QString ROBOT_SERIAL_NUMBER = "RB-M-";
  RobotModel ROBOT_MODEL = RobotModel::NONE;
  RobotType ROBOT_TYPE  = RobotType::NONE;
  QString PLATFORM_TYPE = "";
  QString MILEAGE = "";
  QString ROBOT_WHEEL_TYPE = "UNKNOWN";
  double ROBOT_SIZE_X[2] = {-0.35, 0.35}; // min, max
  double ROBOT_SIZE_Y[2] = {-0.35, 0.35};
  double ROBOT_SIZE_Z[2] = {0.0, 0.22};
  double ROBOT_SIZE_ADD_X = 0.0;  
  double ROBOT_SIZE_ADD_Y = 0.0;  
  double ROBOT_SIZE_ADD_Z = 0.0;  
  double ROBOT_WHEEL_BASE = 0.387;
  double ROBOT_WHEEL_RADIUS = 0.0635;
  double ROBOT_RADIUS = 0.5;
  bool USE_SPEAKER = false;
  double ROBOT_LX = 0.5;
  double ROBOT_LY = 0.23;
  double ROBOT_ALARM_BAT_LOW = 60.0;      // battery low alarm threshold (%)
  double ROBOT_ALARM_BAT_CRITICAL = 40.0;   // battery critical alarm threshold (%)
  bool USE_ROBOT_ALARM = true;

  bool USE_BQR = false;
  bool USE_IMU = false;

  // localization config
  QString LOC_MODE = "3D";   // "2D" or "3D"
  bool USE_ARUCO = false;
  bool USE_EKF = false;

  // Networking
  bool USE_MULTI     = false;
  bool USE_COMM_RTSP = false;
  bool USE_COMM_MSA  = false;
  bool USE_COMM_FMS  = false;

  // debug
  bool USE_SIM = false;
  bool USE_BEEP = false;
  QString SERVER_IP = "127.0.0.1";
  QString SERVER_ID = "rainbow";
  QString SERVER_PW = "rainbow";

  // logging
  QString LOG_LEVEL = "info";         // spdlog level: trace, debug, info, warn, error, critical, off
  bool DEBUG_LIDAR_2D = false;        // enable/disable LIDAR debug logs
  bool DEBUG_LIDAR_3D = false;
  bool DEBUG_MOBILE =  false;         // enable/disable MOBILE debug logs
  bool DEBUG_COMM_RRS = false;        // enable/disable COMM RRS debug logs
  bool DEBUG_AUTOCONTROL = false;
  bool DEBUG_LOCALIZATION = false;
  bool DEBUG_OBSMAP = false;

  bool LOG_ENABLE_FILE_OUTPUT = false;    // enable/disable file output
  QString LOG_FILE_PATH = "logs/app.log";   // log file path

  // lidar bottom
  int   LIDAR_BOTTOM_NUM = 1;
  bool  USE_LIDAR_BOTTOM = false;
  double  LIDAR_BOTTOM_MIN_RANGE = 1.0;
  double  LIDAR_BOTTOM_MAX_RANGE = 40.0;
  QString LIDAR_BOTTOM_TYPE = "";
  QString LIDAR_BOTTOM_IP[2]  = { "", "" };
  QString LIDAR_BOTTOM_TF[2]  = { "0,0,0,0,0,0", "0,0,0,0,0,0" };
  QString LIDAR_BOTTOM_DEV[2] = { "0,0,0,0,0,0", "0,0,0,0,0,0" };

  // lidar 2d
  int LIDAR_2D_NUM = 1;
  bool USE_LIDAR_2D = false;
  bool USE_LIDAR_2D_OBSTACLE = true;
  QString LIDAR_2D_TYPE = "";
  double LIDAR_2D_MIN_RANGE = 1.0;
  double LIDAR_2D_MAX_RANGE = 40.0;
  QString LIDAR_2D_IP[2] = { "", "" };
  QString LIDAR_2D_TF[2] = { "0,0,0,0,0,0", "0,0,0,0,0,0" };
  QString LIDAR_2D_DEV[2] = { "0,0,0,0,0,0", "0,0,0,0,0,0" };

  // lidar 3d
  bool USE_LIDAR_3D = false;
  bool USE_LIDAR_3D_OBSTACLE = true;
  QString LIDAR_3D_TYPE = "";
  int LIDAR_3D_NUM = 1;
  double LIDAR_3D_MIN_RANGE = 1.0;
  double LIDAR_3D_MAX_RANGE = 70.0;
  QString LIDAR_3D_IP[2] = { "", "" };
  QString LIDAR_3D_TF[2] = { "0,0,0,0,0,0", "0,0,0,0,0,0" };

  // bottom lidar
  bool USE_BLIDAR = false;

  // cam
  int   CAM_NUM = 2;
  int   CAM_COLOR_PROFILE[max_cam_cnt] = {0};
  int   CAM_DEPTH_PROFILE[max_cam_cnt] = {0};
  double  CAM_HEIGHT_MIN = 0.1; // for rgbd-cam cloud cropping
  double  CAM_HEIGHT_MAX = 1.0;
  bool  USE_CAM = false;
  bool  USE_CAM_OBSTACLE = true;
  bool  USE_CAM_RGB = false;
  bool  USE_CAM_DEPTH = false;
  bool  USE_CAM_FILTER = false;
  double  CAM_FILTER_ROR_RADIUS = 0.1;
  int   CAM_FILTER_ROR_MIN_NEIGHBORS = 5;
  int   CAM_FILTER_CLUSTER_MIN_SIZE = 20;
  QString CAM_TYPE = "";
  QString CAM_SERIAL_NUMBER[max_cam_cnt];
  QString CAM_TF[max_cam_cnt];

  // motor
  int MOTOR_ID_L = 1;
  int MOTOR_ID_R = 0;
  double MOTOR_DIR = 1.0;
  double MOTOR_GEAR_RATIO = 3.0;
  double MOTOR_LIMIT_V = 2.0;
  double MOTOR_LIMIT_V_ACC = 1.0;
  double MOTOR_LIMIT_W = 180.0;
  double MOTOR_LIMIT_W_ACC = 180.0;
  double MOTOR_GAIN_KP = 4400.0;
  double MOTOR_GAIN_KI = 0.0;
  double MOTOR_GAIN_KD = 100.0;
  double MOTOR_SAFETY_LIMIT_V = 2.1;
  double MOTOR_SAFETY_LIMIT_W = 360.0;

  // mapping
  int MAPPING_ICP_MAX_FEATURE_NUM = 1000;
  int MAPPING_ICP_DO_ERASE_GAP = 10;
  int MAPPING_ICP_DO_ACCUM_NUM = 2;
  int MAPPING_KFRM_UPDATE_NUM = 50;
  int MAPPING_WINDOW_SIZE = 100;
  double MAPPING_ICP_COST_THRESHOLD = 0.5;
  double MAPPING_ICP_ERROR_THRESHOLD = 0.2;
  double MAPPING_ICP_VIEW_THRESHOLD = 170.0;
  double MAPPING_KFRM_LC_TRY_DIST = 3.0;
  double MAPPING_KFRM_LC_TRY_OVERLAP = 0.25;
  double MAPPING_VOXEL_SIZE = 0.05;

  // localization 2d
  int LOC_2D_ICP_MAX_FEATURE_NUM = 1000;
  int LOC_2D_SURFEL_NUM = 3;
  double LOC_2D_ICP_ODO_FUSION_RATIO = 0.8;
  double LOC_2D_ICP_COST_THRESHOLD_0 = 1.0;
  double LOC_2D_ICP_COST_THRESHOLD = 0.3;
  double LOC_2D_ICP_ERROR_THRESHOLD = 0.2;
  double LOC_2D_ARUCO_ODO_FUSION_RATIO = 0.8;
  double LOC_2D_ARUCO_ODO_FUSION_DIST = 2.0;
  double LOC_2D_SURFEL_RANGE = 0.15;
  double LOC_2D_CHECK_DIST = 0.3;
  double LOC_2D_CHECK_IE = 0.2;
  double LOC_2D_CHECK_IR = 0.3;
  bool LOC_2D_USE_ROTATION_FALLBACK = false;
  double LOC_2D_ROTATION_FALLBACK_STEP = 10.0;
  double LOC_2D_ROTATION_FALLBACK_RANGE = 180.0;

  // localization 3d
  int LOC_MAX_FEATURE_NUM = 500;
  int LOC_SURFEL_NN_NUM = 1;
  double LOC_SURFEL_BALANCE = 0.4; // 0.35 ~ 1.0(off)
  double LOC_COST_THRESHOLD = 1.0;
  double LOC_INLIER_CHECK_DIST = 0.3;

  // obstacle map
  int OBS_AVOID = 0;
  double OBS_DEADZONE = 0.7;
  double OBS_LOCAL_GOAL_D = 4.0;
  double OBS_SAFE_MARGIN_X = 0.1;
  double OBS_SAFE_MARGIN_Y = 0.1;
  double OBS_PATH_MARGIN_X = 0.05;
  double OBS_PATH_MARGIN_Y = 0.0;
  double OBS_MAP_GRID_SIZE = 0.05;
  double OBS_MAP_RANGE = 5.0;
  double OBS_MAP_MIN_V = 0.3;
  double OBS_MAP_MIN_Z = -1.0;
  double OBS_MAP_MAX_Z = 1.0;
  double OBS_PREDICT_TIME = 3.0;
  double OBS_DISTANCE_LED_NEAR = 1.0;
  double OBS_DISTANCE_LED_FAR = 1.5;

  // control
  double DRIVE_GOAL_APPROACH_GAIN = 1.0;
  double DRIVE_GOAL_D = 0.05;
  double DRIVE_GOAL_TH = 2.0;
  double DRIVE_EXTENDED_CONTROL_TIME = 1.0;
  double DRIVE_V_DEADZONE = 0.02;
  double DRIVE_W_DEADZONE = 0.1;

  // docking
  int DOCKING_RETRY_COUNT = 3;
  double DOCKING_CHG_LENGTH = 0.472;
  double DOCKING_GOAL_D = 0.03; //3cm
  double DOCKING_GOAL_TH = 0.01745; //0.1*D2R;
  double DOCKING_KP_d = 0.15;
  double DOCKING_KD_d = 0.1;
  double DOCKING_KP_th = 0.35;
  double DOCKING_KD_th = 0.15;
  double DOCKING_CLUST_D_THRESHOLD = 0.015;
  double DOCKING_FIND_VMARK_DIST_THRESHOLD_MIN = 1.0;
  double DOCKING_FIND_VMARK_DIST_THRESHOLD_MAX = 2.5;
  double DOCKING_CLUST_DIST_THRESHOLD_MIN = 0.4;
  double DOCKING_CLUST_DIST_THRESHOLD_MAX = 2.5;
  double DOCKING_CLUST_ANGLE_THRESHOLD = 45.0*D2R;
  double DOCKING_DOCK_SIZE_X[2] = {-0.025, 0.025};
  double DOCKING_POINTDOCK_MARGIN = 0.18;
  double DOCKING_ICP_ERR_THRESHOLD = 0.003;
  int DOCKING_TYPE = 0; // 0: L_dock 1: FQR_dock 2: bqr
  bool DOCKING_REVERSE_MODE = false;
  double DOCKING_DWA_YAW_WEIGHT = 0.25;
  double DOCKING_CHECK_MOTOR_A = 120.0;
  double DOCKING_WAITING_TIME = 10.0; //second
  double DOCKING_FINAL_ICP_ERR_THRESHOLD = 0.1;
  double DOCKING_X_OFFSET = 0.0;
  double DOCKING_Y_OFFSET = 0.0;
  double DOCKING_LINEAR_X_OFFSET = -0.035;
  bool DOCKING_REVERSE_FLAG = false;
  int DOCKING_FIELD = 0;
  double XNERGY_SET_CURRENT = 8.0;
  QString CHARGE_TYPE = "RAINBOW";
  QString LIDAR_TYPE = "SICK";

  // safety
  bool USE_MONITORING_FIELD = false;
  int MONITORING_FIELD_NUM = 0;
  double MONITORING_TRAJ_DT = 1.0;
  double MONITORING_PREDICT_T = 5.0;
  std::vector<double> FIELD_SIZE_MIN_X;
  std::vector<double> FIELD_SIZE_MAX_X;
  std::vector<double> FIELD_SIZE_MIN_Y;
  std::vector<double> FIELD_SIZE_MAX_Y;
  bool USE_SAFETY_CROSS_MONITOR = false;
  bool USE_SAFETY_SPEED_CONTROL = false;
  bool USE_SAFETY_OBSTACLE_DETECT = false;
  bool USE_SAFETY_BUMPER = false;
  bool USE_SAFETY_INTERLOCK = false;

  // QA
  double QA_STEP_DISTANCE = 0.7;
  double QA_STOP_MIN_DISTANCE = 0.3;

  // update 
  bool USE_CONFIG_UPDATE = false;

  QStringList load_folder_list();

  // map
  QString MAP_PATH = "";

  // version info
  QString VERSION_INFO;

  // missing variables tracking
  QStringList missing_variables;

  // flag
  std::atomic<bool> is_load = {false};  

};

#endif // CONFIG_H
