#ifndef SLAMNAV2_SLAMNAV_COMMON_TYPES_H
#define SLAMNAV2_SLAMNAV_COMMON_TYPES_H

// OpenMP
#include <omp.h>

// linux native
#include <sys/timerfd.h>

// stl
#include <stdio.h>
#include <vector>
#include <memory>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <streambuf>
#include <random>
#include <variant>

// tbb
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>

// pcl
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/geometry/mesh_base.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include "cv_to_qt.h"

// octomap
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

// pseudo color
// #include "tinycolormap.hpp"

// spdlog
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

// std::error code
#include <system_error>

// sophus
#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>
#include <sophus/interpolate.hpp>

// qt
#include <QApplication>
#include <QStandardPaths>
#include <QDateTime>
#include <QDir>
#include <QFileDialog>
#include <QFileInfo>
#include <QTextStream>
#include <QInputDialog>
#include <QJsonDocument>
#include <QJsonValue>
#include <QJsonArray>
#include <QJsonObject>
#include <QTableWidget>
#include <QHeaderView>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
Q_DECLARE_METATYPE(Eigen::Matrix4d)
Q_DECLARE_METATYPE(std::vector<int>)

// defines
constexpr double ACC_G  = 9.80665;
constexpr double N2S  = 1.0e-9; // nanosec to sec
constexpr double S2N  = 1.0e9; // sec to nanosec
constexpr double U2S  = 1.0e-6; // microsec to sec
constexpr double S2U  = 1.0e6; // sec to microsec
constexpr double M2S  = 1.0e-3; // millisec to sec
constexpr double S2M  = 1.0e3; // sec to millisec
constexpr double D2R  = M_PI/180.0;
constexpr double R2D  = 180.0/M_PI;

#define toWrap(rad) (std::atan2(std::sin(rad), std::cos(rad)))
#define deltaRad(ed,st) (std::atan2(std::sin(ed - st), std::cos(ed - st)))

#define MO_STORAGE_NUM 300

#define S100_BAT_MAX_VOLTAGE 53.5
#define S100_BAT_MIN_VOLTAGE 42.0

#define LOG_MODULE_NAME(name) namespace { const char* MODULE_NAME = name; }

// enumulator
enum class RobotType
{
  S100_A,
  S100_B,
  S100_A_3D,
  S100_B_3D,
  D400,
  QD,
  MECANUM_Q150,
  MECANUM_VALEO,
  SEM,
  SDC,
  SEC_CORE,
  SEC_EE,
  NONE
};

enum class RobotModel
{
  S100,
  D400,
  QD,
  MECANUM,
  DD,
  SEM,
  SDC,
  SEC,
  NONE
};

enum FOOT_STATE
{
  FOOT_STATE_IDLE = 0,     // 정지 상태
  FOOT_STATE_INIT,      // 시작 준비 or 초기화
  FOOT_STATE_MOVING ,     // 위로 이동 동작 수행 중
  FOOT_STATE_EMO_STOP ,   // 비상정지
  FOOT_STATE_DONE    // 이동 동작 완료
};

enum OPEARATION_MODE_STATE
{
  MOBILE_POWER_OFF    = 0,
  MOBILE_MAIN_POWER_UP  = 1,
  MOBILE_PC_POWER_UP    = 2,
  MOBILE_ROBOT_POWER_OFF  = 3,
  MOBILE_ROBOT_INITIALIZE = 4,
  MOBILE_NORMAL_OP    = 5,
  MOBILE_NORMAL_OP_AUTO = 6,
  MOBILE_NORMAL_OP_MANUAL = 7,
  MOBILE_NORMAL_LOW_BAT = 8,
  MOBILE_OPERATIONL_STOP  = 9,
  MOBILE_CHARGING     = 10,
  MOBILE_CONFIGURATION  = 11,
};

enum ROBOT_INITIALZATION_STATE
{
  MOBILE_RI_IDLE      = 0,
  MOBILE_RI_SAFETY_CHECK  = 1,
  MOBILE_RI_POWER_ON    = 2,
  MOBILE_RI_POWER_CHECK = 3,
  MOBILE_RI_MOTOR_INIT  = 4,
  MOBILE_RI_MOTOR_CHECK = 5,
  MOBILE_RI_DONE      = 6,
  MOBILE_RI_FAIL      = 7,
};

enum DOCKING_CHARGE_STATE
{
  CHARGE_STATE_IDLE=0,
  CHARGE_STATE_TRIG_TO_CHARGE,
  CHARGE_STATE_BATTERY_ON,
  CHARGE_STATE_CHARGING,
  CHARGE_STATE_TRIG_TO_STOP_CHARGE,
  CHARGE_STATE_FAIL,
};

enum AUTO_FSM_STATE
{
  AUTO_FSM_FIRST_ALIGN = 0,
  AUTO_FSM_DRIVING,
  AUTO_FSM_FINAL_ALIGN,
  AUTO_FSM_OBS,
  AUTO_FSM_COMPLETE,    
  AUTO_FSM_DOCKING,
  AUTO_FSM_PAUSE,    
};

enum LED_STATE
{
  LED_OFF = 0,
  LED_RED,
  LED_GREEN,
  LED_WHITE,
  LED_BLUE,
  LED_YELLOW,
  LED_MAGENTA,
  LED_RED_BLINK,
  LED_GREEN_BLINK,
  LED_WHITE_BLINK,
  LED_BLUE_BLINK,
  LED_YELLOW_BLINK,
  LED_RIGHT_YELLOW_BLINK,
  LED_LEFT_YELLOW_BLINK,
  LED_MAGENTA_BLINK,
};


enum SAFETY_LED_STATE
{
  SAFETY_LED_OFF = 0,
  SAFETY_LED_RED,
  SAFETY_LED_GREEN_BLINKING,
  SAFETY_LED_CYAN,
  SAFETY_LED_PURPLE,
  SAFETY_LED_PURPLE_BLINKING,
  SAFETY_LED_YELLOW,
  SAFETY_LED_YELLOW_WAVERING,
  SAFETY_LED_YELLOW_FADE,
  SAFETY_LED_YELLOW_BLINKING,
  SAFETY_LED_PURPLE_YELLOW,
  SAFETY_LED_BLUE,
  SAFETY_LED_BLUE_WAVERING,
  SAFETY_LED_CONTRACTING_GREEN,
  SAFETY_LED_WHITE_WAVERING,
  SAFETY_LED_WHITE
};

enum DOCK_FSM_STATE
{
  DOCKING_FSM_OFF = 0,
  DOCKING_FSM_POINTDOCK,
  DOCKING_FSM_COMPENSATE,
  DOCKING_FSM_YCOMPENSATE,
  DOCKING_FSM_DOCK,
  DOCKING_FSM_UNDOCK,
  DOCKING_FSM_OBS,
  DOCKING_FSM_WAIT,
  DOCKING_FSM_COMPLETE,
  DOCKING_FSM_FAILED,
  DOCKING_FSM_POINTDOCK_FOR_CHRGE,
  DOCKING_FSM_CHKCHARGE,
  DOCKING_FSM_UNDOCK_FAILED,
  DOCKING_FSM_UNDOCK_OFF
};

enum AUTO_OBS_STATE
{
  AUTO_OBS_CHECK = 0,
  AUTO_OBS_RECOVERY,
  AUTO_OBS_AVOID,
  AUTO_OBS_WAIT,
  AUTO_OBS_WAIT2,
  AUTO_OBS_VIR
};

enum OBS_STATE
{
  OBS_NONE = 0,
  OBS_DYN  = 1,
  OBS_VIR  = 2,
};

enum LOCAL_PATH_STATE
{
  LOCAL_PATH_IDLE = 0,
  LOCAL_PATH_REQUEST,
  LOCAL_PATH_BUSY,
};

enum TASK_FSM_STATE
{
  TASK_IDLE = 0,
  TASK_MOVE,
  TASK_CHECK_MOVE,
  TASK_PROGRESS,
  TASK_CHECK_PROGRESS,
  TASK_WAIT,
  TASK_PAUSED,
  TASK_COMPLETE,
};

enum CHECK_DRIVING_STATE
{
  DRIVING_FAILED    = 0,
  DRIVING_NOT_READY = 1,
  DRIVING_FINE    = 2,
  DRIVING_MANUAL    = 3,
};

enum MOTOR_ERROR_STATE
{
  MOTOR_ERR_MOD = 2,
  MOTOR_ERR_JAM = 4,
  MOTOR_ERR_CUR = 8,
  MOTOR_ERR_BIG = 16,
  MOTOR_ERR_IN  = 32,
  MOTOR_ERR_PSI = 64,
  MOTOR_ERR_NON = 128,
};

enum ROBOT_OPERATING_MODE
{
  ROBOT_AUTO_MODE   =0,
  ROBOT_JOYSTICK_MODE =1,
};

enum ROBOT_OPERATION_STATE
{
  //for safety
  SM_OM_POWER_OFF     = 0,
  SM_OM_MAIN_POWER_UP   = 1,
  SM_OM_PC_POWER_UP   = 2,
  SM_OM_ROBOT_POWER_OFF = 3,
  SM_OM_ROBOT_INITIALIZE  = 4,
  SM_OM_NORMAL_OP     = 5,
  SM_OM_NORMAL_OP_AUTO  = 6,
  SM_OM_NORMAL_OP_MANUAL  = 7,
  SM_OM_NORMAL_LOW_BAT  = 8,
  SM_OM_OPERATIONAL_STOP  = 9,
  SM_OM_CHARGING      = 10,
  SM_OM_CONFIGURATION   = 11,
};

enum CHARGING_STATION_STATE
{
  CHARGING_STATION_IDLE         = 0,
  CHARGING_STATION_TRIG_TO_CHARGE     = 1,
  CHARGING_STATION_BATTERY_ON       = 2,
  CHARGING_STATION_CHARGING       = 3,
  CHARGING_STATION_CHARGE_FINISH  = 4,
  CHARGING_STATION_FAIL         = 5,
};

enum SEMIAUTOINIT_STATE
{
  NONE     = 0,
  INITIALIZING = 1,
  COMPLETE   = 2
};

enum MAP_LOAD_STATE
{
  MAP_NOT_LOADED = 0,
  MAP_LOADING = 1,
  MAP_LOADED = 2,
};

enum PDU_MOVE_STATE
{
  PDU_MOVE_NONE=0,
  PDU_MOVE_LINEAR_X=1,
  PDU_MOVE_LINEAR_Y=2,
  PDU_MOVE_ROTATE=3
};

// enum class
enum class StateMultiReq
{
  NO_CHANGE,
  NONE,
  RECV_PATH,
  REQ_PATH
};

enum class StateObsCondition
{
  NO_CHANGE,
  NONE,
  FAR,
  NEAR,
  VIR
};

enum class StateCurGoal
{
  NO_CHANGE,
  NONE,
  CANCEL,
  MOVE,
  FAIL,
  COMPLETE,
  OBSTACLE
};

enum class TypeNode
{
  GOAL,
  ROUTE,
  VIRTUAL_WALL,
  ZONE
};

enum class AttributeGoal
{
  NONE,
  FORWARD,
  BACKWARD,
  OFFSET,
  INIT,
  STATION
};

enum class AttributeRoute
{
  NONE
};

enum class AttributeVirtualWall
{
  NONE
};

enum class AttributeZone
{
  NONE,
  FORBIDDEN,
  SWITCHABLE_FORBIDDEN,
  SPEED,
  SENSOR_MUTE,
  SOUND,
  LIGHT,
  AVOID,
  IGNORE_LIDAR1,
  IGNORE_LIDAR2,
  OFFSET,
  MASK
};

enum class DriveDir
{
  FORWARD,
  REVERSE
};

struct LOG_INFO
{
  QString user_log = "";
  QString color_code = "";
  bool is_time = true;
  bool is_hide = false;

  LOG_INFO()
  {
    user_log = "";
    color_code = "";
    is_time = true;
    is_hide = false;
  }

  LOG_INFO(const LOG_INFO& p)
  {
    user_log = p.user_log;
    color_code = p.color_code;
    is_time = p.is_time;
    is_hide = p.is_hide;
  }

  LOG_INFO& operator=(const LOG_INFO& p)
  {
    user_log = p.user_log;
    color_code = p.color_code;
    is_time = p.is_time;
    is_hide = p.is_hide;
    return *this;
  }
};

// topomap node
struct NODE_ROLE
{
  bool conveyor;
  bool warning_beep;
  bool ignore_2d;
  bool ignore_3d;
  bool ignore_cam;
  bool ignore_obs_2d;
  bool ignore_obs_3d;
  bool ignore_obs_cam;

  NODE_ROLE()
  {
    conveyor     = false;
    warning_beep   = false;
    ignore_2d    = false;
    ignore_3d    = false;
    ignore_cam     = false;
    ignore_obs_2d  = false;
    ignore_obs_3d  = false;
    ignore_obs_cam = false;
  }

  NODE_ROLE(const NODE_ROLE& p)
  {
    conveyor     = p.conveyor;
    warning_beep   = p.warning_beep;
    ignore_2d    = p.ignore_2d;
    ignore_3d    = p.ignore_3d;
    ignore_cam     = p.ignore_cam;
    ignore_obs_2d  = p.ignore_obs_2d;
    ignore_obs_3d  = p.ignore_obs_3d;
    ignore_obs_cam = p.ignore_obs_cam;
  }

  NODE_ROLE& operator=(const NODE_ROLE& p)
  {
    conveyor     = p.conveyor;
    warning_beep   = p.warning_beep;
    ignore_2d    = p.ignore_2d;
    ignore_3d    = p.ignore_3d;
    ignore_cam     = p.ignore_cam;
    ignore_obs_2d  = p.ignore_obs_2d;
    ignore_obs_3d  = p.ignore_obs_3d;
    ignore_obs_cam = p.ignore_obs_cam;

    return *this;
  }
};

struct NODE
{
  QString id;
  QString name;
  QString type;         // ROUTE, GOAL, OBS, ZONE
  Eigen::Vector3d size;
  QString context;        // additional info

  Eigen::Matrix4d tf;       // node tf

  NODE_ROLE role;

  NODE()
  {
    id = "";
    name = "";
    type = "";
    size.setZero();

    context = "";

    tf.setIdentity();
  }

  NODE(const NODE& p)
  {
    id = p.id;
    name = p.name;
    type = p.type;
    size = p.size;
    context = p.context;

    tf = p.tf;

    role = p.role;
  }

  NODE& operator=(const NODE& p)
  {
    id = p.id;
    name = p.name;
    type = p.type;
    size = p.size;
    context = p.context;

    tf = p.tf;

    role = p.role;

    return *this;
  }

  bool operator==(const NODE& p)
  {
    if(id == p.id)
    {
      return true;
    }
    return false;
  }
};

struct LINK
{
  QString st_id;
  QString ed_id;

  QString dir;
  QString method;
  double speed = 0.0;

  LINK()
  {

  }

  LINK(const LINK& p)
  {
    st_id = p.st_id;
    ed_id = p.ed_id;

    dir = p.dir;
    method = p.method;
    speed = p.speed;
  }

  LINK& operator=(const LINK& p)
  {
    st_id = p.st_id;
    ed_id = p.ed_id;

    dir = p.dir;
    method = p.method;
    speed = p.speed;

    return *this;
  }
};

// autocontrol parameters
struct CTRL_PARAM
{
  double ST_V      =  0.05;
  double ED_V      =  0.05;

  double LIMIT_V     =  1.0;
  double LIMIT_W     =  30.0;
  double LIMIT_V_ACC   =  0.3;
  double LIMIT_V_DCC   =  0.3;
  double LIMIT_W_ACC   =  180.0;
  double LIMIT_PIVOT_W =  30.0;

  double DRIVE_T     =  0.0;
  double DRIVE_H     =  4.0;
  double DRIVE_A     =  0.9;
  double DRIVE_B     =  0.03;
  double DRIVE_L     =  0.4;
  double DRIVE_K     =  1.0;
  double DRIVE_EPS   =  0.3;
};

// dockcontrol parameters
struct DCTRL_PARAM
{
  double GAIN_P    = 1.0;
  double GAIN_D    = 0.0;
  double LIMIT_V     = 0.5;
  double LIMIT_W     = 30.0;
  double LIMIT_V_ACC = 0.5;
  double LIMIT_W_ACC = 180.0;
};

struct ASTAR_NODE
{
  ASTAR_NODE* parent = nullptr;
  NODE* node = nullptr;
  Eigen::Matrix4d tf;
  double g = 0;
  double h = 0;
  double f = 0;

  ASTAR_NODE()
  {
    parent = nullptr;
    node = nullptr;
    tf.setIdentity();
    g = 0;
    h = 0;
    f = 0;
  }

  ASTAR_NODE(const ASTAR_NODE& p)
  {
    parent = p.parent;
    node = p.node;
    tf = p.tf;
    g = p.g;
    h = p.h;
    f = p.f;
  }

  ASTAR_NODE& operator=(const ASTAR_NODE& p)
  {
    parent = p.parent;
    node = p.node;
    tf = p.tf;
    g = p.g;
    h = p.h;
    f = p.f;
    return *this;
  }
};

struct HASTAR_NODE
{
  HASTAR_NODE* parent = nullptr;
  NODE* node = nullptr;
  Eigen::Matrix4d tf;
  double g = 0;
  double h = 0;
  double f = 0;
  int prim = 0;

  HASTAR_NODE()
  {
    parent = nullptr;
    node = nullptr;
    tf.setIdentity();
    g = 0;
    h = 0;
    f = 0;
    prim = 0;
  }

  HASTAR_NODE(const HASTAR_NODE& p)
  {
    parent = p.parent;
    node = p.node;
    tf = p.tf;
    g = p.g;
    h = p.h;
    f = p.f;
    prim = p.prim;
  }

  HASTAR_NODE& operator=(const HASTAR_NODE& p)
  {
    parent = p.parent;
    node = p.node;
    tf = p.tf;
    g = p.g;
    h = p.h;
    f = p.f;
    prim = p.prim;
    return *this;
  }
};

struct PATH
{
  double t;
  bool is_final;
  Eigen::Matrix4d ed_tf;
  std::vector<QString> node;
  std::vector<Eigen::Matrix4d> pose;
  std::vector<Eigen::Vector3d> pos;  
  std::vector<double> ref_v;
  DriveDir drive_dir;
  QString drive_method;

  PATH()
  {
    t = 0;
    ed_tf.setIdentity();
    is_final = false;
    drive_dir = DriveDir::FORWARD;
  }

  PATH(const PATH& p)
  {
    t = p.t;
    node = p.node;
    pose = p.pose;
    pos = p.pos;        
    ref_v = p.ref_v;    
    ed_tf = p.ed_tf;
    is_final = p.is_final;
    drive_dir = p.drive_dir;
    drive_method = p.drive_method;
  }

  PATH& operator=(const PATH& p)
  {
    t = p.t;
    node = p.node;
    pose = p.pose;
    pos = p.pos;    
    ref_v = p.ref_v;    
    ed_tf = p.ed_tf;
    is_final = p.is_final;
    drive_dir = p.drive_dir;
    drive_method = p.drive_method;
    return *this;
  }

  bool operator==(const PATH& p) const
  {
    if(pose.size() != p.pose.size())
    {
      return false;
    }

    for(size_t i = 0; i < pose.size(); i++)
    {
      if(!pose[i].isApprox(p.pose[i]))
      {
        return false;
      }
    }

    return true;
  }

  bool operator!=(const PATH& p) const
  {
    return !(*this == p);
  }
};

struct MOVE_INFO
{
  int preset = 0;
  double x = 0;
  double y = 0;
  double z = 0;
  double rz = 0;
  QString method = "";
  QString command = "";
  QString node_id = "";

  double docking_offset_x = 0;
  double docking_offset_y = 0;
  double docking_offset_th = 0;

  MOVE_INFO()
  {
    command = "";
    x = 0;
    y = 0;
    z = 0;
    rz = 0;
    node_id = "";
    preset = 0;
    method = "";
    docking_offset_x = 0;
    docking_offset_y = 0;
    docking_offset_th = 0;
  }

  MOVE_INFO(const MOVE_INFO& p)
  {
    command = p.command;
    x = p.x;
    y = p.y;
    z = p.z;
    rz = p.rz;
    node_id = p.node_id;
    preset = p.preset;
    method = p.method;
    docking_offset_x = p.docking_offset_x;
    docking_offset_y = p.docking_offset_y;
    docking_offset_th = p.docking_offset_th;
  }

  MOVE_INFO& operator=(const MOVE_INFO& p)
  {
    command = p.command;
    x = p.x;
    y = p.y;
    z = p.z;
    rz = p.rz;
    node_id = p.node_id;
    preset = p.preset;
    method = p.method;
    docking_offset_x = p.docking_offset_x;
    docking_offset_y = p.docking_offset_y;
    docking_offset_th = p.docking_offset_th;
    return *this;
  }
};




#endif // SLAMNAV2_SLAMNAV_COMMON_TYPES_H
