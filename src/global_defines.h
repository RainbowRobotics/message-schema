#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H

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
#include <iostream>
#include <streambuf>
#include <random>
#include <variant>

// tbb
#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_vector.h>

// vtk
#include <QVTKOpenGLNativeWidget.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkDataObjectToTable.h>
#include <vtkElevationFilter.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkQtTableView.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkVersion.h>
#include <vtkWindowToImageFilter.h>

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

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
Q_DECLARE_METATYPE(Eigen::Matrix4d)

// sophus
#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>
#include <sophus/interpolate.hpp>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include "cv_to_qt.h"

// nanoflann
#include "nanoflann.hpp"

// octomap
#include <octomap/octomap.h>
#include <octomap/math/Utils.h>

// pseudo color
#include "tinycolormap.hpp"

// pdal
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <pdal/io/LasReader.hpp>
#include <pdal/StageFactory.hpp>

// spdlog
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

// std::error code
#include <system_error>

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
#include <QtConcurrent/QtConcurrent>

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
    NONE
};

enum class RobotModel
{
    S100,
    D400,
    QD,
    MECANUM,
    SEM,
    SDC,
    NONE
};

enum FOOT_STATE
{
    FOOT_STATE_IDLE = 0,         // 정지 상태
    FOOT_STATE_INIT,            // 시작 준비 or 초기화
    FOOT_STATE_MOVING ,       // 위로 이동 동작 수행 중
    FOOT_STATE_EMO_STOP ,       // 비상정지
    FOOT_STATE_DONE      // 이동 동작 완료
};

enum OPEARATION_MODE_STATE
{
    MOBILE_POWER_OFF        = 0,
    MOBILE_MAIN_POWER_UP    = 1,
    MOBILE_PC_POWER_UP      = 2,
    MOBILE_ROBOT_POWER_OFF  = 3,
    MOBILE_ROBOT_INITIALIZE = 4,
    MOBILE_NORMAL_OP        = 5,
    MOBILE_NORMAL_OP_AUTO   = 6,
    MOBILE_NORMAL_OP_MANUAL = 7,
    MOBILE_NORMAL_LOW_BAT   = 8,
    MOBILE_OPERATIONL_STOP  = 9,
    MOBILE_CHARGING         = 10,
    MOBILE_CONFIGURATION    = 11,
};

enum ROBOT_INITIALZATION_STATE
{
    MOBILE_RI_IDLE          = 0,
    MOBILE_RI_SAFETY_CHECK  = 1,
    MOBILE_RI_POWER_ON      = 2,
    MOBILE_RI_POWER_CHECK   = 3,
    MOBILE_RI_MOTOR_INIT    = 4,
    MOBILE_RI_MOTOR_CHECK   = 5,
    MOBILE_RI_DONE          = 6,
    MOBILE_RI_FAIL          = 7,
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
    DRIVING_FINE      = 2,
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
    ROBOT_AUTO_MODE     =0,
    ROBOT_JOYSTICK_MODE =1,
};

enum ROBOT_OPERATION_STATE
{
    //for safety
    SM_OM_POWER_OFF         = 0,
    SM_OM_MAIN_POWER_UP     = 1,
    SM_OM_PC_POWER_UP       = 2,
    SM_OM_ROBOT_POWER_OFF   = 3,
    SM_OM_ROBOT_INITIALIZE  = 4,
    SM_OM_NORMAL_OP         = 5,
    SM_OM_NORMAL_OP_AUTO    = 6,
    SM_OM_NORMAL_OP_MANUAL  = 7,
    SM_OM_NORMAL_LOW_BAT    = 8,
    SM_OM_OPERATIONAL_STOP  = 9,
    SM_OM_CHARGING          = 10,
    SM_OM_CONFIGURATION     = 11,
};

enum CHARGING_STATION_STATE
{
    CHARGING_STATION_IDLE                 = 0,
    CHARGING_STATION_TRIG_TO_CHARGE       = 1,
    CHARGING_STATION_BATTERY_ON           = 2,
    CHARGING_STATION_CHARGING             = 3,
    CHARGING_STATION_TRIG_TO_STOP_CHARGE  = 4,
    CHARGING_STATION_FAIL                 = 5,
};

enum SEMIAUTOINIT_STATE
{
    NONE         = 0,
    INITIALIZING = 1,
    COMPLETE     = 2
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

// structure
struct TIME_IMG
{
    double t = 0;
    cv::Mat img;

    TIME_IMG()
    {

    }

    TIME_IMG(const TIME_IMG& p)
    {
        t = p.t;
        img = p.img.clone();
    }

    TIME_IMG& operator=(const TIME_IMG& p)
    {
        t = p.t;
        img = p.img.clone();
        return *this;
    }
};

struct CAM_INTRINSIC
{
    int w = 0;
    int h = 0;

    double fx = 0;
    double fy = 0;
    double cx = 0;
    double cy = 0;

    double k1 = 0;
    double k2 = 0;
    double k3 = 0;
    double k4 = 0;
    double k5 = 0;
    double k6 = 0;
    double p1 = 0;
    double p2 = 0;

    int coef_num = 0;

    CAM_INTRINSIC()
    {

    }

    CAM_INTRINSIC(const CAM_INTRINSIC& p)
    {
        w = p.w;
        h = p.h;

        fx = p.fx;
        fy = p.fy;
        cx = p.cx;
        cy = p.cy;

        k1 = p.k1;
        k2 = p.k2;
        k3 = p.k3;
        k4 = p.k4;
        k5 = p.k5;
        k6 = p.k6;
        p1 = p.p1;
        p2 = p.p2;

        coef_num = p.coef_num;
    }

    CAM_INTRINSIC& operator=(const CAM_INTRINSIC& p)
    {
        w = p.w;
        h = p.h;

        fx = p.fx;
        fy = p.fy;
        cx = p.cx;
        cy = p.cy;

        k1 = p.k1;
        k2 = p.k2;
        k3 = p.k3;
        k4 = p.k4;
        k5 = p.k5;
        k6 = p.k6;
        p1 = p.p1;
        p2 = p.p2;

        coef_num = p.coef_num;
        return *this;
    }
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

struct PICKING
{
    bool l_drag;
    Eigen::Vector3d l_pt0;
    Eigen::Vector3d l_pt1;
    Eigen::Vector3d l_pose;

    bool r_drag;
    Eigen::Vector3d r_pt0;
    Eigen::Vector3d r_pt1;
    Eigen::Vector3d r_pose;

    // node
    QString pre_node;
    QString cur_node;

    int last_btn = 0; // 0: left, 1: right

    PICKING()
    {
        l_drag = false;
        l_pt0.setZero();
        l_pt1.setZero();
        l_pose.setZero();

        r_drag = false;
        r_pt0.setZero();
        r_pt1.setZero();
        r_pose.setZero();

        pre_node = "";
        cur_node = "";

        last_btn = 0;
    }

    PICKING(const PICKING& p)
    {
        l_drag = p.l_drag;
        l_pt0 = p.l_pt0;
        l_pt1 = p.l_pt1;
        l_pose = p.l_pose;

        r_drag = p.r_drag;
        r_pt0 = p.r_pt0;
        r_pt1 = p.r_pt1;
        r_pose = p.r_pose;

        pre_node = p.pre_node;
        cur_node = p.cur_node;

        last_btn = p.last_btn;
    }

    PICKING& operator=(const PICKING& p)
    {
        l_drag = p.l_drag;
        l_pt0 = p.l_pt0;
        l_pt1 = p.l_pt1;
        l_pose = p.l_pose;

        r_drag = p.r_drag;
        r_pt0 = p.r_pt0;
        r_pt1 = p.r_pt1;
        r_pose = p.r_pose;

        pre_node = p.cur_node;
        cur_node = p.pre_node;

        last_btn = p.last_btn;
        return *this;
    }
};

struct MOBILE_SETTING
{
    // for safety setting
    unsigned int version = 0;
    unsigned char robot_type = 0;

    float v_limit =.0;
    float w_limit =.0;
    float a_limit =.0;
    float b_limit =.0;

    float v_limit_jog = .0;
    float w_limit_jog = .0;
    float a_limit_jog = .0;
    float b_limit_jog = .0;

    float v_limit_monitor = .0;
    float w_limit_monitor = .0;

    float safety_v_limit =.0;
    float safety_w_limit =.0;

    float w_s = .0;
    float w_r = .0;
    float gear = .0;
    float dir = .0;

    unsigned char d_out[16] = {0,};
};

struct MOBILE_STATUS
{
    // for timesync
    double t = 0;
    float  return_time = 0;
    uint32_t recv_tick = 0;

    /* motor status */

    // motor connetion
    uint8_t connection_m0 = 0;
    uint8_t connection_m1 = 0;
    uint8_t connection_m2 = 0;
    uint8_t connection_m3 = 0;

    // motor status
    uint8_t status_m0 = 0;
    uint8_t status_m1 = 0;
    uint8_t status_m2 = 0;
    uint8_t status_m3 = 0;

    // motor temperature (using inlier sensor)
    uint8_t temp_m0 = 0;
    uint8_t temp_m1 = 0;
    uint8_t temp_m2 = 0;
    uint8_t temp_m3 = 0;

    // motor temperature (estimation)
    uint8_t esti_temp_m0 = 0;
    uint8_t esti_temp_m1 = 0;
    uint8_t esti_temp_m2 = 0;
    uint8_t esti_temp_m3 = 0;

    // motor core temperature (estimation)
    float core_temp0 = 0;
    float core_temp1 = 0;
    uint8_t state = 0;      // S100 state

    // motor current Amphere
    uint8_t cur_m0 = 0;
    uint8_t cur_m1 = 0;
    uint8_t cur_m2 = 0;
    uint8_t cur_m3 = 0;

    // PDU default state
    uint8_t charge_state     = 0;
    uint8_t power_state      = 0;
    uint8_t motor_stop_state = 0;
    uint8_t remote_state     = 0;

    // battery
    float power = 0;
    float bat_in = 0;
    float bat_out = 0;
    float bat_current = 0;
    float bat_voltage = 0;
    float total_power = 0;
    uint8_t bat_percent = 0;

    // lift (extra module)
    float lift_voltage_in = 0.;
    float lift_voltage_out = 0.;
    float lift_current = 0.;

    // docking charging
    float charge_current  = 0;  // current charging Amphere
    float contact_voltage = 0; // current charging voltage (docking)

    // imu status
    float imu_gyr_x =0;
    float imu_gyr_y =0;
    float imu_gyr_z =0;
    float imu_acc_x =0;
    float imu_acc_y =0;
    float imu_acc_z =0;

    // inter lock
    uint8_t inter_lock_state = 0;

    // for safety
    uint8_t sw_stop  = 0;
    uint8_t sw_reset = 0;
    uint8_t sw_start = 0;
    uint8_t om_state = 0;
    uint8_t ri_state = 0;
    uint8_t lidar_field;
    uint8_t bumper_state     = 0;
    uint8_t auto_manual_sw   = 0;
    uint8_t brake_release_sw = 0;


    uint8_t safety_state_emo_pressed_1          = 0;
    uint8_t safety_state_ref_meas_mismatch_1    = 0;
    uint8_t safety_state_over_speed_1           = 0;
    uint8_t safety_state_obstacle_detected_1    = 0;
    uint8_t safety_state_speed_field_mismatch_1 = 0;
    uint8_t safety_state_interlock_stop_1       = 0;
    uint8_t safety_state_bumper_stop_1          = 0;
    uint8_t operational_stop_state_flag_1       = 0;

    uint8_t safety_state_emo_pressed_2          = 0;
    uint8_t safety_state_ref_meas_mismatch_2    = 0;
    uint8_t safety_state_over_speed_2           = 0;
    uint8_t safety_state_obstacle_detected_2    = 0;
    uint8_t safety_state_speed_field_mismatch_2 = 0;
    uint8_t safety_state_interlock_stop_2       = 0;
    uint8_t safety_state_bumper_stop_2          = 0;
    uint8_t operational_stop_state_flag_2       = 0;

    short ref_dps[2]  = {0,0};
    short meas_dps[2] = {0,0};

    unsigned char mcu0_dio[8] ={0,};
    unsigned char mcu1_dio[8] ={0,};

    unsigned char mcu0_din[8] ={0,};
    unsigned char mcu1_din[8] ={0,};

    unsigned char adc_value[4] ={0,};
    unsigned char dac_value[4] ={0,};

    float tabos_rc           = 0.f; // remain capacity -ah
    float tabos_ae           = 0.f; // availiable energy -wh
    float tabos_voltage      = 0.f; // v
    float tabos_current      = 0.f; // a
    float tabos_temperature  = 0.;  // battery temperature-c
    uint16_t tabos_status    = 0;
    unsigned short tabos_ttf = 0;   // time to full-min
    unsigned short tabos_tte = 0;   // time to empty-min
    unsigned char tabos_soc  = 0;   // state of charge-%
    unsigned char tabos_soh  = 0;   // state of health-%

    uint8_t bms_type         = 0;

};

struct MOBILE_POSE
{
    double t;

    Eigen::Vector3d pose; // global pose(x, y, rz)
    Eigen::Vector3d vel; // local vel(vx, vy, wz)

    MOBILE_POSE()
    {
        t = 0;
        pose.setZero();
        vel.setZero();
    }

    MOBILE_POSE(const MOBILE_POSE& p)
    {
        t = p.t;
        pose = p.pose;
        vel = p.vel;
    }

    MOBILE_POSE& operator=(const MOBILE_POSE& p)
    {
        t = p.t;
        pose = p.pose;
        vel = p.vel;
        return *this;
    }
};

struct MOBILE_IMU
{
    double t = 0;
    double acc_x = 0;
    double acc_y = 0;
    double acc_z = 0;
    double gyr_x = 0;
    double gyr_y = 0;
    double gyr_z = 0;
    double rx = 0;
    double ry = 0;
    double rz = 0;
};

struct PT_XYZR
{
    int k       = 0;       // add cnt of tree
    int k0      = 0;      // original add cnt
    int do_cnt  = 0;  // dynamic object count
    double x    = 0;    // x coordinates
    double y    = 0;    // y coordinates
    double z    = 0;    // z coordinates
    double vx   = 0;   // view vector x
    double vy   = 0;   // view vector y
    double vz   = 0;   // view vector z
    double r    = 0;    // reflect 0~1
};

struct PT_SURFEL
{
    int lb    = 0;   // 0:none, 1:travel
    double x  = 0.0;
    double y  = 0.0;
    double z  = 0.0;
    double r  = 0.0;
    double nx = 0.0;
    double ny = 0.0;
    double nz = 0.0;
};

struct RAW_FRAME
{
    double t0 = 0;
    double t1 = 0;

    MOBILE_POSE mo;                   // mobile pose at t0
    Eigen::Vector3d pose0;
    Eigen::Vector3d pose1;
    std::vector<double> times;
    std::vector<double> reflects;
    std::vector<Eigen::Vector3d> pts; // deskewed local pts

    RAW_FRAME()
    {
    }
    RAW_FRAME(const RAW_FRAME& p)
    {
        t0 = p.t0;
        t1 = p.t1;
        pose0 = p.pose0;
        pose1 = p.pose1;
        times = p.times;
        reflects = p.reflects;
        pts = p.pts;
        mo = p.mo;
    }
    RAW_FRAME& operator=(const RAW_FRAME& p)
    {
        t0 = p.t0;
        t1 = p.t1;
        pose0 = p.pose0;
        pose1 = p.pose1;
        times = p.times;
        reflects = p.reflects;
        pts = p.pts;
        mo = p.mo;
        return *this;
    }
};

struct FRAME
{
    double t = 0;    
    MOBILE_POSE mo;                     // mobile pose at t
    std::vector<double> reflects;
    std::vector<Eigen::Vector3d> pts;

    FRAME()
    {
    }
    FRAME(const FRAME& p)
    {
        t = p.t;        
        reflects = p.reflects;
        pts = p.pts;
        mo = p.mo;
    }
    FRAME& operator=(const FRAME& p)
    {
        t = p.t;        
        reflects = p.reflects;
        pts = p.pts;
        mo = p.mo;
        return *this;
    }
};

struct KFRAME
{
    int id = 0;
    Eigen::Matrix4d G;
    Eigen::Matrix4d opt_G;
    std::vector<PT_XYZR> pts;

    KFRAME()
    {
        id = 0;
        G.setIdentity();
        opt_G.setIdentity();
    }

    KFRAME(const KFRAME& p)
    {
        id = p.id;
        pts = p.pts;
        G = p.G;
        opt_G = p.opt_G;
    }

    KFRAME& operator=(const KFRAME& p)
    {
        id = p.id;
        pts = p.pts;
        G = p.G;
        opt_G = p.opt_G;
        return *this;
    }
};

struct XYZR_CLOUD
{
    std::vector<PT_XYZR> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0) return pts[idx].x;
        else if (dim == 1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional: computation of bounding box
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

struct XYZ_NODE
{
    std::vector<Eigen::Vector3d> pos;
    std::vector<QString> id;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pos.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if (dim == 0) return pos[idx][0];
        else if (dim == 1) return pos[idx][1];
        else return pos[idx][2];
    }

    // Optional: computation of bounding box
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, XYZ_NODE>, XYZ_NODE, 3> KD_TREE_NODE;

struct COST_JACOBIAN
{
    double c = 0;
    double w = 0;
    double J[12] = {0,};

    COST_JACOBIAN(){}

    COST_JACOBIAN(const COST_JACOBIAN& p)
    {
        c = p.c;
        w = p.w;
        memcpy(J, p.J, sizeof(double)*12);
    }

    COST_JACOBIAN& operator=(const COST_JACOBIAN& p)
    {
        c = p.c;
        w = p.w;
        memcpy(J, p.J, sizeof(double)*12);
        return *this;
    }
};

struct TIME_POSE
{
    double t = 0;
    Eigen::Matrix4d tf;    

    TIME_POSE()
    {
        t = 0;
        tf.setIdentity();        
    }

    TIME_POSE(const TIME_POSE& p)
    {
        t = p.t;
        tf = p.tf;        
    }

    TIME_POSE& operator=(const TIME_POSE& p)
    {
        t = p.t;
        tf = p.tf;        
        return *this;
    }
};

struct TIME_PTS
{
    double t = 0;
    std::vector<Eigen::Vector3d> pts;

    TIME_PTS()
    {
        t = 0;
    }

    TIME_PTS(const TIME_PTS& p)
    {
        t = p.t;
        pts = p.pts;
    }

    TIME_PTS& operator=(const TIME_PTS& p)
    {
        t = p.t;
        pts = p.pts;
        return *this;
    }
};

struct TIME_POSE_PTS
{
    double t = 0;
    Eigen::Matrix4d tf;    
    std::vector<Eigen::Vector3d> pts;

    TIME_POSE_PTS()
    {
        t = 0;
        tf.setIdentity();
    }

    TIME_POSE_PTS(const TIME_POSE_PTS& p)
    {
        t = p.t;
        tf = p.tf;        
        pts = p.pts;
    }

    TIME_POSE_PTS& operator=(const TIME_POSE_PTS& p)
    {
        t = p.t;
        tf = p.tf;        
        pts = p.pts;
        return *this;
    }
};

struct TIME_POSE_ID
{
    int id = 0;
    double t = 0;
    Eigen::Matrix4d tf;

    TIME_POSE_ID()
    {
        t = 0;
        id = 0;
        tf.setIdentity();
    }

    TIME_POSE_ID(const TIME_POSE_ID& p)
    {
        t = p.t;
        id = p.id;
        tf = p.tf;
    }

    TIME_POSE_ID& operator=(const TIME_POSE_ID& p)
    {
        t = p.t;
        id = p.id;
        tf = p.tf;
        return *this;
    }
};

// topomap node
struct NODE
{
    QString id;
    QString name;
    QString type;                   // ROUTE, GOAL, OBS, ZONE
    QString info;                   // additional info
    Eigen::Matrix4d tf;             // node tf
    std::vector<QString> linked;

    NODE()
    {
        id = "";
        name = "";
        type = "";
        info = "";
        tf.setIdentity();
        linked.clear();        
    }

    NODE(const NODE& p)
    {
        id = p.id;
        name = p.name;
        type = p.type;
        info = p.info;
        tf = p.tf;
        linked = p.linked;        
    }

    NODE& operator=(const NODE& p)
    {
        id = p.id;
        name = p.name;
        type = p.type;
        info = p.info;
        tf = p.tf;
        linked = p.linked;
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

// autocontrol parameters
struct CTRL_PARAM
{
    double ST_V          =  0.05;
    double ED_V          =  0.05;

    double LIMIT_V       =  1.0;
    double LIMIT_W       =  30.0;
    double LIMIT_V_ACC   =  0.3;
    double LIMIT_V_DCC   =  0.3;
    double LIMIT_W_ACC   =  180.0;
    double LIMIT_PIVOT_W =  30.0;

    double DRIVE_T       =  0.0;
    double DRIVE_H       =  4.0;
    double DRIVE_A       =  0.9;
    double DRIVE_B       =  0.03;
    double DRIVE_L       =  0.4;
    double DRIVE_K       =  1.0;
    double DRIVE_EPS     =  0.3;
};

// dockcontrol parameters
struct DCTRL_PARAM
{
    double GAIN_P      = 1.0;
    double GAIN_D      = 0.0;
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

    PATH()
    {
        t = 0;
        ed_tf.setIdentity();
        is_final = false;
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

// bottom qr sensor(GLS611) info
struct BQR_INFO
{
    QString id = "";
    int code_num = -1;
    int t = 0;
    int x = 0;
    int y = 0;
    int z = 0;
    double th = 0;

    double xmcl = 0.0;
    double ymcl = 0.0;

    BQR_INFO()
    {
        id = "";
        code_num = -1;
        t = 0;
        x = 0;
        y = 0;
        z = 0;
        th = 0;
        xmcl = 0.0;
        ymcl = 0.0;
    }
    BQR_INFO(const BQR_INFO& p)
    {
        id = p.id;
        code_num = p.code_num;
        t = p.t;
        x = p.x;
        y = p.y;
        z = p.z;
        th = p.th;
        xmcl = p.xmcl;
        ymcl = p.ymcl;
    }
    BQR_INFO& operator=(const BQR_INFO& p)
    {
        id = p.id;
        code_num = p.code_num;
        t = p.t;
        x = p.x;
        y = p.y;
        z = p.z;
        th = p.th;
        xmcl = p.xmcl;
        ymcl = p.ymcl;
        return *this;
    }
};

struct CPU_USAGE
{
    long long user = 0;
    long long nice = 0;
    long long system = 0;
    long long idle = 0;
    long long iowait = 0;
    long long irq = 0;
    long long softirq = 0;

    CPU_USAGE()
    {
        user = 0;
        nice = 0;
        system = 0;
        idle = 0;
        iowait = 0;
        irq = 0;
        softirq = 0;
    }

    CPU_USAGE(const CPU_USAGE& p)
    {
        user = p.user;
        nice = p.nice;
        system = p.system;
        idle = p.idle;
        iowait = p.iowait;
        irq = p.irq;
        softirq = p.softirq;
    }

    CPU_USAGE& operator=(const CPU_USAGE& p)
    {
        user = p.user;
        nice = p.nice;
        system = p.system;
        idle = p.idle;
        iowait = p.iowait;
        irq = p.irq;
        softirq = p.softirq;
        return *this;
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

struct COPY_INFO
{
    QString id = "";
    int original_idx = -1;
    std::vector<int> original_links;

    COPY_INFO()
    {
        id = "";
        original_idx = -1;
    }

    COPY_INFO(const COPY_INFO& p)
    {
        id = p.id;
        original_idx = p.original_idx;
        original_links = p.original_links;
    }

    COPY_INFO& operator=(const COPY_INFO& p)
    {
        id = p.id;
        original_idx = p.original_idx;
        original_links = p.original_links;
        return *this;
    }
};

struct NODE_INFO
{
    QString info;

    Eigen::Vector3d sz;

    bool slow;
    bool fast;
    bool warning_beep;
    bool ignore_2d;
    bool ignore_3d;
    bool ignore_cam;
    bool ignore_obs_2d;
    bool ignore_obs_3d;
    bool ignore_obs_cam;

    NODE_INFO()
    {
        info = "";

        sz.setZero();

        slow           = false;
        fast           = false;
        warning_beep   = false;
        ignore_2d      = false;
        ignore_3d      = false;
        ignore_cam     = false;
        ignore_obs_2d  = false;
        ignore_obs_3d  = false;
        ignore_obs_cam = false;
    }

    NODE_INFO(const NODE_INFO& p)
    {
        info = p.info;

        sz = p.sz;

        slow           = p.slow;
        fast           = p.fast;
        warning_beep   = p.warning_beep;
        ignore_2d      = p.ignore_2d;
        ignore_3d      = p.ignore_3d;
        ignore_cam     = p.ignore_cam;
        ignore_obs_2d  = p.ignore_obs_2d;
        ignore_obs_3d  = p.ignore_obs_3d;
        ignore_obs_cam = p.ignore_obs_cam;
    }

    NODE_INFO& operator=(const NODE_INFO& p)
    {
        info = p.info;

        sz = p.sz;

        slow           = p.slow;
        fast           = p.fast;
        warning_beep   = p.warning_beep;
        ignore_2d      = p.ignore_2d;
        ignore_3d      = p.ignore_3d;
        ignore_cam     = p.ignore_cam;
        ignore_obs_2d  = p.ignore_obs_2d;
        ignore_obs_3d  = p.ignore_obs_3d;
        ignore_obs_cam = p.ignore_obs_cam;

        return *this;
    }
};

struct LINK_INFO
{
    QString st_id;
    QString ed_id;

    Eigen::Vector3d st;
    Eigen::Vector3d ed;
    Eigen::Vector3d mid;

    double length = 0.0;

    QString info;

    LINK_INFO()
    {
        st.setZero();
        ed.setZero();
        mid.setZero();

        length = 0.0;
    }

    LINK_INFO(const LINK_INFO& p)
    {
        st_id = p.st_id;
        ed_id = p.ed_id;

        st = p.st;
        ed = p.ed;
        mid = p.mid;

        length = p.length;

        info = p.info;
    }

    LINK_INFO& operator=(const LINK_INFO& p)
    {
        st_id = p.st_id;
        ed_id = p.ed_id;

        st = p.st;
        ed = p.ed;
        mid = p.mid;

        length = p.length;

        info = p.info;

        return *this;
    }
};


// tree typedef
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, XYZR_CLOUD>, XYZR_CLOUD, 3> KD_TREE_XYZR;

// for livox
struct IMU
{
    double t = 0; // sec

    double rx = 0; // so3 vector
    double ry = 0;
    double rz = 0;

    double acc_x = 0; // m/s^2
    double acc_y = 0;
    double acc_z = 0;

    double gyr_x = 0; // rad/s
    double gyr_y = 0;
    double gyr_z = 0;

    IMU()
    {

    }

    IMU(const IMU& p)
    {
        t = p.t;
        acc_x = p.acc_x;
        acc_y = p.acc_y;
        acc_z = p.acc_z;
        gyr_x = p.gyr_x;
        gyr_y = p.gyr_y;
        gyr_z = p.gyr_z;
        rx = p.rx;
        ry = p.ry;
        rz = p.rz;
    }

    IMU& operator=(const IMU& p)
    {
        t = p.t;
        acc_x = p.acc_x;
        acc_y = p.acc_y;
        acc_z = p.acc_z;
        gyr_x = p.gyr_x;
        gyr_y = p.gyr_y;
        gyr_z = p.gyr_z;
        rx = p.rx;
        ry = p.ry;
        rz = p.rz;
        return *this;
    }
};

struct LVX_PT
{
    double t       = 0;   // sec
    double alpha   = 0;   // rate in frm
    double reflect = 0;   // reflect
    uint8_t tag    = 0;   // tag for noise filtering

    float x = 0;          // float for octree
    float y = 0;
    float z = 0;

    LVX_PT()
    {

    }

    LVX_PT(const LVX_PT& p)
    {
        t = p.t;
        alpha = p.alpha;
        reflect = p.reflect;
        tag = p.tag;
        x = p.x;
        y = p.y;
        z = p.z;
    }

    LVX_PT& operator=(const LVX_PT& p)
    {
        t = p.t;
        alpha = p.alpha;
        reflect = p.reflect;
        tag = p.tag;
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }
};

struct LVX_FRM
{
    double t;
    std::vector<LVX_PT> pts;

    LVX_FRM()
    {
        t = 0;
    }

    LVX_FRM(const LVX_FRM& p)
    {
        t = p.t;
        pts = p.pts;
    }

    LVX_FRM& operator=(const LVX_FRM& p)
    {
        t = p.t;
        pts = p.pts;
        return *this;
    }
};

struct CLOUD
{
    std::vector<Eigen::Vector3d> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the dim'th component of the idx'th point in the class:
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
        if(dim == 0)
        {
            return pts[idx][0];
        }
        else if(dim == 1)
        {
            return pts[idx][1];
        }
        else
        {
            return pts[idx][2];
        }
    }

    // Optional: computation of bounding box
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
};
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, CLOUD>, CLOUD, 3> KD_TREE;


struct MonitoringField
{
    int monitor_id;                          
    double min_x, max_x, min_y, max_y;     
    bool is_blocked;                        
    
    MonitoringField() : monitor_id(-1),
                  min_x(0), max_x(0), min_y(0), max_y(0), 
                  is_blocked(false) {}
    
    MonitoringField(const MonitoringField& p)
    {
        monitor_id = p.monitor_id;
        min_x = p.min_x;
        max_x = p.max_x;
        min_y = p.min_y;
        max_y = p.max_y;
        is_blocked = p.is_blocked;
    }
    
    MonitoringField& operator=(const MonitoringField& p)
    {
        monitor_id = p.monitor_id;
        min_x = p.min_x;
        max_x = p.max_x;
        min_y = p.min_y;
        max_y = p.max_y;
        is_blocked = p.is_blocked;
        return *this;
    }
};

#endif // GLOBAL_DEFINES_H
