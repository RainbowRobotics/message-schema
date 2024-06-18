#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H

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

// tbb
#include <tbb/concurrent_queue.h>

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
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// sophus
#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>
#include <sophus/interpolate.hpp>

// opencv
#include <opencv2/opencv.hpp>
#include "cv_to_qt.h"

// nanoflann
#include "nanoflann.hpp"

// pseudo color
#include "tinycolormap.hpp"

// qt
#include <QApplication>
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

// defines
#define ACC_G 9.80665
#define N2S (1.0e-9) // nanosec to sec
#define S2N (1.0e9) // sec to nanosec
#define U2S (1.0e-6) // microsec to sec
#define S2U (1.0e6) // sec to microsec
#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)
#define toWrap(rad) (std::atan2(std::sin(rad), std::cos(rad)))
#define deltaRad(ed,st) (std::atan2(std::sin(ed - st), std::cos(ed - st)))

#define MO_STORAGE_NUM 300
#define POINT_PLOT_SIZE 3
#define VIRTUAL_OBS_SIZE 0.3

#define GLOBAL_PATH_STEP 0.1
#define LOCAL_PATH_STEP 0.01

// enumulator
enum AUTO_FSM_STATE
{
    AUTO_FSM_FIRST_ALIGN = 0,
    AUTO_FSM_DRIVING,
    AUTO_FSM_APPROACH,
    AUTO_FSM_FINAL_ALIGN,
    AUTO_FSM_COMPLETE,
};

enum LOCAL_PATH_STATE
{
    LOCAL_PATH_IDLE = 0,
    LOCAL_PATH_REQUEST,
    LOCAL_PATH_BUSY,
};

// structure
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

struct MOBILE_STATUS
{
    double t = 0;

    // motor status
    uint8_t connection_m0 = 0;
    uint8_t connection_m1 = 0;

    uint8_t status_m0 = 0;
    uint8_t status_m1 = 0;

    uint8_t temp_m0 = 0;
    uint8_t temp_m1 = 0;

    uint8_t temp_ex_m0 = 0;
    uint8_t temp_ex_m1 = 0;

    uint8_t cur_m0 = 0;
    uint8_t cur_m1 = 0;

    uint8_t charge_state = 0;
    uint8_t power_state = 0;
    uint8_t emo_state = 0;
    uint8_t remote_state = 0;

    float bat_in = 0;
    float bat_out = 0;
    float bat_current = 0;
    float power = 0;
    float total_power = 0;

    // for timesync
    uint32_t recv_tick = 0;
    float return_time = 0;

    // roller status
    uint8_t roller_controller_state = 0;
    uint8_t roller_sensor0 = 0;
    uint8_t roller_sensor1 = 0;
    uint8_t roller_sensor2 = 0;
    uint8_t roller_sensor3 = 0;
    uint8_t roller_manual_sw0 = 0;
    uint8_t roller_manual_sw1 = 0;
    uint8_t roller_blocking_state0 = 0;
    uint8_t roller_blocking_state1 = 0;
    uint8_t roller_blocking_manual_sw0 = 0;
    uint8_t roller_blocking_manual_sw1 = 0;
    uint8_t roller_blocking_manual_sw2 = 0;
    uint8_t roller_blocking_manual_sw3 = 0;
    uint8_t orgo_on_init = 0;
    uint8_t orgo_on_run = 0;
    uint8_t orgo_pos_state0 = 0;
    uint8_t orgo_pos_state1 = 0;
    uint8_t orgo_manual_sw0 = 0;
    uint8_t orgo_manual_sw1 = 0;
    uint32_t orgo_pos0 = 0;
    uint32_t orgo_pos1 = 0;

    // imu status
    float imu_gyr_x =0;
    float imu_gyr_y =0;
    float imu_gyr_z =0;
    float imu_acc_x =0;
    float imu_acc_y =0;
    float imu_acc_z =0;
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
    double x = 0;    // x coordinates
    double y = 0;    // y coordinates
    double z = 0;    // z coordinates
    double vx = 0;   // view vector x
    double vy = 0;   // view vector y
    double vz = 0;   // view vector z
    double r = 0;    // reflect 0~1
    int k0 = 0;      // original add cnt
    int k = 0;       // add cnt of tree
    int do_cnt = 0;  // dynamic object count
};

struct PT_SURFEL
{
    double x = 0;
    double y = 0;
    double z = 0;
    double nx = 0;
    double ny = 0;
    double nz = 0;
    double r = 0;
    int lb = 0; // 0:none, 1:travel
};

struct CAM_DEPTH_FRAME
{
    double t = 0;
    std::vector<Eigen::Vector3d> pts;
    MOBILE_POSE mo; // mobile pose at t

    CAM_DEPTH_FRAME()
    {
    }
    CAM_DEPTH_FRAME(const CAM_DEPTH_FRAME& p)
    {
        t = p.t;
        pts = p.pts;
        mo = p.mo;
    }
    CAM_DEPTH_FRAME& operator=(const CAM_DEPTH_FRAME& p)
    {
        t = p.t;
        pts = p.pts;
        mo = p.mo;
        return *this;
    }
};

struct CAM_IMG_FRAME
{
    double t = 0;
    cv::Mat img;
    MOBILE_POSE mo; // mobile pose at t0

    CAM_IMG_FRAME()
    {
    }
    CAM_IMG_FRAME(const CAM_IMG_FRAME& p)
    {
        t = p.t;
        img = p.img.clone();
        mo = p.mo;
    }
    CAM_IMG_FRAME& operator=(const CAM_IMG_FRAME& p)
    {
        t = p.t;
        img = p.img.clone();
        mo = p.mo;
        return *this;
    }
};

struct RAW_FRAME
{
    double t0 = 0;
    double t1 = 0;
    std::vector<double> times;
    std::vector<double> reflects;
    std::vector<Eigen::Vector3d> dsk; // deskewed local pts
    MOBILE_POSE mo; // mobile pose at t0

    RAW_FRAME()
    {
    }
    RAW_FRAME(const RAW_FRAME& p)
    {
        t0 = p.t0;
        t1 = p.t1;
        times = p.times;
        reflects = p.reflects;
        dsk = p.dsk;
        mo = p.mo;
    }
    RAW_FRAME& operator=(const RAW_FRAME& p)
    {
        t0 = p.t0;
        t1 = p.t1;
        times = p.times;
        reflects = p.reflects;
        dsk = p.dsk;
        mo = p.mo;
        return *this;
    }
};

struct FRAME
{
    double t = 0;    
    std::vector<double> reflects;
    std::vector<Eigen::Vector3d> pts;
    MOBILE_POSE mo; // mobile pose at t

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
    std::vector<PT_XYZR> pts;
    Eigen::Matrix4d G;
    Eigen::Matrix4d opt_G;

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

struct TIME_POSE_PTS
{
    double t = 0;
    Eigen::Matrix4d tf;
    Eigen::Matrix4d tf2; // optional tf
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
        tf2 = p.tf2;
        pts = p.pts;
    }

    TIME_POSE_PTS& operator=(const TIME_POSE_PTS& p)
    {
        t = p.t;
        tf = p.tf;
        tf2 = p.tf2;
        pts = p.pts;
        return *this;
    }
};

// topomap node
struct NODE
{
    QString id;
    QString name;
    QString type; // ROUTE, GOAL
    QString info; // Attribute
    Eigen::Matrix4d tf; // node tf
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
    double LIMIT_V = 1.0;
    double LIMIT_W = 50.0;
    double LIMIT_V_ACC = 0.5;
    double LIMIT_W_ACC = 360.0;
    double LIMIT_PIVOT_W = 30.0;
    double MIN_LD = 0.1;
    double MAX_LD = 1.0;
    double MIN_V = 0.05;
    double MIN_W = 10.0;
};

struct ASTAR_NODE
{
    ASTAR_NODE* parent = NULL;
    NODE* node = NULL;
    double g = 0;
    double h = 0;
    double f = 0;

    ASTAR_NODE()
    {
        parent = NULL;
        node = NULL;
        g = 0;
        h = 0;
        f = 0;
    }

    ASTAR_NODE(const ASTAR_NODE& p)
    {
        parent = p.parent;
        node = p.node;
        g = p.g;
        h = p.h;
        f = p.f;
    }

    ASTAR_NODE& operator=(const ASTAR_NODE& p)
    {
        parent = p.parent;
        node = p.node;
        g = p.g;
        h = p.h;
        f = p.f;
        return *this;
    }
};

struct PATH
{
    std::vector<Eigen::Matrix4d> pose;
    std::vector<Eigen::Vector3d> pos;
    std::vector<double> ref_v;
    Eigen::Matrix4d goal_tf;

    PATH()
    {
        goal_tf.setIdentity();        
    }

    PATH(const PATH& p)
    {
        pose = p.pose;
        pos = p.pos;        
        ref_v = p.ref_v;
        goal_tf = p.goal_tf;
    }

    PATH& operator=(const PATH& p)
    {
        pose = p.pose;
        pos = p.pos;
        ref_v = p.ref_v;
        goal_tf = p.goal_tf;
        return *this;
    }
};

struct CODE_INFO
{
    QString id;
    double xmcl;
    double ymcl;
    double tilt;
    int timestamp;
    int x;
    int y;
    int z;

    CODE_INFO()
    {
        id = "";
        xmcl = 0;
        ymcl = 0;
        tilt = 0;
        timestamp = 0;
        x = 0;
        y = 0;
        z = 0;
    }
    CODE_INFO(const CODE_INFO& p)
    {
        id = p.id;
        xmcl = p.xmcl;
        ymcl = p.ymcl;
        tilt = p.tilt;
        timestamp = p.timestamp;
        x = p.x;
        y = p.y;
        z = p.z;
    }
    CODE_INFO& operator=(const CODE_INFO& p)
    {
        id = p.id;
        xmcl = p.xmcl;
        ymcl = p.ymcl;
        tilt = p.tilt;
        timestamp = p.timestamp;
        x = p.x;
        y = p.y;
        z = p.z;
        return *this;
    }
};

// tree typedef
typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, XYZR_CLOUD>, XYZR_CLOUD, 3> KD_TREE_XYZR;

#endif // GLOBAL_DEFINES_H
