#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H

// stl
#include <stdio.h>
#include <vector>
#include <memory>
#include <chrono>
#include <algorithm>
#include <unordered_map>
#include <mutex>
#include <thread>

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

// sophus
#define SOPHUS_USE_BASIC_LOGGING
#include <sophus/geometry.hpp>
#include <sophus/se3.hpp>
#include <sophus/interpolate.hpp>

// qt
#include <QApplication>
#include <QDateTime>
#include <QDir>
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
#define N2S (1.0e-9)
#define S2N (1.0e9)
#define D2R (M_PI/180.0)
#define R2D (180.0/M_PI)
#define toWrap(rad) (std::atan2(std::sin(rad), std::cos(rad)))
#define deltaRad(ed,st) (std::atan2(std::sin(ed - st), std::cos(ed - st)))

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
    Eigen::Vector3d pose; // global (x, y, th)
    Eigen::Vector3d vel;  // global (x_dot, y_dot, th_dot)
    Eigen::Vector2d vw;   // local (v, w)

    MOBILE_POSE()
    {
        t = 0;
        pose.setZero();
        vel.setZero();
        vw.setZero();
    }

    MOBILE_POSE(const MOBILE_POSE& p)
    {
        t = p.t;
        pose = p.pose;
        vel = p.vel;
        vw = p.vw;
    }
};


#endif // GLOBAL_DEFINES_H
