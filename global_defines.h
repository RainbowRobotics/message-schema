#ifndef GLOBAL_DEFINES_H
#define GLOBAL_DEFINES_H

// STL
#include <stdio.h>
#include <vector>
#include <memory>
#include <chrono>
#include <algorithm>
#include <unordered_map>
#include <mutex>
#include <thread>

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






#endif // GLOBAL_DEFINES_H
