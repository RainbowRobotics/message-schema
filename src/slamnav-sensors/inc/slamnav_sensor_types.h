//
// Created by jy on 25. 12. 27..
//

#ifndef SLAMNAV2_SLAMNAV_SENSOR_TYPES_H
#define SLAMNAV2_SLAMNAV_SENSOR_TYPES_H

#include <QObject>
#include <QString>

#include <thread>
#include <atomic>
#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>

#include "slamnav_robot_types.h"

struct RAW_FRAME
{
  double t0 = 0;
  double t1 = 0;

  MOBILE_POSE mo;           // mobile pose at t0
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
  MOBILE_POSE mo;           // mobile pose at t
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
  double t     = 0;   // sec
  double alpha   = 0;   // rate in frm
  double reflect = 0;   // reflect
  uint8_t tag    = 0;   // tag for noise filtering

  float x = 0;      // float for octree
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

#endif //SLAMNAV2_SLAMNAV_SENSOR_TYPES_H