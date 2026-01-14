#ifndef EKF_H
#define EKF_H

// defines
#include "slamnav_slam_types.h"
#include "my_utils.h"

#include "logger.h"

#include <QObject>

class EKF : public QObject
{
public:
  explicit EKF(QObject *parent = nullptr);
  ~EKF();

  void init(const Eigen::Matrix4d& tf);
  void reset();

  Eigen::Matrix4d get_cur_tf();

  void predict(const Eigen::Matrix4d& odom_tf);
  void estimate(const Eigen::Matrix4d& icp_tf, const Eigen::Vector2d& ieir);

  // flags
  std::atomic<bool> initialized = {false};

private:
  // mutex
  std::mutex mtx;

  // state estimation (x, y, theta)
  Eigen::Vector3d x_hat;

  // covariance estimation
  Eigen::Matrix3d P_hat = Eigen::Matrix3d::Identity();

  Eigen::Matrix4d pre_mo_tf = Eigen::Matrix4d::Identity();

  // process noise
  Eigen::Matrix3d M_k = Eigen::Matrix3d::Identity();

  // measurement nois
  double icp_err_xy = 0.01;
  double icp_err_th = 2.0;
  Eigen::Matrix3d R_k = Eigen::Matrix3d::Identity();

  // result
  Eigen::Matrix4d cur_tf = Eigen::Matrix4d::Identity();

  // flags
  std::atomic<bool> has_pre_mo_tf = {false};


};

#endif // EKF_H
