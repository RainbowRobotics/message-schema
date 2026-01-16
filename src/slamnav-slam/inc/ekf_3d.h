#ifndef EKF_3D_H
#define EKF_3D_H

// defines
#include "slamnav_slam_types.h"
#include "my_utils.h"

#include "logger.h"

#include <QObject>

namespace Eigen
{
typedef Eigen::Matrix<double,6,1> Vector6d;
typedef Eigen::Matrix<double,6,6> Matrix6d;
}

class EKF_3D : public QObject
{
  Q_OBJECT
public:
  explicit EKF_3D(QObject *parent = nullptr);
  ~EKF_3D();

  void init(const TIME_POSE& tp);
  void reset();

  Eigen::Matrix4d get_cur_tf();
  TIME_POSE get_cur_tp();
  void set_process_noise_scale(double scale);

  void predict(const TIME_POSE& odom_tp);
  void estimate(const TIME_POSE& icp_tp);

  // flags
  std::atomic<bool> initialized = {false};

private:
  // mutex
  std::mutex mtx;

  // state estimation (x, y, z, rx, ry, rz)
  Eigen::Vector6d x_hat;

  // covariance estimation
  Eigen::Matrix6d P_hat = Eigen::Matrix6d::Identity();

  Eigen::Matrix4d pre_mo_tf = Eigen::Matrix4d::Identity();

  // process noise
  double q_scale = 1.0;
  Eigen::Matrix6d M_k = Eigen::Matrix6d::Identity();

  // measurement nois
  double icp_err_xy = 0.01;
  double icp_err_th = 2.0;
  Eigen::Matrix6d R_k = Eigen::Matrix6d::Identity();

  // result
  Eigen::Matrix4d cur_tf = Eigen::Matrix4d::Identity();
  TIME_POSE cur_tp;

  // flags
  std::atomic<bool> has_pre_mo_tf = {false};
};

#endif // EKF_3D_H
