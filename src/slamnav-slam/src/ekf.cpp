#include "ekf.h"
namespace 
{
  const char* MODULE_NAME = "EKF";
}

EKF::EKF(QObject *parent) : QObject{parent}
{
  x_hat.setZero();
  P_hat.setIdentity();
  pre_mo_tf.setIdentity();
  cur_tf.setIdentity();
}

EKF::~EKF()
{

}

void EKF::init(const Eigen::Matrix4d& tf)
{
  // clear first
  reset();

  std::lock_guard<std::mutex> lock(mtx);

  // set state, covariance
  double x0  = tf(0,3);
  double y0  = tf(1,3);
  double th0 = std::atan2(tf(1,0), tf(0,0));

  x_hat = Eigen::Vector3d(x0, y0, th0);
  P_hat = Eigen::Matrix3d::Identity() * 1e-3;

  // set process noise, Q_k (odom)
  M_k = Eigen::Matrix3d::Zero();
  M_k(0,0) = 0.05 * 0.05;
  M_k(1,1) = 0.05 * 0.05;
  M_k(2,2) = (3.0 * D2R) * (3.0 * D2R);

  // set measurement noise, R_k (icp)
  R_k = Eigen::Matrix3d::Zero();
  R_k(0,0) = icp_err_xy * icp_err_xy;
  R_k(1,1) = icp_err_xy * icp_err_xy;
  R_k(2,2) = (icp_err_th * D2R) * (icp_err_th * D2R);

  has_pre_mo_tf = false;

  initialized = true;

  //printf("[EKF] init (%f, %f, %f)\n", x_hat[0], x_hat[1], x_hat[2]*R2D);
  spdlog::info("[EKF] init ({:.3f}, {:.3f}, {:.3f} )",x_hat[0], x_hat[1], x_hat[2]*R2D);
  //log_info("initialized");
}

void EKF::reset()
{
  x_hat.setZero();
  P_hat.setIdentity();
  pre_mo_tf.setIdentity();

  has_pre_mo_tf.store(false);
  initialized.store(false);
}

Eigen::Matrix4d EKF::get_cur_tf()
{
  std::lock_guard<std::mutex> lock(mtx);
  Eigen::Matrix4d res = se2_to_TF(x_hat);
  return res;
}

void EKF::predict(const Eigen::Matrix4d& odom_tf)
{
  if(!initialized.load())
  {
    return;
  }

  if(!has_pre_mo_tf)
  {
    pre_mo_tf = odom_tf;
    has_pre_mo_tf = true;
    return;
  }

  Eigen::Matrix4d delta_tf = pre_mo_tf.inverse() * odom_tf;

  // based on body
  double delta_x = delta_tf(0,3);
  double delta_y = delta_tf(1,3);
  double delta_th = std::atan2(delta_tf(1,0), delta_tf(0,0));

  std::lock_guard<std::mutex> lock(mtx);

  double th  = x_hat(2);
  double cos_th  = std::cos(th);
  double sin_th  = std::sin(th);

  // state prediction
  Eigen::Vector3d x_bar;
  x_bar(0) = x_hat(0) + delta_x*cos_th - delta_y*sin_th;
  x_bar(1) = x_hat(1) + delta_x*sin_th + delta_y*cos_th;
  x_bar(2) = std::atan2(std::sin(th + delta_th), std::cos(th + delta_th)); // wrap

  // jacobian F_k
  Eigen::Matrix3d F_k = Eigen::Matrix3d::Identity();
  F_k(0,2) = -delta_x*sin_th - delta_y*cos_th;
  F_k(1,2) =  delta_x*cos_th - delta_y*sin_th;

  // jacobian L_k
  Eigen::Matrix3d L_k;
  L_k << cos_th, -sin_th, 0,
       sin_th,  cos_th, 0,
       0,   0,    1;

  // Q_k
  Eigen::Matrix3d Q_k = L_k * M_k * L_k.transpose();

  // covariance prediction
  Eigen::Matrix3d P_bar = F_k * P_hat * F_k.transpose() + Q_k;

  // commit
  x_hat = x_bar;
  P_hat = P_bar;

  pre_mo_tf = odom_tf;

  // printf("[EKF] prediction: (%f, %f, %f)\n", x_hat[0], x_hat[1], x_hat[2]*R2D);
  spdlog::debug("[EKF] prediction: ({:.3f}, {:.3f}, {:.3f} )",x_hat[0], x_hat[1], x_hat[2]*R2D);
}

void EKF::estimate(const Eigen::Matrix4d& icp_tf, const Eigen::Vector2d& ieir)
{
  if(!initialized.load())
  {
    return;
  }

  // update measurement
  Eigen::Vector3d z_k;
  z_k(0) = icp_tf(0,3);
  z_k(1) = icp_tf(1,3);
  z_k(2) = std::atan2(icp_tf(1,0), icp_tf(0,0));

  std::lock_guard<std::mutex> lock(mtx);

  // innovation y = z - h(x_bar)
  Eigen::Vector3d y_k;
  y_k(0) = z_k(0) - x_hat(0);
  y_k(1) = z_k(1) - x_hat(1);
  y_k(2) = std::atan2(std::sin(z_k(2) - x_hat(2)), std::cos(z_k(2) - x_hat(2))); // wrap

  // todo adaptive R (ieir)


  // innovation covariance
  Eigen::Matrix3d S_k = P_hat + R_k;

  // outlier detection - norm check
  double y_norm_xy = Eigen::Vector2d(y_k(0), y_k(1)).norm();
  double y_th_deg = std::abs(y_k(2)) * R2D;
  if(y_norm_xy > 0.5)
  {
    printf("[EKF] ICP rejected by xy-norm: %.3f m\n", y_norm_xy);
    spdlog::warn("[EKF] ICP rejected by xy-norm: {:.3f} m", y_norm_xy);
    return;
  }
  if(y_th_deg > 10.0)
  {
    printf("[EKF] ICP rejected by theta: %.2f deg\n", y_th_deg);
    spdlog::warn("[EKF] ICP rejected by theta: {:.2f} deg", y_th_deg);
    return;
  }

  // outlier detection - mahalanobis distance check (99% gate)
  double d2 = y_k.transpose() * S_k.ldlt().solve(y_k);
  if(d2 > 11.34)
  {
    printf("[EKF] ICP rejected, mahalanobis=%.2f\n", d2);
    spdlog::warn("[EKF] ICP rejected, mahalanobis={:.2f}", d2);
    return;
  }

  // calc kalman gain
  Eigen::Matrix3d K_k = P_hat * S_k.ldlt().solve(Eigen::Matrix3d::Identity());

  // state estimation
  Eigen::Vector3d x_new = x_hat + K_k * y_k;
  x_new(2) = std::atan2(std::sin(x_new(2)), std::cos(x_new(2))); // wrap

  // error covariance update (joseph form)
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d I_minus_K = I3 - K_k;
  Eigen::Matrix3d P_new = I_minus_K * P_hat * I_minus_K.transpose() + K_k * R_k * K_k.transpose();

  P_new = 0.5 * (P_new + P_new.transpose());

  // commit
  x_hat = x_new;
  P_hat = P_new;

  // debug
  // printf("[EKF] estimation:(%.3f, %.3f, %.3f), innovation:(%.3f, %.3f, %.3f), K_k:(%.3f, %.3f, %.3f)\n",
  //      x_hat[0], x_hat[1], x_hat[2]*R2D, y_k[0], y_k[1], y_k[2]*R2D, K_k(0,0), K_k(1,1), K_k(2,2));
  spdlog::debug("[EKF] estimation: ({:.3f}, {:.3f}, {:.3f}), innovation: ({:.3f}, {:.3f}, {:.3f}), K_k: ({:.3f}, {:.3f}, {:.3f})",
       x_hat[0], x_hat[1], x_hat[2]*R2D, y_k[0], y_k[1], y_k[2]*R2D, K_k(0,0), K_k(1,1), K_k(2,2));
}
