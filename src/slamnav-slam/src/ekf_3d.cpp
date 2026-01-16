#include "ekf_3d.h"
namespace 
{
  const char* MODULE_NAME = "EKF_3D";
}

EKF_3D::EKF_3D(QObject *parent) : QObject{parent}
{
  x_hat.setZero();
  P_hat.setIdentity();
  pre_mo_tf.setIdentity();
  cur_tf.setIdentity();
}

EKF_3D::~EKF_3D()
{

}


void EKF_3D::init(const TIME_POSE& tp)
{
  // clear first
  reset();

  std::lock_guard<std::mutex> lock(mtx);

  // set state, covariance
  x_hat = TF_to_ZYX(tp.tf);
  P_hat = Eigen::Matrix6d::Identity() * 1e-3;

  // set process noise, Q_k (odom)
  M_k = Eigen::Matrix6d::Zero();
  M_k(0,0) = 0.05*0.05;
  M_k(1,1) = 0.05*0.05;
  M_k(2,2) = 0.02*0.02;
  M_k(3,3) = (0.5*D2R) * (0.5*D2R);
  M_k(4,4) = (0.5*D2R) * (0.5*D2R);
  M_k(5,5) = (3.0*D2R) * (3.0*D2R);

  // set measurement noise, R_k (icp)
  R_k = Eigen::Matrix6d::Zero();
  R_k(0,0) = 0.01*0.01;
  R_k(1,1) = 0.01*0.01;
  R_k(2,2) = 0.01*0.01;
  R_k(3,3) = (0.05*D2R) * (0.05*D2R);
  R_k(4,4) = (0.05*D2R) * (0.05*D2R);
  R_k(5,5) = (1.0*D2R)  * (1.0*D2R);

  cur_tp = tp;

  has_pre_mo_tf = false;

  initialized = true;

  //printf("[EKF_3D] init (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f )\n",
  //     x_hat[0], x_hat[1], x_hat[2], x_hat[3]*R2D, x_hat[4]*R2D, x_hat[5]*R2D);
  log_info("init ({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f} )",x_hat[0], x_hat[1], x_hat[2], x_hat[3]*R2D, x_hat[4]*R2D, x_hat[5]*R2D);
}
         
void EKF_3D::reset()
{
  x_hat.setZero();
  P_hat.setIdentity();
  pre_mo_tf.setIdentity();

  has_pre_mo_tf.store(false);
  initialized.store(false);

  //printf("[EKF_3D] reset\n");
  log_info("[EKF_3D] reset");
}

Eigen::Matrix4d EKF_3D::get_cur_tf()
{
  std::lock_guard<std::mutex> lock(mtx);
  Eigen::Matrix4d res = ZYX_to_TF(x_hat);
  return res;
}

TIME_POSE EKF_3D::get_cur_tp()
{
  std::lock_guard<std::mutex> lock(mtx);
  TIME_POSE res = cur_tp;
  return res;
}

void EKF_3D::set_process_noise_scale(double scale)
{
  std::lock_guard<std::mutex> lock(mtx);
  q_scale = scale;
}

void EKF_3D::predict(const TIME_POSE& odom_tp)
{
  if(!initialized.load())
  {
    return;
  }

  Eigen::Matrix4d odom_tf = odom_tp.tf;

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

  double x = x_hat(0);
  double y = x_hat(1);
  double z = x_hat(2);
  double phi = x_hat(3);
  double theta = x_hat(4);
  double psi0 = x_hat(5);
  double psi = psi0 + 0.5*delta_th;
  // double psi  = psi0;

  double cos_psi = std::cos(psi), sin_psi = std::sin(psi);
  double cos_theta = std::cos(theta), sin_theta = std::sin(theta);
  double cos_phi = std::cos(phi), sin_phi = std::sin(phi);

  // state prediction
  Eigen::Vector6d x_bar;
  x_bar(0) = x + (cos_psi*cos_theta)*delta_x + (cos_psi*sin_theta*sin_phi - sin_psi*cos_phi)*delta_y;
  x_bar(1) = y + (sin_psi*cos_theta)*delta_x + (sin_psi*sin_theta*sin_phi + cos_psi*cos_phi)*delta_y;
  x_bar(2) = z + (-sin_theta)*delta_x + (cos_theta*sin_phi)*delta_y;
  x_bar(3) = phi;
  x_bar(4) = theta;
  x_bar(5) = std::atan2(std::sin(psi0 + delta_th), std::cos(psi0 + delta_th));

  // jacobian F_k
  Eigen::Matrix6d F_k = Eigen::Matrix6d::Identity();
  F_k(0,3) = (cos_psi*sin_theta*cos_phi + sin_psi*sin_phi) * delta_y;
  F_k(0,4) = (-cos_psi*sin_theta) * delta_x + (cos_psi*cos_theta*sin_phi) * delta_y;
  F_k(0,5) = (-sin_psi*cos_theta) * delta_x + (-sin_psi*sin_theta*sin_phi - cos_psi*cos_phi) * delta_y;

  F_k(1,3) = (sin_psi*sin_theta*cos_phi - cos_psi*sin_phi) * delta_y;
  F_k(1,4) = (-sin_psi*sin_theta) * delta_x + (sin_psi*cos_theta*sin_phi) * delta_y;
  F_k(1,5) = ( cos_psi*cos_theta) * delta_x + ( cos_psi*sin_theta*sin_phi - sin_psi*cos_phi) * delta_y;

  F_k(2,3) = (cos_theta*cos_phi) * delta_y;
  F_k(2,4) = (-cos_theta) * delta_x + (-sin_theta*sin_phi) * delta_y;
  F_k(2,5) = 0.0;

  // jacobian L_k
  Eigen::Matrix6d L_k = Eigen::Matrix6d::Zero();
  L_k(0,0) = cos_psi*cos_theta;
  L_k(0,1) = cos_psi*sin_theta*sin_phi - sin_psi*cos_phi;
  L_k(0,2) = 0.0;

  L_k(1,0) = sin_psi*cos_theta;
  L_k(1,1) = sin_psi*sin_theta*sin_phi + cos_psi*cos_phi;
  L_k(1,2) = 0.0;

  L_k(2,0) = -sin_theta;
  L_k(2,1) = cos_theta*sin_phi;
  L_k(2,2) = 1.0;

  L_k(0,5) = 0.5 * ((-sin_psi*cos_theta) * delta_x + (-sin_psi*sin_theta*sin_phi - cos_psi*cos_phi) * delta_y);
  L_k(1,5) = 0.5 * (( cos_psi*cos_theta) * delta_x + ( cos_psi*sin_theta*sin_phi - sin_psi*cos_phi) * delta_y);
  L_k(2,5) = 0.0;

  L_k(3,3) = 1.0;
  L_k(4,4) = 1.0;
  L_k(5,5) = 1.0;

  // Q_k
  Eigen::Matrix6d Q_k = q_scale * (L_k * M_k * L_k.transpose());

  // covariance prediction
  Eigen::Matrix6d P_bar = F_k * P_hat * F_k.transpose() + Q_k;

  // commit
  x_hat = x_bar;
  P_hat = 0.5 * (P_bar + P_bar.transpose());
  // P_hat = P_bar;

  pre_mo_tf = odom_tf;

  TIME_POSE tp;
  tp.t = odom_tp.t;
  tp.tf = ZYX_to_TF(x_hat);

  cur_tp = tp;

  // printf("[EKF_3D] prediction: (%.3f, %.3f, %.3f, %.3f, %.3f, %.3f )\n",
  //      x_hat[0], x_hat[1], x_hat[2], x_hat[3]*R2D, x_hat[4]*R2D, x_hat[5]*R2D);
  spdlog::debug("[EKF_3D] prediction: ({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f} )", x_hat[0], x_hat[1], x_hat[2], x_hat[3]*R2D, x_hat[4]*R2D, x_hat[5]*R2D);
}

void EKF_3D::estimate(const TIME_POSE& icp_tp)
{
  if(!initialized.load())
  {
    return;
  }

  Eigen::Matrix4d icp_tf = icp_tp.tf;

  std::lock_guard<std::mutex> lock(mtx);

  // Rz(psi) * Ry(theta) * Rx(phi)
  double phi = x_hat(3);
  double theta = x_hat(4);
  double psi = x_hat(5);
  double cos_phi = std::cos(phi);
  double sin_phi = std::sin(phi);
  double cos_theta = std::cos(theta);
  double sin_theta = std::sin(theta);
  double cos_psi = std::cos(psi);
  double sin_psi = std::sin(psi);
  Eigen::Matrix3d Rx;
  Rx << 1, 0, 0,
        0,  cos_phi, -sin_phi,
        0,  sin_phi,  cos_phi;
  Eigen::Matrix3d Ry;
  Ry <<  cos_theta, 0, sin_theta,
             0,     1, 0,
        -sin_theta, 0, cos_theta;
  Eigen::Matrix3d Rz;
  Rz <<  cos_psi, -sin_psi, 0,
         sin_psi,  cos_psi, 0,
         0,    0,     1;
  Eigen::Matrix3d R_zyx = Rz * Ry * Rx;

  // innovation y = [ p_meas - p_hat ; Log(R_zyx^T * R_meas) ]
  Eigen::Vector6d y_k;
  y_k.block(0,0,3,1) = Eigen::Vector3d(icp_tf(0,3), icp_tf(1,3), icp_tf(2,3)) - x_hat.block(0,0,3,1);
  Sophus::SO3d R_err(R_zyx.transpose() * icp_tf.block(0,0,3,3));
  y_k.block(3,0,3,1) = R_err.log();

  // measurment jacobian
  Eigen::Matrix6d H = Eigen::Matrix6d::Zero();
  H.block(0,0,3,3) = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d E;
  E << 1.0, 0.0,      -sin_theta,
     0.0,  cos_phi,    sin_phi * cos_theta,
     0.0, -sin_phi,    cos_phi * cos_theta;
  H.block(3,3,3,3) =  E;

  // innovation covariance
  Eigen::Matrix6d S_k = H * P_hat * H.transpose() + R_k;
  Eigen::LLT<Eigen::Matrix6d> llt(S_k);
  if(llt.info() != Eigen::Success)
  {
    S_k.diagonal().array() += 1e-9; // jitter
    llt.compute(S_k);
  }

  // outlier detection - mahalanobis distance check (99% gate)
  double d2 = llt.matrixL().solve(y_k).squaredNorm();
  if(d2 > 16.81)
  {
    log_warn("ICP rejected, mahalanobis={:.2f}", d2);
    return;
  }

  // calc kalman gain
  Eigen::Matrix6d K_k = P_hat * H.transpose() * llt.solve(Eigen::Matrix6d::Identity());

  // state estimation
  auto wrap = [](double a){ return std::atan2(std::sin(a), std::cos(a)); };
  Eigen::Vector6d x_new = x_hat + K_k * y_k;
  x_new(3) = wrap(x_new(3));
  x_new(4) = wrap(x_new(4));
  x_new(5) = wrap(x_new(5));

  // error covariance update (joseph form)
  Eigen::Matrix6d I6 = Eigen::Matrix6d::Identity();
  Eigen::Matrix6d I_minus_K = I6 - K_k * H;
  Eigen::Matrix6d P_new = I_minus_K * P_hat * I_minus_K.transpose() + K_k * R_k * K_k.transpose();

  // commit
  x_hat = x_new;
  P_hat = 0.5 * (P_new + P_new.transpose());

  TIME_POSE tp;
  tp.t = icp_tp.t;
  tp.tf = ZYX_to_TF(x_hat);

  cur_tp = tp;

  // debug
  spdlog::debug("[EKF_3D] estimation: ({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}), innovation: ({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}), "
       "K_k: ({:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f}, {:.3f})",
       x_hat(0), x_hat(1), x_hat(2), x_hat(3)*R2D, x_hat(4)*R2D, x_hat(5)*R2D,
       y_k(0), y_k(1), y_k(2), y_k(3)*R2D, y_k(4)*R2D, y_k(5)*R2D,
       K_k(0,0), K_k(1,1), K_k(2,2), K_k(3,3), K_k(4,4), K_k(5,5));
  // printf("[EKF_3D] estimation:(%.3f, %.3f, %.3f, %.2f, %.2f, %.2f), innovation:(%.3f, %.3f, %.3f, %.2f, %.2f, %.2f), "
  //      "K_k:(%.3f, %.3f, %.3f, %.3f, %.3f, %.3f)\n",
  //      x_hat(0), x_hat(1), x_hat(2), x_hat(3)*R2D, x_hat(4)*R2D, x_hat(5)*R2D,
  //      y_k(0), y_k(1), y_k(2), y_k(3)*R2D, y_k(4)*R2D, y_k(5)*R2D,
  //      K_k(0,0), K_k(1,1), K_k(2,2), K_k(3,3), K_k(4,4), K_k(5,5));
}
