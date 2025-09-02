#include "ekf.h"

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

void EKF::init(const Eigen::Matrix4d& tf, double alpha)
{
    std::lock_guard<std::mutex> lock(mtx);

    // set state, covariance
    double x0  = tf(0,3);
    double y0  = tf(1,3);
    double th0 = std::atan2(tf(1,0), tf(0,0));

    x_hat = Eigen::Vector3d(x0, y0, th0);
    P_hat = Eigen::Matrix3d::Identity() * 1e-3;

    // set process noise, Q_k (odom)
    M_k = Eigen::Matrix3d::Zero();
    M_k(0,0) = 0.01 * 0.01;
    M_k(1,1) = 0.01 * 0.01;
    M_k(2,2) = (3.0 * D2R) * (3.0 * D2R);

    // set measurement noise, R_k (icp)
    R_k = Eigen::Matrix3d::Zero();
    R_k(0,0) = 0.03 * 0.03;
    R_k(1,1) = 0.03 * 0.03;
    R_k(2,2) = (1.0 * D2R) * (1.0 * D2R);

    has_pre_mo_tf = false;

    initialized = true;
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

void EKF::set_pre_mo_tf(Eigen::Matrix4d tf)
{
    std::lock_guard<std::mutex> lock(mtx);
    pre_mo_tf = tf;
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
           0,       0,      1;

    // Q_k
    Eigen::Matrix3d Q_k = L_k * M_k * L_k.transpose();

    // covariance prediction
    Eigen::Matrix3d P_bar = F_k * P_hat * F_k.transpose() + Q_k;

    // commit
    x_hat = x_bar;
    P_hat = P_bar;

    pre_mo_tf = odom_tf;

    printf("[EKF] prediction: (%f, %f, %f)\n", x_hat[0], x_hat[1], x_hat[2]);
}

void EKF::estimate(const Eigen::Matrix4d& icp_tf)
{
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

    // innovation covariance
    Eigen::Matrix3d S_k = P_hat + R_k;

    // calc kalman gain
    // Eigen::Matrix3d K_k = P_hat * S_k.inverse();
    Eigen::Matrix3d K_k = P_hat * S_k.ldlt().solve(Eigen::Matrix3d::Identity());

    // state estimation
    Eigen::Vector3d x_new = x_hat + K_k * y_k;
    x_new(2) = std::atan2(std::sin(x_new(2)), std::cos(x_new(2))); // wrap


    // error covariance update (joseph form)
    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d I_minus_K = I3 - K_k;
    Eigen::Matrix3d P_new = I_minus_K * P_hat * I_minus_K.transpose() + K_k * R_k * K_k.transpose();

    // commit
    x_hat = x_new;
    P_hat = P_new;

    printf("[EKF] estimation: (%f, %f, %f), gain: %f\n", x_hat[0], x_hat[1], x_hat[2], K_k);
}
