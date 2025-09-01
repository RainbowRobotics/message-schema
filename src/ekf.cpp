#include "ekf.h"

EKF::EKF(QObject *parent) : QObject{parent}
{
    x_hat.setZero();
    P_hat.setIdentity();
    prev_odom_tf.setIdentity();
}

EKF::~EKF()
{

}

void EKF::init(const Eigen::Matrix4d& tf, double alpha)
{
    // set state, covarianc
    double x0  = tf(0,3);
    double y0  = tf(1,3);
    double th0 = std::atan2(tf(1,0), tf(0,0));

    x_hat = Eigen::Vector3d(x0, y0, th0);
    P_hat = Eigen::Matrix3d::Identity() * 1e-3;

    prev_odom_tf = tf;

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
}

void EKF::reset()
{
    x_hat.setZero();
    P_hat.setIdentity();
    prev_odom_tf.setIdentity();
}

void EKF::predict(const Eigen::Matrix4d& odom_tf)
{
    Eigen::Matrix4d delta_tf = prev_odom_tf.inverse() * odom_tf;

    // based on body
    double delta_x = delta_tf(0,3);
    double delta_y = delta_tf(1,3);
    double delta_th = std::atan2(delta_tf(1,0), delta_tf(0,0));

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

    // update
    x_hat = x_bar;
    P_hat = P_bar;

    prev_odom_tf = odom_tf;
}
