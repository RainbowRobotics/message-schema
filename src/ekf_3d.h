#ifndef EKF_3D_H
#define EKF_3D_H

// defines
#include "global_defines.h"
#include "my_utils.h"

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

    // state estimation (x, y, z, rx, ry, rz)
    Eigen::Vector6d x_hat;

    // covariance estimation
    Eigen::Matrix6d P_hat = Eigen::Matrix6d::Identity();

    Eigen::Matrix4d pre_mo_tf = Eigen::Matrix4d::Identity();

    // process noise
    Eigen::Matrix6d M_k = Eigen::Matrix6d::Identity();

    // measurement nois
    double icp_err_xy = 0.01;
    double icp_err_th = 2.0;
    Eigen::Matrix6d R_k = Eigen::Matrix6d::Identity();

    // result
    Eigen::Matrix4d cur_tf = Eigen::Matrix4d::Identity();

    // flags
    std::atomic<bool> has_pre_mo_tf = {false};
};

#endif // EKF_3D_H
