#ifndef EKF_H
#define EKF_H

// defines
#include "global_defines.h"
#include "my_utils.h"

#include <QObject>

class EKF : public QObject
{
public:
    explicit EKF(QObject *parent = nullptr);
    ~EKF();

    void init(const Eigen::Matrix4d& tf, double alpha);
    void reset();

    void predict(const Eigen::Matrix4d& odom_tf);
    Eigen::Matrix4d update_measurement();


private:

    // mutex
    std::mutex mtx;

    Eigen::Vector3d x_hat;          // state estimation (x, y, theta)


    Eigen::Matrix3d P_hat;          // covariance
    Eigen::Matrix4d prev_odom_tf;

    // process noise
    Eigen::Matrix3d M_k;

    // measurement nois
    Eigen::Matrix3d R_k;

};

#endif // EKF_H
