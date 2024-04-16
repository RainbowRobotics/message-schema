#ifndef UTILS_CPP
#define UTILS_CPP

#include "utils.h"

Eigen::Matrix4d ZYX_to_TF(double tx, double ty, double tz, double rx, double ry, double rz)
{
    Eigen::Matrix4d mat;

    Eigen::Matrix3d Rx, Ry, Rz;
    Rx << 1,       0,        0,
          0, cos(rx), -sin(rx),
          0, sin(rx),  cos(rx);

    Ry << cos(ry), 0, sin(ry),
                0, 1,       0,
         -sin(ry), 0, cos(ry);

    Rz << cos(rz), -sin(rz), 0,
          sin(rz),  cos(rz), 0,
                0,        0, 1;

    Eigen::Matrix3d R = Rz * Ry * Rx;

    mat << R(0,0), R(0,1), R(0,2), tx,
           R(1,0), R(1,1), R(1,2), ty,
           R(2,0), R(2,1), R(2,2), tz,
                0,      0,      0,  1;

    return mat;
}

Eigen::Matrix4d POSE_to_TF(Sophus::Vector6d xi)
{
    return Sophus::SE3d::exp(xi).matrix();
}

#endif // UTILS_CPP
