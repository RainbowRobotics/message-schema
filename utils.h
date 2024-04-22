#ifndef UTILS_H
#define UTILS_H

#include "global_defines.h"

extern double st_time_for_get_time;
double get_time();
QString get_time_str();

Eigen::Matrix4d ZYX_to_TF(double tx, double ty, double tz, double rx, double ry, double rz);
Eigen::Matrix4d POSE_to_TF(Sophus::Vector6d xi);



#endif // UTILS_H
