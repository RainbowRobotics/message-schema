#ifndef UTILS_H
#define UTILS_H

#include "global_defines.h"

extern double st_time_for_get_time;
double get_time();
QString get_time_str();

Eigen::Matrix4d ZYX_to_TF(double tx, double ty, double tz, double rx, double ry, double rz);
Eigen::Matrix4d ZYX_to_TF(QString str);
Eigen::Matrix4d ZYX_to_TF(Sophus::Vector6d zyx);
Sophus::Vector6d TF_to_ZYX(Eigen::Matrix4d tf);

Eigen::Vector3d TF_to_se2(Eigen::Matrix4d tf);
Eigen::Matrix4d se2_to_TF(Eigen::Vector3d xi);

Eigen::Matrix4d se3_to_TF(Sophus::Vector6d xi);
Sophus::Vector6d TF_to_se3(Eigen::Matrix4d tf);

Eigen::Matrix4d string_to_TF(QString str);
QString TF_to_string(Eigen::Matrix4d TF);
double calc_seg_dist(Eigen::Vector3d P0, Eigen::Vector3d P1, Eigen::Vector3d P);

Eigen::Matrix4d intp_tf(double alpha, Eigen::Matrix4d tf0, Eigen::Matrix4d tf1);
void refine_pose(Eigen::Matrix4d& G);
bool compare_view_vector(Eigen::Vector3d V0, const Eigen::Vector3d V1, double threshold);

std::vector<Eigen::Vector3d> transform_pts(std::vector<Eigen::Vector3d> &pts, Eigen::Matrix4d tf);

#endif // UTILS_H
